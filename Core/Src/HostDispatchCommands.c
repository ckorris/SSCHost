#include<stdio.h>

#include "HostDispatchCommands.h"
#include "HostReceiveCommands.h"
#include "I2CNetworkCommon.h"


//I2C

//Forward declarations for methods not in header file.
void SendCommandEnumOnly_Blocking(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, enum CommandType cType);
void SendCommandEnumOnly_NonBlocking(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, enum CommandType cType);

//Blocking. Could be non-blocking but it's not time-critical really, so not worth the complexity at this stage.
void SendSampleParamsCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint8_t cycleCount, uint8_t delayMS)
{
	//Tell the peripheral that we're about to send instructions for sample parameters.
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, SendSampleParams);

	//Make a packet that we'll send over.
	sampleParams packet;

	//packet.DeviceCount = deviceCount;
	//packet.BufferSize = bufferSize;
	packet.CycleCount = cycleCount;
	packet.DelayMS = delayMS;

	//Send it in blocking mode, so we get an answer before moving on and avoid getting our streams crossed.
	HAL_I2C_Master_Transmit(hi2c, peripheralAddress, &packet, sizeof(sampleParams), 10);
}


void BeginSamplingCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, BeginSampling);
}

enum BooleanReturnValue CheckFinishedCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, CheckFinished);

	return ReceiveFinishedStatus(hi2c, peripheralAddress);
}

uint16_t RequestTotalPacketCountCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, RequestTotalPacketCount);

	uint16_t totalPacketCount = ReceiveTotalPacketCount(hi2c, peripheralAddress);
	return totalPacketCount; //Could just return directly but this makes it easier to debug.
}

void RequestSampleHeaderCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, int sampleID, samplePacketHeader* header)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, RequestSampleHeader);

	//Send the ID of the sample that we want the header for.
	uint8_t idBuf[1] = { sampleID };
	HAL_I2C_Master_Transmit(hi2c, peripheralAddress, idBuf, 1, HAL_MAX_DELAY); //Timeout is arbitrary.

	//samplePacketHeader newHeader = ReceiveSamplePacketHeader(hi2c, peripheralAddress);
	ReceiveSamplePacketHeader(hi2c, peripheralAddress, header);
	//*header = newHeader;

}

void RequestSampleDataCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint16_t samplesPerDevice, int sampleID, uint16_t* dataBuffer)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, RequestSampleData);

	//Send the ID of the sample that we want the header for.
	uint8_t idBuf[1] = { sampleID };
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, peripheralAddress, idBuf, 1, HAL_MAX_DELAY); //Timeout is arbitrary.

	if(result != HAL_OK)
	{
		//TODO: Throw error.
		return;
	}

	//uint16_t* newData = ReceiveSamplePacketData(hi2c, peripheralAddress, samplesPerDevice);
	ReceiveSamplePacketData(hi2c, peripheralAddress, samplesPerDevice, dataBuffer);

	//dataBuffer = newData;
}





void SendCommandEnumOnly_Blocking(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, enum CommandType cType)
{
	uint8_t commandBuf[1] = { cType };
	HAL_I2C_Master_Transmit(hi2c, peripheralAddress, commandBuf, 1, COMMAND_SEND_TIMEOUT_MS);
}

void SendCommandEnumOnly_NonBlocking(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, enum CommandType cType)
{
	uint8_t commandBuf[1] = { cType };
	HAL_I2C_Master_Transmit_IT(hi2c, peripheralAddress, commandBuf, 1);
}

//Serial

const uint32_t MAGIC_VALUES[3] = {0x0, 0xFFFFFFFF, 0x7A7ABFBF}; //Values used to validate a sample packet on the PC.

const int TRANSMIT_COUNT = INT16_MAX; //Max samples that can be read and sent to PC via serial in a single operation.

void TransmitSamplePacketToPC(UART_HandleTypeDef *huart, samplePacketHeader header, uint16_t *samples)
{
	//Give the specific sequence that validates that this is the start of a packet, not additional data.
	HAL_UART_Transmit(huart, (uint8_t*)MAGIC_VALUES, sizeof(MAGIC_VALUES), HAL_MAX_DELAY);

	//Send header information.
	HAL_UART_Transmit(huart, (uint8_t*)&header, sizeof(samplePacketHeader), HAL_MAX_DELAY);

	//Send samples.
	uint8_t* start = (uint8_t*)samples; //Pointer to start of samples.

	for(int i = 0; i < header.SampleCount; i += TRANSMIT_COUNT) //We do this dance to prevent trying to send more data than we can fit.
	{
		int thisCycleCount = ((long)TRANSMIT_COUNT < header.SampleCount - i) ? (long)TRANSMIT_COUNT : header.SampleCount - i; //Could break this up but this is readable.
		HAL_UART_Transmit(huart, start, sizeof(uint16_t) * thisCycleCount, HAL_MAX_DELAY);
		start += sizeof(uint16_t) * thisCycleCount;
	}


}










