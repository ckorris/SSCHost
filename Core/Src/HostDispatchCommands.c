#include<stdio.h>

#include "HostDispatchCommands.h"
#include "I2CNetworkCommon.h"

//Forward declarations for methods not in header file.
void SendCommandEnumOnly_Blocking(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, enum CommandType cType);
void SendCommandEnumOnly_NonBlocking(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, enum CommandType cType);

//Blocking. Could be non-blocking but it's not time-critical really, so not worth the complexity at this stage.
void SendSampleParamsCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint8_t deviceCount, uint16_t bufferSize, uint8_t cycleCount, uint8_t delayMS)
{
	//Tell the peripheral that we're about to send instructions for sample parameters.
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, SendSampleParams);

	//Make a packet that we'll send over.
	sampleParams packet;

	packet.DeviceCount = deviceCount;
	packet.BufferSize = bufferSize;
	packet.CycleCount = cycleCount;
	packet.DelayMS = delayMS;

	//Send it in blocking mode, so we get an answer before moving on and avoid getting our streams crossed.
	HAL_I2C_Master_Transmit(hi2c, peripheralAddress, &packet, CommandTypeBufferSize(SendSampleParams), 10);
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

void RequestDataCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint8_t* dataBuffer)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, RequestData);
}

void ResetCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress)
{
	SendCommandEnumOnly_Blocking(hi2c, peripheralAddress, Reset);
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


