#include<stdio.h>
#include <stdlib.h>

#include "HostReceiveCommands.h"

//Blocking for now.
enum BooleanReturnValue ReceiveFinishedStatus(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress) //1 for true, 0 for false, -1 for timeout, -2 for data being bad value. TODO: Make enum.
{
	HAL_StatusTypeDef returnStatus;

	uint8_t resultBuf[1];
	returnStatus = HAL_I2C_Master_Receive(hi2c, peripheralAddress, resultBuf, 1, COMMAND_RECEIVE_READY_TIMEOUT_MS);

	if(returnStatus != HAL_OK)
	{
		return -1;
	}

	uint8_t result = resultBuf[0];
	if(result == 1)
	{
		return 1;
	}
	else if (result == 0)
	{
		return 0;
	}
	else //We didn't time out, but some other garbage value got there somehow.
	{
		return -2;
	}
}

uint16_t ReceiveTotalPacketCount(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress)
{
	HAL_StatusTypeDef returnStatus;

	uint8_t resultBuf[sizeof(uint16_t)];
	returnStatus = HAL_I2C_Master_Receive(hi2c, peripheralAddress, resultBuf, sizeof(uint16_t), COMMAND_RECEIVE_COUNT_TIMEOUT_MS);

	if(returnStatus != HAL_OK) //Error. Skip this one.
 	{
		return 0;
	}

	uint16_t totalPacketCount = (uint16_t)resultBuf[0]; //Could just return directly but this makes it easier to debug.
	return totalPacketCount;
}

void ReceiveSamplePacketHeader(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, samplePacketHeader* header)
{
	uint16_t packetSize = sizeof(samplePacketHeader);
	uint8_t packetBuf[packetSize];
	//uint16_t packetBuf;

	HAL_I2C_Master_Receive(hi2c, peripheralAddress, packetBuf, packetSize, COMMAND_RECEIVE_HEADER_TIMEOUT_MS);

	samplePacketHeader newHeader = *((samplePacketHeader*)&packetBuf);
	*header = newHeader;
}

void ReceiveSamplePacketData(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint16_t samplesPerDevice, uint16_t* dataBuffer)
{

	//uint16_t* dataBuffer = malloc(sizeof(uint16_t) * samplesPerDevice); //Don't forget to free this somewhere.
	//HAL_I2C_Master_Receive(hi2c, peripheralAddress, (uint8_t*)dataBuffer, samplesPerDevice, COMMAND_RECEIVE_DATA_TIMEOUT_MS);
	HAL_I2C_Master_Receive(hi2c, peripheralAddress, (uint8_t*)dataBuffer, sizeof(uint16_t) * samplesPerDevice, COMMAND_RECEIVE_DATA_TIMEOUT_MS);

	//return dataBuffer;
}





