#include<stdio.h>

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

