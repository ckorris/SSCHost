#pragma once

#include<stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"

#define COMMAND_RECEIVE_READY_TIMEOUT_MS 5000 //When checking if sample is done on device (single uint8_t) how long to wait on timeout.

#define COMMAND_RECEIVE_HEADER_TIMEOUT_MS 5000 //When asking for a sample header, how long to wait on timeout.

enum BooleanReturnValue ReceiveFinishedStatus(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress); //1 for true, 0 for false, -1 for error. TODO: Make enum.

samplePacketHeader ReceiveSamplePacketHeader(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress);
