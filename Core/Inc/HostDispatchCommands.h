#pragma once

#include<stdio.h>
#include "stm32f7xx_hal.h"

#define COMMAND_SEND_TIMEOUT_MS 5 //When sending only the command enum (just one uint8_t) in non-blocking mode how long in ms to wait for timeout.

void SendSampleParamsCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint8_t deviceCount, uint16_t bufferSize, uint8_t cycleCount, uint8_t delayMS);

void BeginSamplingCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress);

enum BooleanReturnValue CheckFinishedCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress); //Returns 1 if ready, 0 otherwise. May need to check for individual packets.

void RequestDataCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint8_t* dataBuffer);

void ResetCommand(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress);
