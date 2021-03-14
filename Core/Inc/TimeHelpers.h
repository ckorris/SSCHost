#pragma once

#include <stdio.h>
#include "stm32f7xx_hal.h"

//#define NANOSECOND_DIVIDER 1000000000

#define MICROSECOND_DIVIDER 1000000

#define MILLISECOND_DIVIDER 1000

uint32_t TicksToSubSecond(TIM_HandleTypeDef htim, uint32_t ticks, double fractionOfSecond); //fractionOfSecond, as in, 1000 for milliseconds.

uint32_t ReadCurrentTicks(TIM_HandleTypeDef htim);

void ResetClock(TIM_HandleTypeDef htim);
