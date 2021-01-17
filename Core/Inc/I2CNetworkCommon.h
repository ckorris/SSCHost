#pragma once

#include<stdio.h> 



enum CommandType{SendSampleParams, BeginSampling, CheckFinished, RequestData, Reset};

typedef struct SampleParams
{
	uint8_t DeviceCount;
	uint16_t BufferSize;
	uint8_t CycleCount;
	uint8_t DelayMS;
} sampleParams;

int CommandTypeBufferSize(enum CommandType cType);

//For commands that request a true/false value.
enum BooleanReturnValue{BadData = -2, Timeout = -1, False = 0, True = 1};
