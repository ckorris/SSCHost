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

