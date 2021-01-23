#pragma once

#include<stdio.h> 



enum CommandType
{
	SendSampleParams,
	BeginSampling,
	CheckFinished,
	RequestSampleHeader,
	RequestSampleData
};

//For commands that request a true/false value.
enum BooleanReturnValue{BadData = -2, Timeout = -1, False = 0, True = 1};

typedef struct SampleParams
{
	uint8_t DeviceCount;
	uint16_t BufferSize;
	uint8_t CycleCount;
	uint8_t DelayMS;
} sampleParams;



typedef struct SamplePacketHeader
{
	int DeviceID;
	int SampleID;
	int32_t startTimeUs;
	int32_t endTimeUs;

	int AnalogInPinCount;

	int32_t SampleCount;

} samplePacketHeader;
