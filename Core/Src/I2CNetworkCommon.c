
#include<stdio.h>

#include "I2CNetworkCommon.h"

int CommandTypeBufferSize(enum CommandType cType)
{
	switch(cType)
	{
	case SendSampleParams:
		return sizeof(sampleParams);
		//return 5;
	case BeginSampling:
		return 0;
	case CheckFinished:
		return 0;
	case RequestData:
		return 0;
	case Reset:
		return 0;
	default:
		return 0;
	}
}


