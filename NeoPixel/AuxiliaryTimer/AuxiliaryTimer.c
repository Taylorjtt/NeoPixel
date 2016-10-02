/*
 * class.c
 *
 *  Created on: Aug 12, 2016
 *      Author: JohnTaylor
 */


#include "AuxiliaryTimer.h"

AuxiliaryTimerHandle AuxiliaryTimer_Constructor(void *pmemory, const size_t numbytes)
{
	AuxiliaryTimerHandle handle;
	AuxiliaryTimerObject *obj;

	if(numbytes < sizeof(AuxiliaryTimerObject))
	{
		return ((AuxiliaryTimerHandle)NULL);
	}

	handle = (AuxiliaryTimerHandle)pmemory;
	obj = (AuxiliaryTimerObject *)handle;
	obj->oneHzDecimationFactor = 15;
	obj->fourHzDecimationFactor = 1;
	obj->oneHzDecimatorCount = 0;
	obj->fourHzDecimatorCount = 0;
	obj->halfHzDecimationFactor = 59;
	obj->halfHzDecimatorCount = 0;
	return handle;
}

void AuxiliaryTimer_service(AuxiliaryTimerHandle handle, HAL_Handle halHandle,NeoPixelStringHandle neoPixelString)
{
	AuxiliaryTimerObject *obj = (AuxiliaryTimerObject *)handle;

	if(HAL_getAuxiliaryTimerFlag(halHandle))
	{

		HAL_resetAuxiliaryTimerFlag(halHandle);
		obj->oneHzDecimatorCount++;
		obj->fourHzDecimatorCount++;
		obj->halfHzDecimatorCount++;


		if(obj->fourHzDecimatorCount > obj->fourHzDecimationFactor)
		{
			//serviced at 2Hz

			obj->fourHzDecimatorCount = 0;
			NeoPixelString_rotate(neoPixelString);
			NeoPixelString_draw(neoPixelString);


		}
		if(obj->oneHzDecimatorCount > obj->oneHzDecimationFactor)
		{
			//serviced at 1Hz
			obj->oneHzDecimatorCount = 0;
			sendNeoPixelBit = true;

		}

		if(obj->halfHzDecimatorCount > obj->halfHzDecimationFactor)
		{
			obj->halfHzDecimatorCount = 0;
		}
	}
}


