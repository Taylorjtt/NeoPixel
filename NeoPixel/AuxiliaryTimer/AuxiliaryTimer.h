/*
 * Class.h
 *
 *  Created on: Aug 12, 2016
 *      Author: JohnTaylor
 */

#ifndef AUX_TIMER_H_
#define AUX_TIMER_H_

#include <stdlib.h>

#include "../HardwareAbstractionLayer/hal.h"
#include "../NeoPixelString/NeoPixelString.h"
#include "../AbstractDataTypeStructs.h"
extern bool sendNeoPixelBit;
typedef struct _AUXILIARY_TIMER_OBJ_
{
	int oneHzDecimationFactor;
	int fourHzDecimationFactor;
	int oneHzDecimatorCount;
	int fourHzDecimatorCount;
	int halfHzDecimatorCount;
	int halfHzDecimationFactor;
}AuxiliaryTimerObject;


extern AuxiliaryTimerHandle AuxiliaryTimer_Constructor(void *pmemory, const size_t numbytes);
void AuxiliaryTimer_service(AuxiliaryTimerHandle handle, HAL_Handle halHandle,NeoPixelStringHandle neoPixelString);
#endif /* TEMPLATES_CLASS_H_ */
