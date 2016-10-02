/*
 * MyocycleHome.c
 *
 *  Created on: Aug 15, 2016
 *      Author: JohnTaylor
 */



#include "HardwareAbstractionLayer/hal.h"
#include "Util/memCopy.h"
#include "Util/constants.h"
#include "AuxiliaryTimer/AuxiliaryTimer.h"
#include <stdbool.h>
#include "NeoPixelString/NeoPixelString.h"

HAL_Handle halHandle;
AuxiliaryTimerHandle auxiliaryTimer;
NeoPixelStringHandle neoPixelString;
StimulationControllerHandle stimulationController;
bool sendNeoPixelBit =false;
bool bitToSend = false;
Uint8 amplitude = 20;

int main(void)
  {
	//copy fast-access functions to RAM
	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
	// initialize the hardware abstraction layer
	halHandle = HAL_init(&hal,sizeof(hal));
	HAL_setupProcessor(halHandle);
	DELAY_US(1e6);

	auxiliaryTimer = far_malloc(sizeof(AuxiliaryTimerObject));
	neoPixelString = far_malloc(sizeof(NeoPixelString));
	/*
	 *End dynamic memory allocation
	 */

	auxiliaryTimer = AuxiliaryTimer_Constructor((void *)auxiliaryTimer,sizeof(AuxiliaryTimerObject));
	neoPixelString = NeoPixelString_Constructor((void *)neoPixelString,sizeof(NeoPixelString),15);


	while(true)
	{

		AuxiliaryTimer_service(auxiliaryTimer,halHandle,neoPixelString);
	}


}
