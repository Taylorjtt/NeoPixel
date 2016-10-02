/*
 * hal.h
 *
 *  Created on: Mar 11, 2016
 *      Author: JohnTaylor
 */

#ifndef TORQUETESTBEDEMBEDDED_HAL_H_
#define TORQUETESTBEDEMBEDDED_HAL_H_

#include "hal_obj.h"
#include "../Configuration.h"
#include <stdbool.h>
#include "../f2806_headers/F2806x_Gpio.h"
#define LEFT_I2C_ADDRESS 0x21
#define RIGHT_I2C_ADDRESS 0x20
#define RIGHT_QUAD_CHANNEL_NUMBER 0
#define RIGHT_HAM_CHANNEL_NUMBER 1
#define RIGHT_GLUTE_CHANNEL_NUMBER 2

#define LEFT_QUAD_CHANNEL_NUMBER 0
#define LEFT_HAM_CHANNEL_NUMBER 1
#define LEFT_GLUTE_CHANNEL_NUMBER 2
extern float actualErgometerCurrent;
extern float actualErgometerSpeed;

#define MOTOR_DUTY_TO_COUNTS 20.0

#pragma CODE_SECTION(HAL_setupFlash,"ramfuncs");
#pragma CODE_SECTION(auxiliaryInterrupt,"ramfuncs");
#pragma CODE_SECTION(HAL_sendNeoBit,"ramfuncs");
 extern char testXH;
 extern char testXL;
 extern char testYH;
 extern char testYL;
 typedef enum {LOW_MUSCLE_GROUP_LEVEL,MED_MUSCLE_GROUP_LEVEL,HIGH_MUSCLE_GROUP_LEVEL}MUSCLE_GROUP_LEVEL;
typedef enum {RIGHT_QUAD, RIGHT_HAM, RIGHT_GLUTE, LEFT_QUAD, LEFT_HAM, LEFT_GLUTE,NO_CHANNEL}STIMULATION_CHANNEL;
typedef enum {LOW_SPASM,MED_SPASM,HIGH_SPASM}SPASM_LEVEL;
typedef enum {LEFT,RIGHT}LEG_SIDE;

#define Device_cal (void   (*)(void))0x3D7C80
//! \brief Defines used in oscillator calibration functions
//! \brief Defines the scale factor for Q15 fixed point numbers (2^15)
#define FP_SCALE 32768

//! \brief Defines the quantity added to Q15 numbers before converting to integer to round the number
#define FP_ROUND FP_SCALE/2

//! \brief Defines the amount to add to Q16.15 fixed point number to shift from a fine trim range of
//! \brief (-31 to 31) to (1 to 63).  This guarantees that the trim is positive and can
//! \brief therefore be efficiently rounded
#define OSC_POSTRIM 32
#define OSC_POSTRIM_OFF FP_SCALE*OSC_POSTRIM

//! \brief The following functions return reference values stored in OTP.

//! \brief Defines the slope used to compensate oscillator 1 (fine trim steps / ADC code). Stored in fixed point Q15 format
#define getOsc1FineTrimSlope() (*(int16_t (*)(void))0x3D7E90)()

//! \brief Defines the oscillator 1 fine trim at high temp
#define getOsc1FineTrimOffset() (*(int16_t (*)(void))0x3D7E93)()

//! \brief Defines the oscillator 1 coarse trim
#define getOsc1CoarseTrim() (*(int16_t (*)(void))0x3D7E96)()

//! \brief Defines the slope used to compensate oscillator 2 (fine trim steps / ADC code). Stored
//! \brief in fixed point Q15 format.
#define getOsc2FineTrimSlope() (*(int16_t (*)(void))0x3D7E99)()

//! \brief Defines the oscillator 2 fine trim at high temp
#define getOsc2FineTrimOffset() (*(int16_t (*)(void))0x3D7E9C)()

//! \brief Defines the oscillator 2 coarse trim
#define getOsc2CoarseTrim() (*(int16_t (*)(void))0x3D7E9F)()

//! \brief Defines the ADC reading of temperature sensor at reference temperature for compensation
#define getRefTempOffset() (*(int16_t (*)(void))0x3D7EA2)()

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
#define HAL_PWM_DBFED_CNT         1


//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define HAL_PWM_DBRED_CNT         1

extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);
void HAL_disableWdog(HAL_Handle halHandle);
void HAL_setupProcessor(HAL_Handle handle);
void HAL_setupClks(HAL_Handle handle);
void HAL_setupPll(HAL_Handle handle,const PLL_ClkFreq_e clkFreq);
void HAL_setupPie(HAL_Handle handle);
void HAL_cal(HAL_Handle handle);
void HAL_setupPeripheralClks(HAL_Handle handle);
void HAL_setupFlash(HAL_Handle handle);
void HAL_setupGpios(HAL_Handle handle);
void HAL_toggleDebugLED(HAL_Handle handle);
void HAL_osc1Comp(HAL_Handle handle, const int16_t sensorSample);
void HAL_osc2Comp(HAL_Handle handle, const int16_t sensorSample);
uint16_t HAL_getOscTrimValue(int16_t coarse, int16_t fine);
void HAL_sendRGB(float red, float green, float blue,float intensity);
inline void HAL_sendNeoBit(bool bit)
{
	if(bit)
	{
		 GpioDataRegs.GPASET.all |= 1;
		 _nop();
		 _nop();
		 _nop();
		 _nop();
		 _nop();
		 _nop();
		 GpioDataRegs.GPACLEAR.all |= 1;
		 _nop();

	}
	else
	{
		 GpioDataRegs.GPASET.all |= 1;
		 _nop();
		 GpioDataRegs.GPACLEAR.all |= 1;
		 _nop();
		 _nop();

	}

}
void HAL_resetAuxiliaryTimerFlag(HAL_Handle halHandle);
bool HAL_getAuxiliaryTimerFlag(HAL_Handle halHandle);
void HAL_enableLCDInterrupt(HAL_Handle halHandle,bool enable);
__interrupt void auxiliaryInterrupt(void);
__interrupt void neoPixelInterrupt(void);

#endif /* TORQUETESTBEDEMBEDDED_HAL_H_ */
