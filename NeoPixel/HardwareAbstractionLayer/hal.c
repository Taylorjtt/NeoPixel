/*
 * hal.c
 *
 *  Created on: Mar 11, 2016
 *      Author: JohnTaylor
 */

#include "hal.h"
#include "hal_obj.h"

HAL_Obj hal;

HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{

  HAL_Handle handle;
  HAL_Obj *obj;


  if(numBytes < sizeof(HAL_Obj))
    return((HAL_Handle)NULL);

  // assign the handle
  handle = (HAL_Handle)pMemory;

  // assign the object
  obj = (HAL_Obj *)handle;

  // initialize the watchdog driver
  obj->wdogHandle = WDOG_init((void *)WDOG_BASE_ADDR,sizeof(WDOG_Obj));

  // initialize the clock handle
  obj->clkHandle = CLK_init((void *)CLK_BASE_ADDR,sizeof(CLK_Obj));

  // initialize the CPU handle
  obj->cpuHandle = CPU_init(&cpu,sizeof(cpu));

  // initialize the FLASH handle
  obj->flashHandle = FLASH_init((void *)FLASH_BASE_ADDR,sizeof(FLASH_Obj));

  // initialize the GPIO handle
  obj->gpioHandle = GPIO_init((void *)GPIO_BASE_ADDR,sizeof(GPIO_Obj));

  // initialize the oscillator handle
  obj->oscHandle = OSC_init((void *)OSC_BASE_ADDR,sizeof(OSC_Obj));

  // initialize the PIE handle
  obj->pieHandle = PIE_init((void *)PIE_BASE_ADDR,sizeof(PIE_Obj));

  obj->sciAHandle = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
  obj->sciBHandle = SCI_init((void *)SCIB_BASE_ADDR, sizeof(SCI_Obj));

  // initialize the PLL handle
  obj->pllHandle = PLL_init((void *)PLL_BASE_ADDR,sizeof(PLL_Obj));
  obj->spiBHandle = SPI_init((void *)SPIB_BASE_ADDR,sizeof(SPI_Obj));

  obj->pwmHandle[0] = PWM_init((void *)PWM_ePWM1_BASE_ADDR,sizeof(PWM_Obj));
  obj->pwmHandle[1] = PWM_init((void *)PWM_ePWM2_BASE_ADDR,sizeof(PWM_Obj));
  obj->pwmHandle[2] = PWM_init((void *)PWM_ePWM3_BASE_ADDR,sizeof(PWM_Obj));

  // initialize timer handles
  obj->timerHandle[0] = TIMER_init((void *)TIMER0_BASE_ADDR,sizeof(TIMER_Obj));
  obj->timerHandle[1] = TIMER_init((void *)TIMER1_BASE_ADDR,sizeof(TIMER_Obj));
  obj->timerHandle[2] = TIMER_init((void *)TIMER2_BASE_ADDR,sizeof(TIMER_Obj));

  obj->qepAHandle= QEP_init((void *)QEP1_BASE_ADDR, sizeof(QEP_Obj));
  obj->ecap1Handle = CAP_init((void *)CAP1_BASE_ADDR, sizeof(CAP_Obj));
  obj->neoBit = false;
  obj->neoBitDone =false;
  obj->auxTimerFlag = false;

  return(handle);

}
float averageGPIOFrequency  = 0;
/*
 * General Set-Up Functions
 */
void HAL_setupProcessor(HAL_Handle handle)
{
	 HAL_Obj *obj = (HAL_Obj *)handle;

	 CPU_disableGlobalInts(obj->cpuHandle);
	 // disable cpu interrupts
	 CPU_disableInts(obj->cpuHandle);
	 // clear cpu interrupt flags
	 CPU_clearIntFlags(obj->cpuHandle);
	 HAL_setupClks(handle);
	 HAL_setupPll(handle,PLL_ClkFreq_80_MHz);
	 HAL_setupPie(handle);
	 HAL_cal(handle);
	 HAL_setupPeripheralClks(handle);
	 HAL_setupGpios(handle);
	 HAL_setupFlash(handle);
	 //give set up time to finish
	 DELAY_US(100);
	 CPU_enableGlobalInts(obj->cpuHandle);
	 return;

}

void HAL_toggleDebugLED(HAL_Handle handle)
{
	 HAL_Obj *obj = (HAL_Obj *)handle;
	 GPIO_toggle(obj->gpioHandle, GPIO_Number_10);
}
void HAL_setupFlash(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  FLASH_enablePipelineMode(obj->flashHandle);

  FLASH_setNumPagedReadWaitStates(obj->flashHandle,FLASH_NumPagedWaitStates_3);

  FLASH_setNumRandomReadWaitStates(obj->flashHandle,FLASH_NumRandomWaitStates_3);

  FLASH_setOtpWaitStates(obj->flashHandle,FLASH_NumOtpWaitStates_5);

  FLASH_setStandbyWaitCount(obj->flashHandle,FLASH_STANDBY_WAIT_COUNT_DEFAULT);

  FLASH_setActiveWaitCount(obj->flashHandle,FLASH_ACTIVE_WAIT_COUNT_DEFAULT);

  return;
} // HAL_setupFlash() function
void HAL_setupGpios(HAL_Handle handle)
{
	 HAL_Obj *obj = (HAL_Obj *)handle;

	// JTAG
	GPIO_setMode(obj->gpioHandle,GPIO_Number_35,GPIO_35_Mode_JTAG_TDI);
	GPIO_setMode(obj->gpioHandle,GPIO_Number_36,GPIO_36_Mode_JTAG_TMS);
	GPIO_setMode(obj->gpioHandle,GPIO_Number_37,GPIO_37_Mode_JTAG_TDO);
	GPIO_setMode(obj->gpioHandle,GPIO_Number_38,GPIO_38_Mode_JTAG_TCK);


	GPIO_setMode(obj->gpioHandle,GPIO_Number_0,GPIO_0_Mode_GeneralPurpose);
	GPIO_setDirection(obj->gpioHandle,GPIO_Number_0,GPIO_Direction_Output);



	 return;
}
void HAL_setupPeripheralClks(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  //motor PWM
  CLK_enablePwmClock(obj->clkHandle,PWM_Number_3);

  CLK_disableHrPwmClock(obj->clkHandle);

  //stimulation channel
  CLK_enableI2cClock(obj->clkHandle);

  CLK_disableLinAClock(obj->clkHandle);
  CLK_disableClaClock(obj->clkHandle);

  //wifi uart
  CLK_enableSciaClock(obj->clkHandle);

  //lcd uart
  CLK_enableScibClock(obj->clkHandle);

  //current sensor
  CLK_enableSpibClock(obj->clkHandle);

  //encoder
  CLK_enableEqep1Clock(obj->clkHandle);

  //GPIO capture
  CLK_enableEcap1Clock(obj->clkHandle);

  CLK_enableTbClockSync(obj->clkHandle);
  CLK_enableEcanaClock(obj->clkHandle);

  return;
} // end of HAL_setupPeripheralClks() function
void HAL_cal(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // enable the ADC clock
  CLK_enableAdcClock(obj->clkHandle);


  // Run the Device_cal() function
  // This function copies the ADC and oscillator calibration values from TI reserved
  // OTP into the appropriate trim registers
  // This boot ROM automatically calls this function to calibrate the interal
  // oscillators and ADC with device specific calibration data.
  // If the boot ROM is bypassed by Code Composer Studio during the development process,
  // then the calibration must be initialized by the application
  ENABLE_PROTECTED_REGISTER_WRITE_MODE;
  (*Device_cal)();
  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  // disable the ADC clock
  CLK_disableAdcClock(obj->clkHandle);

  return;
} // end of HAL_cal() function

void HAL_sendRGB(float red, float green, float blue,float intensity)
{
	Uint8 green8 = intensity*green*2.55;
	Uint8 red8 = intensity*red*2.55;
	Uint8 blue8 = intensity*blue*2.55;

	HAL_sendNeoBit(green8>>7 & 1);
	HAL_sendNeoBit(green8>>6 & 1);
	HAL_sendNeoBit(green8>>5 & 1);
	HAL_sendNeoBit(green8>>4 & 1);
	HAL_sendNeoBit(green8>>3 & 1);
	HAL_sendNeoBit(green8>>2 & 1);
	HAL_sendNeoBit(green8>>1 & 1);
	HAL_sendNeoBit(green8>>0 & 1);

	HAL_sendNeoBit(red8>>7 & 1);
	HAL_sendNeoBit(red8>>6 & 1);
	HAL_sendNeoBit(red8>>5 & 1);
	HAL_sendNeoBit(red8>>4 & 1);
	HAL_sendNeoBit(red8>>3 & 1);
	HAL_sendNeoBit(red8>>2 & 1);
	HAL_sendNeoBit(red8>>1 & 1);
	HAL_sendNeoBit(red8>>0 & 1);

	HAL_sendNeoBit(blue8>>7 & 1);
	HAL_sendNeoBit(blue8>>6 & 1);
	HAL_sendNeoBit(blue8>>5 & 1);
	HAL_sendNeoBit(blue8>>4 & 1);
	HAL_sendNeoBit(blue8>>3 & 1);
	HAL_sendNeoBit(blue8>>2 & 1);
	HAL_sendNeoBit(blue8>>1 & 1);
	HAL_sendNeoBit(blue8>>0 & 1);
}
void HAL_setupPie(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  PIE_disable(obj->pieHandle);

  PIE_disableAllInts(obj->pieHandle);

  PIE_clearAllInts(obj->pieHandle);

  PIE_clearAllFlags(obj->pieHandle);

  PIE_setDefaultIntVectorTable(obj->pieHandle);

  PIE_enable(obj->pieHandle);



  /*
   * set up the Aux timer interrupt
   */

  CLK_enableCpuTimerClock(obj->clkHandle, CLK_CpuTimerNumber_2);
  CPU_enableInt(obj->cpuHandle, CPU_IntNumber_14);
  TIMER_enableInt(obj->timerHandle[2]);
  TIMER_setEmulationMode(obj->timerHandle[2], TIMER_EmulationMode_RunFree);

  TIMER_setPeriod(obj->timerHandle[2],2.666666e6);	//initialize to 30hz
  PIE_registerSystemIntHandler(obj->pieHandle, PIE_SystemInterrupts_TINT2, (PIE_IntVec_t) &auxiliaryInterrupt);
  TIMER_setPreScaler(obj->timerHandle[2],0);
  TIMER_start(obj->timerHandle[2]);


  return;
} // end of HAL_setupPie() function

void HAL_osc2Comp(HAL_Handle handle, const int16_t sensorSample)
{
	int16_t compOscFineTrim;
	HAL_Obj *obj = (HAL_Obj *)handle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    compOscFineTrim = ((sensorSample - getRefTempOffset())*(int32_t)getOsc2FineTrimSlope()
                      + OSC_POSTRIM_OFF + FP_ROUND )/FP_SCALE + getOsc2FineTrimOffset() - OSC_POSTRIM;

    if(compOscFineTrim > 31)
      {
        compOscFineTrim = 31;
      }
	else if(compOscFineTrim < -31)
      {
        compOscFineTrim = -31;
      }

    OSC_setTrim(obj->oscHandle, OSC_Number_2, HAL_getOscTrimValue(getOsc2CoarseTrim(), compOscFineTrim));

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of HAL_osc2Comp() function
uint16_t HAL_getOscTrimValue(int16_t coarse, int16_t fine)
{
  uint16_t regValue = 0;

  if(fine < 0)
    {
      regValue = ((-fine) | 0x20) << 9;
    }
  else
    {
      regValue = fine << 9;
    }

  if(coarse < 0)
    {
      regValue |= ((-coarse) | 0x80);
    }
  else
    {
      regValue |= coarse;
    }

  return regValue;
} // end of HAL_getOscTrimValue() function
void HAL_setupClks(HAL_Handle handle)
{
	  HAL_Obj *obj = (HAL_Obj *)handle;


	  // enable internal oscillator 1
	  CLK_enableOsc1(obj->clkHandle);

	  // set the oscillator source
	  CLK_setOscSrc(obj->clkHandle,CLK_OscSrc_Internal);

	  // disable the external clock in
	  CLK_disableClkIn(obj->clkHandle);

	  // disable the crystal oscillator
	  CLK_disableCrystalOsc(obj->clkHandle);

	  // disable oscillator 2
	  CLK_disableOsc2(obj->clkHandle);

	  // set the low speed clock prescaler
	  CLK_setLowSpdPreScaler(obj->clkHandle,CLK_LowSpdPreScaler_SysClkOut_by_1);

	  // set the clock out prescaler
	  CLK_setClkOutPreScaler(obj->clkHandle,CLK_ClkOutPreScaler_SysClkOut_by_1);

	  return;
}
void HAL_setupPll(HAL_Handle handle,const PLL_ClkFreq_e clkFreq)
{
	HAL_Obj *obj = (HAL_Obj *)handle;


	  // make sure PLL is not running in limp mode
	  if(PLL_getClkStatus(obj->pllHandle) != PLL_ClkStatus_Normal)
	    {
	      // reset the clock detect
	      PLL_resetClkDetect(obj->pllHandle);
	      asm("        ESTOP0");
	    }


	  // Divide Select must be ClkIn/4 before the clock rate can be changed
	  if(PLL_getDivideSelect(obj->pllHandle) != PLL_DivideSelect_ClkIn_by_4)
	  {
	     PLL_setDivideSelect(obj->pllHandle,PLL_DivideSelect_ClkIn_by_4);
	  }


	  if(PLL_getClkFreq(obj->pllHandle) != clkFreq)
	    {
	      // disable the clock detect
	      PLL_disableClkDetect(obj->pllHandle);

	      // set the clock rate
	      PLL_setClkFreq(obj->pllHandle,clkFreq);
	    }


	  // wait until locked
	  while(PLL_getLockStatus(obj->pllHandle) != PLL_LockStatus_Done) {}


	  // enable the clock detect
	  PLL_enableClkDetect(obj->pllHandle);


	  // set divide select to ClkIn/2 to get desired clock rate
	  // NOTE: clock must be locked before setting this register
	  PLL_setDivideSelect(obj->pllHandle,PLL_DivideSelect_ClkIn_by_2);

	  return;
}

void HAL_disableWdog(HAL_Handle halHandle)
{
  HAL_Obj *obj = (HAL_Obj *)halHandle;


  WDOG_disable(obj->wdogHandle);


  return;
}


void HAL_resetAuxiliaryTimerFlag(HAL_Handle halHandle)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;
	obj->auxTimerFlag = false;
}
bool HAL_getAuxiliaryTimerFlag(HAL_Handle halHandle)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;
	return obj->auxTimerFlag;
}

__interrupt void auxiliaryInterrupt(void)
{
	hal.auxTimerFlag = true;
	EALLOW;
	CpuTimer2Regs.TCR.bit.TIF = 1;
	EDIS;
}
__interrupt void neoPixelInterrupt(void)
{
	EALLOW;
	CpuTimer0Regs.TCR.bit.TIF = 1;
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
	EDIS;
}



