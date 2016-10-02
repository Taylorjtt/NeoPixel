#include "stdint.h"
#include "stdbool.h"
#include "debugFlags.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include "BSP/pwm/bsp_pwm.h"
#include "BSP/communications/bsp_sci.h"
#include "BSP/communications/bsp_spi.h"
#include "Util/adcHandler.h"
#include "BSP/adc/bsp_adc.h"
#include "Util/encoderHandler.h"
#include "Util/constants.h"
#include "BSP/Encoder/bsp_s4t_encoder.h"
#include "BSP/Processor/processor.h"
#include "BSP/GPIO/bsp_gpio.h"
#include "Stimulation/torqueTransferCalculator.h"
#include "BSP/i2c/i2c.h"
#include "BSP/gui/gui.h"
#include "BSP/gui/gui_defines.h"
#include "BSP/Wifi/Wifi.h"
#include "BSP/Wifi/Wifi_defines.h"
#include "globalVariables.h"
#include "Math/math.h"
#include "Util/Torque.h"
#include "Peripherial Drivers/cap.h"
#include "TorqueEstimation/WormGearMotorTorqueModel.h"
#include "Util/trackingLoop.h"

/*
 * I2C Addresses for the isolated processors
 */
#define LEFT_I2C_ADDRESS 0x21
#define RIGHT_I2C_ADDRESS 0x20

#define RIGHT_QUAD_CHANNEL_NUMBER 0
#define RIGHT_HAM_CHANNEL_NUMBER 1
#define RIGHT_GLUTE_CHANNEL_NUMBER 2

#define LEFT_QUAD_CHANNEL_NUMBER 0
#define LEFT_HAM_CHANNEL_NUMBER 1
#define LEFT_GLUTE_CHANNEL_NUMBER 2
#define RESET_CHANNEL_STATUS_NUMBER 3

#define MAX_PULSEWIDTH 500.0
#define MIN_PULSEWIDTH 30.0



CAP_Handle gpioCapture;
int16 streamPower = 0;
Uint8 numberOfSpasms = 0;
BSP_ADC_Handle bspAdc;
BSP_SPI_Handle spiHandle;
T_ADCHandler_Handle currentSensorHandler;
BSP_S4T_Encoder_Handle bspEncoder;
T_EncoderHandler_Handle encoderHandler;
MOTOR_FAULT_STATUS faultStatus = NO_FAULT;
estimator currentEstimator;
estimator torqueEstimator;
estimator velocityEstimator;
estimator symmetryEstimator;
uint16_t padsFlagCount = 0;
uint16_t maxPadsFlagCount = 1;
uint16_t torqueControlLoopCount = 0;
uint16_t velocityControlLoopCount = 0;
int TORQUE_LOOP_DIVIDER = 560;
int SPEED_LOOP_DIVIDER = 20;
float power = 0.0;
float averageActiveTorque = 0.0;
float averagePassiveTorque = 0.0;
float powerSum = 0.0;
float activePowerSum = 0.0;
float passiveTorqueSum = 0.0;
float activeTorque = 0;
float rampingSpasmCurrent =	6.8;
float workoutSpasmCurrent = 6.1;

float powerAverageCount = 0;
float warmupCurrent = 30;
bool logDataFlag = false;
bool checkLCDConnectionFlag = false;
float warmupPulseWidth = 30;


float quadCurrent = 90;
float hamCurrent = 50;
float gluteCurrent = 80;
float Kus = 50;
float testCurrent = 100;
float testPulseWidth = 500;
bool testingChannels = false;

int rightQuadPulseWidth = 0;
int leftQuadPulseWidth = 0;
int rightHamPulseWidth = 0;
int leftHamPulseWidth = 0;
int rightGlutePulseWidth = 0;
int leftGlutePulseWidth = 0;

bool rtcFlag = false;
bool firstEnabledControlLoop = true;
bool stimulationFlag = false;
bool rightQuadEnable = true;
bool rightHamEnable = true;
bool rightGluteEnable = true;
bool leftQuadEnable = true;
bool leftHamEnable = true;
bool leftGluteEnable = true;
bool controlFlag = false;


bool forceGlutes = false;
bool forceQuads = false;
bool forceHams = false;



int rightTorqueTransferIndex = 0;
int leftTorqueTransferIndex = 0;
uint16_t spasmCount = 0;
bool spasmFlag = false;
float spasmCurrent = 6.5;
bool enableStim = false;
bool disableStim = false;
float positionInDegrees = 0;
uint16_t rawMotorCurrentBytes = 0;
float motorCurrentVolts = 0.0;
float instantaneousCurrent = 0.0;
float position = 0.0;
volatile float dutyCycle = 50.0;


float speedFFWGain = 3.009;
float speedControlOut = 0.0;
volatile float speedFFW = 0.0;
PID_Handle speedControlPID;

float MAX_VEL_INTEGRATOR = 10;
float MIN_VEL_INTEGRATOR = -10;
float MAX_DUTY = 80.0;
float MIN_DUTY = 49.0;

float_t passiveTorques[360];
float_t cap1;
float_t cap2;
float_t cap3;
float_t cap4;
float averageGPIOFrequency = 0.0;

int main(void)
{
	initializeProcessor();
	CLK_enableTbClockSync(clock);
	initializeStimulationInterrupt();
	initializeRTCInterrupt();
	intializeControlLoopInterrupt();


	currentEstimator  = (estimator)
	{
		.deltaT = 5e-5,
	 	.estimate = 0.0,
	 	.estimateDot = 0.0,
	 	.estimateDotIntegrator = 0.0,
	 	.estimateError = 0.0,
	 	.ki = 5.0,
	 	.kp = 10
	};

	symmetryEstimator  = (estimator)
	{
		.deltaT = 0.55,
	 	.estimate = 0.0,
	 	.estimateDot = 0.0,
	 	.estimateDotIntegrator = 0.0,
	 	.estimateError = 0.0,
	 	.ki = 0.1,
	 	.kp = 1.0
	};

	torqueEstimator  = (estimator)
	{
		.deltaT = 0.028,
	 	.estimate = 0.0,
	 	.estimateDot = 0.0,
	 	.estimateDotIntegrator = 0.0,
	 	.estimateError = 0.0,
	 	.ki = 5.0,
	 	.kp = 10.0
	};
	velocityEstimator  = (estimator)
	{
		.deltaT = 0.002,
	 	.estimate = 0.0,
	 	.estimateDot = 0.0,
	 	.estimateDotIntegrator = 0.0,
	 	.estimateError = 0.0,
	 	.ki = 400.0,
	 	.kp = 100.0
	};

	/*
	 * board specific SCI communications initialization
	 */
	BSP_SCI_init(clock, theCpu,pie,gpio);
	BSP_SCI_WIFI_init(clock, theCpu,pie,gpio);
	/*
	 * initialize the ADC handler to convert the binary current value to amps
	 */
	currentSensorHandler = malloc(sizeof(T_ADCHandler));
	currentSensorHandler = ADCHandler_init((void *)currentSensorHandler,sizeof(T_ADCHandler),12,0,2.5,8.33, -1.2);
	/*
	 * board specific PWM Module that allows one to control Motor PWM duty and Shunt Regulator Duty
	 */

	bspPWM = malloc(sizeof(BSP_PWM_OBJ));
	bspPWM = bsp_pwm_init((void*)bspPWM,sizeof(BSP_PWM_OBJ),gpio,clock);

	/*
	 * board specific SPI module that reads the motor current sensor
	 */
	spiHandle = malloc(sizeof(BSP_SPI_Obj));
	spiHandle = BSP_SPI_init((void*)spiHandle,sizeof(BSP_SPI_Obj),clock,theCpu,pie,gpio);



	/*
	 * board specific ADC Module that allows one get raw counts from the motor current adc, supply voltage adc and
	 * the reference voltage adc
	*/
	bspAdc = malloc(sizeof(BSP_ADC_OBJ));
	bspAdc = bsp_adc_init((void*)bspAdc,sizeof(BSP_ADC_OBJ),theCpu,clock,pie,bsp_pwm_getPWMHandle(bspPWM,PWM_BH));

	/*
	 * board specific Encoder module that allows one to get raw ticks from the encoder as well as the difference between the
	 * last two velocity count samples
	 */
	bspEncoder = malloc(sizeof(BSP_S4T_Encoder_Obj));
	bspEncoder = BSP_S4T_Encoder_init((void*)bspEncoder,sizeof(BSP_S4T_Encoder_Obj),clock,theCpu,pie,gpio);


	encoderHandler = malloc(sizeof(T_EncoderHandler));

	/*
	 * Create an encoder handler with a sampling frequency of 50hz and 8000 counts per revolution
	*/
	encoderHandler = EncoderHandler_init((void *)encoderHandler,sizeof(T_EncoderHandler),50,8000);

	speedControlPID = malloc(sizeof(PID_Obj));
	speedControlPID = PID_init((void *)speedControlPID,sizeof(PID_Obj));

	PID_setGains(speedControlPID,3.0,0.01, 0.0);
    PID_setDerFilterParams(speedControlPID,1.0,0.0,0.000333,0.0,0.0);       //cutoff frequency of 3Khz
	PID_setFbackValue(speedControlPID,0.0);
	PID_setMinMax(speedControlPID,0.0,100.0);
	PID_setUi(speedControlPID,0.0);
    PID_setUIMinMax(speedControlPID,-5.0,5.0);

	DELAY_US(4e6);
	/*
	 * initialize the GPIO pins
	 */
	bsp_gpio_init();
	gpioCapture = CAP_init((void *)CAP1_BASE_ADDR, sizeof(CAP_Obj));
	setUpGPIOCapture();
	DELAY_US(5e6);

	Wifi_Check_Update();
	DELAY_US(1e6);
	Wifi_Send_Command(Wifi_handshake_A,2);

	pic[3]= 0x00; //  Logo screen
	GUI_send_command(pic,4);

	Wifi_scan_A[2] = 0x00;   // Scan Network
	Wifi_Send_Command(Wifi_scan_A,3);

	Wifi_Send_Command(Wifi_userProfilesScan_A,2);

	Wifi_userProfileNumber_A[1] = WIFI_USER_NUMBER_CHECK;
	Wifi_userProfileNumber_A[2] = 0;
	Wifi_Send_Command(Wifi_userProfileNumber_A,3);

	DELAY_US(1e6);
	pic[3]= 0x01;  // First screen
	GUI_send_command(pic,4);


	/*
	 * Initialize I2C and stimulation interrupt
	 */
	initializeI2C();

	enableStimulation(false);

	enableLeftBoost(true);
	DELAY_US(1e6);
	enableRightBoost(true);

	isGeometryValid();
	changedStimulationFrequency(60.0);


	while(true)
	{
		position = EncoderHandler_getPosition(encoderHandler,BSP_S4T_Encoder_getRawCounts());
		if(enableStim)
		{
			enableStimulation(true);
			enableStim = false;
		}
		if(disableStim)
		{
			enableStimulation(false);
			disableStim = false;
		}
		if(stimulationFlag)
		{
			doStimulation();
			stimulationFlag = false;
		}
		if(calculatePassiveTorqueFlag)
		{
			calculatePassiveTorques();
		}

		if(controlFlag)
		{
			doControl();
			controlFlag = false;
		}
		if(rtcFlag)
		{
			serviceRTC();
			rtcFlag = false;
		}
		if(DeviceInfoDisplay)
		{
			DeviceInfo();
			DeviceInfoDisplay = false;

			//if we are programming the wifi, disable the wifi SCI when we are in the settings screen
			if(!ENABLE_WIFI_UART)
			{
				BSP_SCI_WIFI_disable(clock, theCpu,pie,gpio);
			}
		}

	}
}

void calculatePassiveTorques(void)
{
	int i = 0;
	for(i = 0; i < 360; i++)
	{
		float_t angle = i*degreesToRadians;
		float_t passiveTorque = Calculate_Passive_Torque(angle,velocityEstimator.estimate,velocityEstimator.estimateDotIntegrator);
		passiveTorques[i] = passiveTorque;
	}
	calculatePassiveTorqueFlag = false;
}

void setUpGPIOCapture(void)
{

	//set up gpio 5 for reading in the gpio
	GPIO_setMode(gpio, GPIO_Number_5, GPIO_5_Mode_ECAP1);
	GPIO_setDirection(gpio, GPIO_Number_5, GPIO_Direction_Input);

	CPU_disableGlobalInts(theCpu);
	CAP_disableTimestampCounter(gpioCapture);
	CAP_disableInt(gpioCapture,CAP_Int_Type_All);

	CLK_enableEcap1Clock(clock);
	CAP_setModeCap(gpioCapture);

	//capture on falling edges
	CAP_setCapEvtPolarity(gpioCapture, CAP_Event_1,CAP_Polarity_Rising);
	CAP_setCapEvtPolarity(gpioCapture, CAP_Event_2,CAP_Polarity_Falling);
	CAP_setCapEvtPolarity(gpioCapture, CAP_Event_3,CAP_Polarity_Rising);
	CAP_setCapEvtPolarity(gpioCapture, CAP_Event_4,CAP_Polarity_Falling);

	//reset on every capture event (delta-t mode)
	CAP_setCapEvtReset(gpioCapture, CAP_Event_1,CAP_Reset_Enable);
	CAP_setCapEvtReset(gpioCapture, CAP_Event_2,CAP_Reset_Enable);
	CAP_setCapEvtReset(gpioCapture, CAP_Event_3,CAP_Reset_Enable);
	CAP_setCapEvtReset(gpioCapture, CAP_Event_4,CAP_Reset_Enable);
	CAP_disableSyncIn(gpioCapture);
	CAP_enableCaptureLoad(gpioCapture);
	CAP_setCapContinuous(gpioCapture);

	EALLOW;
	ECap1Regs.ECCTL1.bit.FREE_SOFT = 0b11;
	EDIS;
	CAP_enableInt(gpioCapture,CAP_Int_Type_CEVT1);
	CAP_clearInt(gpioCapture, CAP_Int_Type_All);
	CPU_enableInt(theCpu, CPU_IntNumber_4);
	PIE_registerPieIntHandler(pie,PIE_GroupNumber_4,PIE_SubGroupNumber_1,(PIE_IntVec_t) &gpioCaptureInterrupt);	//register the interrupt
	PIE_enableInt(pie,PIE_GroupNumber_4,PIE_InterruptSource_ECAP1);		//enable the interrupt in the PIE
	CAP_enableTimestampCounter(gpioCapture);
	CPU_enableGlobalInts(theCpu);
}
void doVelocityRamping()
{

	if (newVelocity != desiredSpeed)
	{
		desiredSpeed += velocityIncrement;

		if (velocityIncrement < 0 && desiredSpeed < newVelocity)
		{
			desiredSpeed = newVelocity;
			speedRamping = false;
		}
		if (velocityIncrement > 0 && desiredSpeed > newVelocity)
		{
			speedRamping = false;
			desiredSpeed = newVelocity;
		}
	}
}

void doLevelRamping()
{
	if(newLevel != level)
	{
		level += levelIncrement;

		if(levelIncrement < 0 && level < newLevel)
		{
			level = newLevel;
			rampingLevel = false;
		}
		if(levelIncrement > 0 && level > newLevel)
		{
			level = newLevel;
			rampingLevel = false;
		}
	}
}

void doActivePowerEstimation(void)
{

	powerAverageCount++;
	activeTorque = Calculate_Active_Torque(position,currentEstimator.estimate,velocityEstimator.estimate,velocityEstimator.estimateDotIntegrator);
	runEstimator(&(torqueEstimator),activeTorque);
	power = torqueEstimator.estimate*velocityEstimator.estimate;
	power = MATH_sat(power,999.0,0.0000001);
	if(power > 1.0)
	{
		activePowerSum += power;
	}


	if(!speedRamping && warmup_timer_flag)
	{
		/*
		 * if we are in the warmup, do torque estimates
		 */
		Update_Passive_Torque_Params(getNetCrankTorque(currentEstimator.estimate, velocityEstimator.estimate, velocityEstimator.estimateDotIntegrator), position);
	}
	checkLCDConnectionFlag = true;

}

void doSpasmDetection(void)
{
	if(speedRamping)
	{
		spasmCurrent = rampingSpasmCurrent;
	}
	else
	{
		spasmCurrent = workoutSpasmCurrent;
	}
	if(currentEstimator.estimate > spasmCurrent)
	{
		spasmCount++;
		if(spasmCount > maxSpasmCount)
		{
			numberOfSpasms++;
			spasmFlag = true;   // change for debugging
			controlEnabled = false;
			enableStimulation(false);
			setDesiredSpeed(0.0);
		}
	}
	else
	{	if(spasmCount > 0)
		{
			spasmCount--;
		}

	}
}
void doControl(void)
{
	recoverEstimator(&currentEstimator);


	//increment the control loop dividers
	torqueControlLoopCount++;
	velocityControlLoopCount++;
	if(controlEnabled)
	{

		if (torqueControlLoopCount > TORQUE_LOOP_DIVIDER)
		{

			doActivePowerEstimation();

			if(warmup_timer_flag || workout_timer_flag)
			{

				streamPower = power * 100;
				unsigned char positionByte = (unsigned char)(position / 0.024544);
				Wifi_workoutStream_A[2] = positionByte;
				Wifi_workoutStream_A[3] = (unsigned char)((streamPower >> 8) & 0x00FF);
				Wifi_workoutStream_A[4] = (unsigned char)(streamPower & 0x00FF);
				Wifi_workoutStream_A[5] = (unsigned char) level;
				Wifi_Send_Command(Wifi_workoutStream_A,6);
			}
			torqueControlLoopCount = 0;
		}

		if(velocityControlLoopCount > SPEED_LOOP_DIVIDER)
		{

		   doSymmetryCalculation();
		   doVelocityRamping();
		   toggleDebugLED();



		   velocity = EncoderHandler_getVelocity(encoderHandler,BSP_S4T_Encoder_getRawCountsDot());
		   runEstimator(&(velocityEstimator),velocity);
		   velocityControlLoopCount = 0;
		   speedFFW = speedFFWGain*desiredSpeed + 49.89;
		   PID_run_parallel(speedControlPID, desiredSpeed, velocityEstimator.estimate, speedFFW, &speedControlOut);
		   dutyCycle = speedControlOut;
		   dutyCycle = MATH_sat(dutyCycle,MAX_DUTY,MIN_DUTY);
		   bsp_pwm_setDuty(bspPWM,MOTOR_PWM,dutyCycle);

			if(SPASM_ENABLE)
			{
				doSpasmDetection();
			}
		}
		if(firstEnabledControlLoop)
		{
			enableLockedAntiphase();
			firstEnabledControlLoop = false;
		}

		if(ENCODER_WATCDOG_ENABLE)
		{
			QEP_enable_watchdog(getQepHandle(bspEncoder));
			QEP_enable_interrupt(getQepHandle(bspEncoder),QEINT_Wto);
		}
	}
	else
	{

		QEP_disable_interrupt(getQepHandle(bspEncoder),QEINT_Wto);
		QEP_disable_watchdog(getQepHandle(bspEncoder));
		EQep1Regs.QWDTMR = 0;
		firstEnabledControlLoop = true;
		//put the controller into drive coast mode with a pwm of zero
		GPIO_setLow(gpio,GPIO_Number_6); //pwmh
		GPIO_setLow(gpio,GPIO_Number_7); //pwml
		dutyCycle = 0.0;
		speedControlPID->refValue = 0.0;
		speedControlPID->fbackValue = 0.0;
		speedControlPID->Ui = 0.0;
		bsp_pwm_setDuty(bspPWM,MOTOR_PWM,0.0);

	}

}
void doSymmetryCalculation(void)
{
	if(isInStimRegion(rightQuadOnAngle, rightQuadOffAngle,position))
	{
		if(torqueEstimator.estimate > 1.0)
		{
			rightPowerSum += power;
		}
	}
	else if(isInStimRegion(leftQuadOnAngle, leftQuadOffAngle,position))
	{
		if(torqueEstimator.estimate > 1.0)
		{
			leftPowerSum += power;
		}

	}
}
void enableLockedAntiphase(void)
{
	//disable the motor before making changes
	enableMotor(false);

	//gpio7 is pwmL, set it high
	GPIO_setDirection(gpio,GPIO_Number_7,GPIO_Direction_Output);
	GPIO_setHigh(gpio,GPIO_Number_7);

	//gpio6 is pwmH, set it high
	GPIO_setDirection(gpio,GPIO_Number_6,GPIO_Direction_Output);
	GPIO_setHigh(gpio,GPIO_Number_6);
	dutyCycle = 51;
	//set the duty to 50%
	bsp_pwm_setDuty(bspPWM,MOTOR_PWM,51.0);
	enableMotor(true);

}

void initializeStimulationInterrupt(void)
{
	stimulationTimer = TIMER_init((void *)TIMER1_BASE_ADDR, sizeof(TIMER_Obj));
	CLK_enableCpuTimerClock(clock, CLK_CpuTimerNumber_1);
	CPU_enableInt(theCpu, CPU_IntNumber_13);
	TIMER_enableInt(stimulationTimer);
	TIMER_setEmulationMode(stimulationTimer, TIMER_EmulationMode_RunFree);

	TIMER_setPeriod(stimulationTimer,1333333);	//initialize to 60hz
	PIE_registerSystemIntHandler(pie, PIE_SystemInterrupts_TINT1, (PIE_IntVec_t) &stimulationInterrupt);
	enableStimulation(false);
}
void intializeControlLoopInterrupt(void)
{
	controlLoopTimer = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
	CPU_enableInt(theCpu, CPU_IntNumber_1);
    CLK_enableCpuTimerClock(clock, CLK_CpuTimerNumber_0);
	PIE_enableTimer0Int(pie);
	PIE_registerPieIntHandler(pie,PIE_GroupNumber_1, PIE_SubGroupNumber_7, (PIE_IntVec_t) &controlLoopInterrupt);
	PIE_enableInt(pie, PIE_GroupNumber_1, PIE_InterruptSource_TIMER_0);
	TIMER_setPeriod(controlLoopTimer, 4000); //set up to interrupt at 20kHz
	TIMER_enableInt(controlLoopTimer);		//enable timer 0 interrupt
	TIMER_setEmulationMode(controlLoopTimer, TIMER_EmulationMode_RunFree);
	TIMER_setPreScaler(controlLoopTimer,0);
	TIMER_start(controlLoopTimer);

}
void initializeRTCInterrupt(void)
{
	RTCTimer = TIMER_init((void *)TIMER2_BASE_ADDR, sizeof(TIMER_Obj));
	CLK_enableCpuTimerClock(clock, CLK_CpuTimerNumber_2);
	CPU_enableInt(theCpu, CPU_IntNumber_14);
	TIMER_enableInt(RTCTimer);
	TIMER_setEmulationMode(RTCTimer, TIMER_EmulationMode_RunFree);

	TIMER_setPeriod(RTCTimer,8e7);	//initialize to 1hz
	PIE_registerSystemIntHandler(pie, PIE_SystemInterrupts_TINT2, (PIE_IntVec_t) &RTCInterrupt);
	enableRTC(false);
}

void changedStimulationFrequency (float newFrequency)
{
	/*
	 * only allow values within an acceptable range (40-100hz)
	 */
	newFrequency = MATH_sat(newFrequency,100.0,30.25);
	uint32_t count = 8e7/newFrequency;
	TIMER_stop(stimulationTimer);
	TIMER_setPeriod(stimulationTimer,count);
	TIMER_reload(stimulationTimer);
	TIMER_start(stimulationTimer);
}
void doStimulation(void)
{

	if(!speedRamping)
	{
		doLevelRamping();
	}

	positionInDegrees = position*radiansToDegrees;
	rightTorqueTransferIndex = (int)(2*positionInDegrees);
	leftTorqueTransferIndex = (rightTorqueTransferIndex + 360)%720;
	float pulsewidth = 0.0;

	/*
	* send the pulses on the left and right quads if they are enabled
	*/
	if(rightQuadEnable)
	{
		if(isInStimRegion(rightQuadOnAngle,rightQuadOffAngle,position) && !testingChannels)
		{
			if(warmup_timer_flag || speedRamping || warmupStartButtonPressed)
			{
				//If we are in the warmup/speedRamping, only send test pulses
				sendPulse(RIGHT_QUAD_CHANNEL_NUMBER,warmupCurrent,warmupPulseWidth,RIGHT_I2C_ADDRESS);
			}
			else
			{
				pulsewidth = -kneeTorqueTransfer[rightTorqueTransferIndex] * Kus* level * kRightQuad;
				pulsewidth = MATH_sat(pulsewidth,MAX_PULSEWIDTH,MIN_PULSEWIDTH);
				rightQuadPulseWidth = (int)pulsewidth;
				sendPulse(RIGHT_QUAD_CHANNEL_NUMBER,quadCurrent,rightQuadPulseWidth,RIGHT_I2C_ADDRESS);
			}
		}
		else if(testingChannels && forceQuads)
		{
			sendPulse(RIGHT_QUAD_CHANNEL_NUMBER,testCurrent,testPulseWidth,RIGHT_I2C_ADDRESS);
		}
		DELAY_US(4);
	}

	if(leftQuadEnable)
	{
		if(isInStimRegion(leftQuadOnAngle,leftQuadOffAngle,position)&& !testingChannels)
		{

			if(warmup_timer_flag || speedRamping || warmupStartButtonPressed)
			{
				//If we are in the warmup, only send test pulses
				sendPulse(LEFT_QUAD_CHANNEL_NUMBER,warmupCurrent,warmupPulseWidth,LEFT_I2C_ADDRESS);
			}
			else
			{
				pulsewidth = -kneeTorqueTransfer[leftTorqueTransferIndex] * Kus* level*kLeftQuad;
				pulsewidth = MATH_sat(pulsewidth,MAX_PULSEWIDTH,MIN_PULSEWIDTH);
				leftQuadPulseWidth = (int)pulsewidth;
				sendPulse(LEFT_QUAD_CHANNEL_NUMBER,quadCurrent,leftQuadPulseWidth,LEFT_I2C_ADDRESS);
			}
		}
		else if(testingChannels && forceQuads)
		{
			sendPulse(LEFT_QUAD_CHANNEL_NUMBER,testCurrent,testPulseWidth,LEFT_I2C_ADDRESS);
		}
	}

	DELAY_US(2000);

	/*
	* update the channel status on the left and right quad if they are enabled
	*/
	if(rightQuadEnable)
	{
		/*
		 * if the right quad is enabled, update its status
		 */
		rightQuadStatus = requestStatus(RIGHT_I2C_ADDRESS) & 0x1;
	}
	if(leftQuadEnable)
	{
		/*
		 * if the left quad is enabled, update its status
		 */
		leftQuadStatus = requestStatus(LEFT_I2C_ADDRESS) & 0x1;
	}
	/*
	* send the pulses on the left and right hams if they are enabled
	 */
	if(rightHamEnable)
	{
		if(isInStimRegion(rightHamOnAngle,rightHamOffAngle,position)&& !testingChannels)
		{


			if(warmup_timer_flag || speedRamping || warmupStartButtonPressed)
			{
				sendPulse(RIGHT_HAM_CHANNEL_NUMBER,warmupCurrent,warmupPulseWidth,RIGHT_I2C_ADDRESS);
			}
			else
			{
				pulsewidth = kneeTorqueTransfer[rightTorqueTransferIndex]*Kus*level*kRightHam;
				pulsewidth = MATH_sat(pulsewidth,MAX_PULSEWIDTH,MIN_PULSEWIDTH);
				rightHamPulseWidth = (int)pulsewidth;
				sendPulse(RIGHT_HAM_CHANNEL_NUMBER,hamCurrent,rightHamPulseWidth,RIGHT_I2C_ADDRESS);
			}
		}
		else if(testingChannels && forceHams)
		{
			sendPulse(RIGHT_HAM_CHANNEL_NUMBER,testCurrent,testPulseWidth,RIGHT_I2C_ADDRESS);
		}
		DELAY_US(4);
	}
	if(leftHamEnable)
	{
		if(isInStimRegion(leftHamOnAngle,leftHamOffAngle,position) && !testingChannels)
		{
			if(warmup_timer_flag || speedRamping || warmupStartButtonPressed)
			{
				sendPulse(LEFT_HAM_CHANNEL_NUMBER,warmupCurrent,warmupPulseWidth,LEFT_I2C_ADDRESS);
			}
			else
			{
				pulsewidth = kneeTorqueTransfer[leftTorqueTransferIndex] * Kus* level*kLeftHam;
				pulsewidth = MATH_sat(pulsewidth,MAX_PULSEWIDTH,MIN_PULSEWIDTH);
				leftHamPulseWidth = (int)pulsewidth;
				sendPulse(LEFT_HAM_CHANNEL_NUMBER,hamCurrent,leftHamPulseWidth,LEFT_I2C_ADDRESS);
			}
		}
		else if(testingChannels && forceHams)
		{
			sendPulse(LEFT_HAM_CHANNEL_NUMBER,testCurrent,testPulseWidth,LEFT_I2C_ADDRESS);
		}
		DELAY_US(4);
	}
	DELAY_US(2000);

	/*
	 * update the channel status on the left and right hams if they are enabled
	 */
	if(rightHamEnable)
	{
		/*
		 * if the right ham is enabled, update its status
		 */
		rightHamStatus = requestStatus(RIGHT_I2C_ADDRESS) & 0x2;

	}
	if(leftHamEnable)
	{
		/*
		* if the right ham is enabled, update its status
		*/
		leftHamStatus = requestStatus(LEFT_I2C_ADDRESS) & 0x2;

	}

	/*
	* send the pulses on the left and right glutes if they are enabled
	*/
	if(rightGluteEnable)
	{
		if(isInStimRegion(rightGluteOnAngle,rightGluteOffAngle,position) && !testingChannels)
		{
			if(warmup_timer_flag || speedRamping || warmupStartButtonPressed)
			{
				sendPulse(RIGHT_GLUTE_CHANNEL_NUMBER,warmupCurrent,warmupPulseWidth,RIGHT_I2C_ADDRESS);
			}
			else
			{
				pulsewidth = hipTorqueTransfer[rightTorqueTransferIndex] * Kus* level* kRightGlute;
				pulsewidth = MATH_sat(pulsewidth,MAX_PULSEWIDTH,MIN_PULSEWIDTH);
				rightGlutePulseWidth = (int)pulsewidth;
				sendPulse(RIGHT_GLUTE_CHANNEL_NUMBER,gluteCurrent,rightGlutePulseWidth,RIGHT_I2C_ADDRESS);
			}
		}
		else if (testingChannels && forceGlutes)
		{

			sendPulse(RIGHT_GLUTE_CHANNEL_NUMBER,testCurrent,testPulseWidth,RIGHT_I2C_ADDRESS);

		}
		DELAY_US(4);
	}
	if(leftGluteEnable)
	{
		if(isInStimRegion(leftGluteOnAngle,leftGluteOffAngle,position) && !testingChannels)
		{


			if(warmup_timer_flag || speedRamping || warmupStartButtonPressed)
			{
				sendPulse(LEFT_GLUTE_CHANNEL_NUMBER,warmupCurrent,warmupPulseWidth,LEFT_I2C_ADDRESS);
			}
			else
			{
				pulsewidth = hipTorqueTransfer[leftTorqueTransferIndex] * Kus*level*kLeftGlute;
				pulsewidth = MATH_sat(pulsewidth,MAX_PULSEWIDTH,MIN_PULSEWIDTH);
				leftGlutePulseWidth = (int)pulsewidth;
				sendPulse(LEFT_GLUTE_CHANNEL_NUMBER,gluteCurrent,leftGlutePulseWidth,LEFT_I2C_ADDRESS);
			}

		}
		else if (testingChannels && forceGlutes)
		{

			sendPulse(LEFT_GLUTE_CHANNEL_NUMBER,testCurrent,testPulseWidth,LEFT_I2C_ADDRESS);

		}
		DELAY_US(4);
	}
	DELAY_US(2000);

	/*
	* update the channel status on the left and right glute if they are enabled
	*/
	if(rightGluteEnable)
	{
		/*
		 * if the right glute is enabled, update it's status
		 */
		rightGluteStatus = requestStatus(RIGHT_I2C_ADDRESS) & 0x4;

	}
	if(leftGluteEnable)
	{
		/*
		 * if the left glute is enabled, update it's status
		*/
		leftGluteStatus = requestStatus(LEFT_I2C_ADDRESS) & 0x4;

	}

	currentrightworkoutchannelstatus = rightGluteStatus | rightHamStatus | rightQuadStatus;   // REMOVE COMMENTS FOR NORMAL OPERATION
	currentleftworkoutchannelstatus = leftGluteStatus | leftHamStatus | leftQuadStatus;	    // REMOVE COMMENTS FOR NORMAL OPERATION

	 if ((currentrightworkoutchannelstatus != rightworkoutchannelstatus || currentleftworkoutchannelstatus != leftworkoutchannelstatus)  && !speedRamping )
	 {
		 padsFlagCount++;
		 if(padsFlagCount > maxPadsFlagCount)
		 {
			 padsFlag = true;

		 }
	 }
	 else
	 {
		 padsFlagCount = 0;
	 }

}
void serviceRTC(void)
{

    if (warmupStartButtonPressed)		 // Timer flag for the warmup popup
    {
    	sendPulse(RESET_CHANNEL_STATUS_NUMBER,0x0,0x0,LEFT_I2C_ADDRESS);
    	DELAY_US(200);
    	sendPulse(RESET_CHANNEL_STATUS_NUMBER,0x0,0x0,RIGHT_I2C_ADDRESS);
    	Initialize_Torque_Approximation();
    	startWarmup();
    }
    if (warmup_timer_flag) 					// Timer flag for the warmup timer
    {
		if(spasmFlag)
		{

			enableRTC(false);
			do_warmup_timer();
			spasmFlag = false;
			Current_Screen = SCREEN_31;
			pic[3]= 31;  //  screen
			GUI_send_command(pic,4);
			enableStimulation(false);
			controlEnabled = false;
			setDesiredSpeed(0.0);
			EQep1Regs.QWDTMR = 0;

		}
		else
		{
			do_warmup_timer();
		}
    }
    if (workout_countdown_popup_flag)	 	// Timer flag for the workout popup
    {
    	do_workout_countdown_popup();
    }
    if (workout_timer_flag) 				// Timer flag for the workout timer
    {
		if(spasmFlag)
		{

			enableRTC(false);
			do_workout_timer();
			spasmFlag = false;
			oldLevel = desiredLevel;
			Current_Screen = SCREEN_29;
			pic[3]= 29;
			GUI_send_command(pic,4);
			enableStimulation(false);
			controlEnabled = false;
			setDesiredSpeed(0.0);
			EQep1Regs.QWDTMR = 0;
			GUI_reset_muscle_group_selected_flags();
			Update_Stimulation_levels_Wifi();
		}
		else if (padsFlag)
		{
			enableStimulation(false);
			controlEnabled = false;
			setDesiredSpeed(0.0);
			oldLevel = desiredLevel;
			enableRTC(false);

			if ((currentleftworkoutchannelstatus & 0x1) != (leftworkoutchannelstatus & 0x1)){
					//Left Quads disconnected
				pic[3]= 17;
				currentleftworkoutchannelstatus |= (leftworkoutchannelstatus & 0x1);

			}
			else if (((currentleftworkoutchannelstatus & 0x2) >> 1) != ((leftworkoutchannelstatus & 0x2) >> 1)){
				//Left Hams disconnected
				pic[3]= 23;
				currentleftworkoutchannelstatus |= (leftworkoutchannelstatus & 0x2);

			}
			else if (((currentleftworkoutchannelstatus & 0x4) >> 2) != ((leftworkoutchannelstatus & 0x4) >> 2)){
				//Left Glutes disconnected
				pic[3]= 26;
				currentleftworkoutchannelstatus |= (leftworkoutchannelstatus & 0x4);

			}
			else if ((currentrightworkoutchannelstatus & 0x1) != (rightworkoutchannelstatus & 0x1)){
				//Right Quads disconnected
				pic[3]= 37;
				currentrightworkoutchannelstatus |= (rightworkoutchannelstatus & 0x1);

			}
			else if (((currentrightworkoutchannelstatus & 0x2) >> 1) != ((rightworkoutchannelstatus & 0x2) >> 1)){
				//Right Hams disconnected
				pic[3]= 38;
				currentrightworkoutchannelstatus |= (rightworkoutchannelstatus & 0x2);

			}
			else if (((currentrightworkoutchannelstatus & 0x4) >> 2) != ((rightworkoutchannelstatus & 0x4) >> 2)){
				//Right Glutes disconnected
				pic[3]= 66;
				currentrightworkoutchannelstatus |= (rightworkoutchannelstatus & 0x4);

			}
			else{

				currentleftworkoutchannelstatus = leftworkoutchannelstatus;
				currentrightworkoutchannelstatus = rightworkoutchannelstatus;
			}
			padsFlag = false;
			GUI_send_command(pic,4);
			GUI_reset_muscle_group_selected_flags();
			Update_Stimulation_levels_Wifi();

		}
		else
		{
	    	do_workout_timer();
	    	GUI_display_workout_parameters();
	    	calculate_workout_averages();

	    	if(workout_timer_done)
	    	{
	    	  pic[3] = 8;
	    	  GUI_send_command(pic,4);
	    	  display_workout_summary();
	    	  newLevel = 0;
	    	  level = 0;
	    	  controlEnabled = false;
	    	  setDesiredSpeed(0.0);
	    	  workout_timer_done = false;
	    	}

		}
    }


}
__interrupt void stimulationInterrupt(void)
{
	stimulationFlag = true;
	EALLOW;
	CpuTimer1Regs.TCR.bit.TIF = 1;
	EDIS;
}
__interrupt void controlLoopInterrupt(void)
{

	controlFlag = true;
	rawMotorCurrentBytes = BSP_SPI_getRawCurrentSensorData(spiHandle);
	motorCurrentVolts = ADC_Handler_Counts_To_Volts(currentSensorHandler,rawMotorCurrentBytes);
	instantaneousCurrent = ADC_Handler_Counts_To_Output_Offset_In_Volts(currentSensorHandler,rawMotorCurrentBytes);
	runEstimator(&(currentEstimator),instantaneousCurrent);
	EALLOW;
	CpuTimer0Regs.TCR.bit.TIF = 1;
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
	EDIS;
}
__interrupt void RTCInterrupt(void)   		// 1Hz interrupt for the timers
{

	rtcFlag = true;
	EALLOW;
	CpuTimer2Regs.TCR.bit.TIF = 1;
	EDIS;
}

__interrupt void gpioCaptureInterrupt(void)
{
	cap1 = ((80e6/CAP_getCap1(gpioCapture)));
	cap2 = ((80e6/CAP_getCap2(gpioCapture)));
	cap3 = ((80e6/CAP_getCap3(gpioCapture)));
	cap4 = ((80e6/CAP_getCap4(gpioCapture)));
	averageGPIOFrequency = (cap1 + cap2 + cap3 + cap4)/4;
	EALLOW;
	CAP_clearInt(gpioCapture,CAP_Int_Type_CEVT1);
	CAP_clearInt(gpioCapture,CAP_Int_Type_Global);
	PieCtrlRegs.PIEACK.bit.ACK4 = 1;
	EDIS;
}
