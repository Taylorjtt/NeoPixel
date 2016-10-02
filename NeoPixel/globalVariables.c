/*
 * globalVars.c
 *
 *  Created on: Jan 12, 2016
 *      Author: JohnTaylor
 */


#include "globalVariables.h"
#include "BSP/gui/gui_defines.h"

TIMER_Handle stimulationTimer;
TIMER_Handle controlLoopTimer;
TIMER_Handle RTCTimer;

BSP_PWM_Handle bspPWM;
uint16_t maxSpasmCount = 250;
ENCODER_FAULT encoderFault = ENCODER_NO_FAULT;
CURRENT_SENSOR_FAULT currentSensorFault = CURRENT_SENSOR_NO_FAULT;
LCD_FAULT lcdFault = LCD_NO_FAULT;
uint16_t lcdResponse = 0;
bool stimulationEnabled = false;
float  LOW_SPASM = 1.0;
float MED_SPASM  = 1.25;
float HIGH_SPASM = 1.5;
float defaultSpeed = 3.665; //rad/s
float level = 0.0;
float maxAcceleration = 1.0;//	rad/s^2
float maxStimulationAcceleration = 0.6;//rad/s^2
float velocity = 0.0;

float desiredSpeed = 0.0;
bool controlEnabled = false;
float kQuad = Low_Muscle_Gain;
float kHam = Low_Muscle_Gain;
float kGlute = Low_Muscle_Gain;
float kLeftQuad = Low_Muscle_Gain;
float kLeftHam = Low_Muscle_Gain;
float kLeftGlute = Low_Muscle_Gain;
float kRightQuad = Low_Muscle_Gain;
float kRightHam = Low_Muscle_Gain;
float kRightGlute = Low_Muscle_Gain;
bool speedRamping = false;
bool calculatePassiveTorqueFlag = false;
float newVelocity = 0.0;
float velocityDifference = 0.0;
float velocityIncrement = 0.0;
float rightPowerSum = 0.0;
float leftPowerSum = 0.0;
float leftSymmetryPercent = 0.0;
float rightSymmetryPercent = 0.0;
float totalTorqueSum;
unsigned char rightChannelStatus = 0;
unsigned char rightworkoutchannelstatus = 0;
unsigned char currentrightworkoutchannelstatus = 0;
unsigned char old_right_channel_status = 0;
unsigned char leftChannelStatus = 0;
unsigned char leftworkoutchannelstatus = 0;
unsigned char currentleftworkoutchannelstatus = 0;
unsigned char old_left_channel_status = 0;
unsigned char rightQuadStatus = 0;
unsigned char rightHamStatus = 0;
unsigned char rightGluteStatus = 0;
unsigned char leftQuadStatus = 0;
unsigned char leftHamStatus = 0;
unsigned char leftGluteStatus = 0;
bool padsFlag = false;
float desiredLevel = 0;
float newLevel = 1.0;
float levelDifference = 0.0;
float levelIncrement = 0.0;
float oldLevel = 0.0;
bool rampingLevel = false;

SCREEN_FSM Current_Screen = SCREEN_1;

void setDesiredLevel(float requestedLevel)
{

		rampingLevel = true;
		newLevel = requestedLevel;
		levelDifference = newLevel - level;

		if(levelDifference > 0)
		{
			levelIncrement = maxStimulationAcceleration*0.016;
		}
		else
		{
			level = newLevel;
		}
}
void setDesiredSpeed(float newSpeed)
{
	speedRamping = true;
	newVelocity = newSpeed;
	velocityDifference = newVelocity - desiredSpeed;

	if(velocityDifference > 0)
	{
		velocityIncrement = maxAcceleration*0.002;
	}
	else
	{
		velocityIncrement = -maxAcceleration*0.002;
	}
	if(newVelocity == 0.0)
	{
		desiredSpeed = 0.0;
	}

}
float signum(float input)
{
	if(input > 0)
	{
		return 1.0;
	}
	else if (input < 0)
	{
		return -1.0;
	}
	return 0.0;
}
void enableStimulation(bool isEnabled)
{

	if(isEnabled)
	{
		stimulationEnabled = true;
		TIMER_enableInt(stimulationTimer);
		TIMER_reload(stimulationTimer);
		TIMER_start(stimulationTimer);
	}
	if(!isEnabled)
	{
		stimulationEnabled = false;
		TIMER_disableInt(stimulationTimer);
		TIMER_stop(stimulationTimer);
		TIMER_reload(stimulationTimer);
	}
}


void enableRTC(bool isEnabled)
{
	if(isEnabled)
	{
		TIMER_enableInt(RTCTimer);
		TIMER_reload(RTCTimer);
		TIMER_start(RTCTimer);
	}
	if(!isEnabled)
	{
		TIMER_disableInt(RTCTimer);
		TIMER_stop(RTCTimer);
		TIMER_reload(RTCTimer);
	}
}


