#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include <math.h>
#include<stdlib.h>


#include "BSP/pwm/bsp_pwm.h"
#include "BSP/communications/bsp_sci.h"
#include "BSP/communications/bsp_spi.h"
#include "BSP/Encoder/bsp_s4t_encoder.h"
#include "BSP/Processor/processor.h"
#include "BSP/gui/gui.h"
#include "BSP/gui/gui_defines.h"

BSP_PWM_Handle bspPWM;
BSP_S4T_Encoder_Handle bspEncoder;
BSP_SPI_Handle bspSpi;

volatile float dutyCycle = 0.0;
volatile uint16_t rawCurrent = 0.0;


volatile uint32_t encoderCounts = 0;
volatile int32_t deltaCounts = 0;



int main(void)
{

	initializeProcessor();
	CLK_enableTbClockSync(clock);
	GPIO_setDirection(gpio,GPIO_Number_10,GPIO_Direction_Output);

	/*
	 * board specific SCI communications initialization
	 */
	BSP_SCI_init(clock, theCpu,pie,gpio);


	/*
	 * board specific SPI communications initialization
	 */
	bspSpi = malloc(sizeof(BSP_SPI_Obj));
	bspSpi = BSP_SPI_init((void*)bspSpi,sizeof(BSP_SPI_Obj),clock,theCpu,pie,gpio);
	/*
	 * board specific PWM Module that allows one to control Motor PWM duty
	 */
	bspPWM = malloc(sizeof(BSP_PWM_OBJ));
	bspPWM = bsp_pwm_init((void*)bspPWM,sizeof(BSP_PWM_OBJ),gpio,clock);
	/*
	 * board specific Encoder module that allows one to get raw ticks from the encoder as well as the difference between the
	 * last two velocity count samples
	 */
	bspEncoder = malloc(sizeof(BSP_S4T_Encoder_Obj));
	bspEncoder = BSP_S4T_Encoder_init((void*)bspEncoder,sizeof(BSP_S4T_Encoder_Obj),clock,theCpu,pie,gpio);


	GUI_send_command(hand_shake,2);

	DELAY_US(500000);

	GUI_send_command(pic,4);
	GUI_send_end_bytes();
	DELAY_US(5000000);

	pic[3]= 0x01;
	GUI_send_command(pic,4);
	GUI_send_end_bytes();
	DELAY_US(1000000);

	while(true)
	{

//		bsp_pwm_setDuty(bspPWM,MOTOR_PWM,dutyCycle);
//
//		rawCurrent = BSP_SPI_getRawCurrentSensorData(bspSpi);
//		DELAY_US(10000);
//
//		GPIO_toggle(gpio,GPIO_Number_10);
//
//		//Specific to GUI
//		if(in_workout == 1){
//			GUI_display_workout_parameters();
//		}

//		GUI_check_for_commands();
//		GUI_check_pads_unconnected_fault();

	}
}
