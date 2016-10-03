/*
 * RGBColor.h
 *
 *  Created on: Oct 1, 2016
 *      Author: John
 */

#ifndef NEOPIXELSTRING_RGBCOLOR_H_
#define NEOPIXELSTRING_RGBCOLOR_H_
#include <stdlib.h>
#include "../HardwareAbstractionLayer/hal.h"
typedef struct
{
	float red;
	float green;
	float blue;
}RGBColor;

inline RGBColor RGBColor_getGradientForValue(float value)
{
	float red;
	float green;
	if(value < 50)
	{
		 red = 2*value/100;
		 green = 1;
	}
	else
	{
		red = 1.0;
		green = 1- 2*(value-50)/100;
	}

	RGBColor color = {red,green,0};
	return color;
}
static RGBColor RED = {1,0,0};
static RGBColor LIME = {0,1,0};
static RGBColor BLUE = {0,0,1};
static RGBColor YELLOW = {1,1,0};
static RGBColor CYAN = {0,1,1};
static RGBColor MAGENTA = {1,0,1};
static RGBColor WHITE = {1,1,1};
static RGBColor ORANGE = {1,0.55,0};
static RGBColor BLACK = {0,0,0};
#endif /* NEOPIXELSTRING_RGBCOLOR_H_ */
