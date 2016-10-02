/*
 * Class.h
 *
 *  Created on: Aug 12, 2016
 *      Author: JohnTaylor
 */

#ifndef WIFI_CLASS_H
#define WIFI_CLASS_H
#define MAX_NUMBER_OF_LEDS 60
#include <stdlib.h>
#include "RGBColor.h"
#include"../HardwareAbstractionLayer/hal.h"
typedef struct NeoPixelStringObject
{
	int numberOfLEDS;
	RGBColor ledColors[MAX_NUMBER_OF_LEDS];
	float intensity;
	float value;

}NeoPixelString;

typedef struct NeoPixelStringObject *NeoPixelStringHandle;
extern NeoPixelStringHandle NeoPixelString_Constructor(void *pmemory, const size_t numbytes, int numberOfLEDs);
void NeoPixelString_setColor(NeoPixelStringHandle handle, int ledNumber, RGBColor color);
void NeoPixelString_diplayValue(NeoPixelStringHandle handle, float value);
void NeoPixelString_draw(NeoPixelStringHandle handle);
void NeoPixelString_rotate(NeoPixelStringHandle handle);
void NeoPixelString_initializeToBasicColors(NeoPixelStringHandle handle);
void NeoPixelString_initializeStringToOneColor(NeoPixelStringHandle handle, RGBColor color);
void NeoPixelString_drawNumber(NeoPixelStringHandle handle,int number);
#endif /* TEMPLATES_CLASS_H_ */
