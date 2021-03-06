/*
 * class.c
 *
 *  Created on: Aug 12, 2016
 *      Author: JohnTaylor
 */


#include "NeoPixelString.h"

NeoPixelStringHandle NeoPixelString_Constructor(void *pmemory, const size_t numbytes,int numberOfLEDs)
{
	NeoPixelStringHandle handle;
	NeoPixelString *obj;

	if(numbytes < sizeof(NeoPixelString))
	{
		return ((NeoPixelStringHandle)NULL);
	}

	handle = (NeoPixelStringHandle)pmemory;
	obj = (NeoPixelString *)handle;
	obj->numberOfLEDS = numberOfLEDs;
	obj->intensity = 50.0;
	NeoPixelString_initializeToBasicColors(handle);
	NeoPixelString_draw(handle);

	return handle;
}
void NeoPixelString_setColor(NeoPixelStringHandle handle, int ledNumber, RGBColor color)
{
	NeoPixelString *obj = (NeoPixelString *)handle;
	if(ledNumber < MAX_NUMBER_OF_LEDS)
	{
		obj->ledColors[ledNumber] = color;
	}
}
void NeoPixelString_rotate(NeoPixelStringHandle handle)
{
	NeoPixelString *obj = (NeoPixelString *)handle;
	int i;
	RGBColor colorOfLEDZero = obj->ledColors[0];


	for(i = 0; i < obj->numberOfLEDS - 1; i++)
	{
		obj->ledColors[i] = obj->ledColors[i+1];
	}
	obj->ledColors[obj->numberOfLEDS -1] = colorOfLEDZero;
}
void NeoPixelString_initializeStringToOneColor(NeoPixelStringHandle handle, RGBColor color)
{
	NeoPixelString *obj = (NeoPixelString *)handle;
	int i = 0;
	for(i = 0; i < obj->numberOfLEDS; i++)
	{
		obj->ledColors[i] = color;
	}
}
void NeoPixelString_initializeToBasicColors(NeoPixelStringHandle handle)
{
	NeoPixelString *obj = (NeoPixelString *)handle;
	int i = 0;
	for(i = 0; i < obj->numberOfLEDS; i++)
	{
		if(i < 3)
		{
			obj->ledColors[i] = WHITE;
		}
		if(i >= 3 && i < 6)
		{
			obj->ledColors[i] = ORANGE;
		}
		if(i >= 6 && i < 9)
		{
			obj->ledColors[i] = WHITE;
		}
		if(i >= 9 && i < 12)
		{
			obj->ledColors[i] = YELLOW;
		}
		if(i >= 12 && i < 15)
		{
			obj->ledColors[i] = ORANGE;
		}

	}
}

void NeoPixelString_draw(NeoPixelStringHandle handle)
{
	NeoPixelString *obj = (NeoPixelString *)handle;
	int i = 0;
	for(i = 0; i < obj->numberOfLEDS; i++)
	{
		HAL_sendRGB(obj->ledColors[i].red,obj->ledColors[i].green,obj->ledColors[i].blue,obj->intensity);
	}
}

