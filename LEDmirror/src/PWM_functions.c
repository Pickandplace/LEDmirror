/*
 * PWM_functions.c
 *
 * Created: 1/2/2015 3:34:23 PM
 *  Author: Jean
* Under the CC BY-NC-SA license
* Jean Wlodarski
* Pickandplace.wordpress.com
 */ 
#include <asf.h>
#include "PWM_functions.h"

void increaseAllOneIncrement(uint8_t *leds)
{
	uint8_t i;
	
	for (i=0;i<12;i++)
	{
		if (leds[i] < 255)
		leds[i] ++;
	}
}

void increaseAllToValueOneIncrement(uint8_t value, uint8_t *leds)
{
	uint8_t i;
	
	for (i=0;i<12;i++)
	{
		if (leds[i] < value)
		leds[i] ++;
	}
}

void increaseAllToValue(uint8_t value, uint8_t *leds)
{
	uint8_t i;
	
	for (i=0;i<12;i++)
	{
		while (leds[i] < value)
		leds[i] ++;
	}
}

void ledsZero( uint8_t *leds)
{
	uint8_t i;
	
	for (i=0;i<12;i++)
	{
		leds[i] = 0;
	}
}


void assignAllToValue(uint8_t value, uint8_t *leds)
{
	uint8_t i;
	
	for (i=0;i<12;i++)
	{
		leds[i] = value;
	}
}

void decreaseAllOneIncrement(uint8_t *leds)
{
	uint8_t i;
	
	for (i=0;i<12;i++)
	{
		if (leds[i] > 0)
		leds[i] --;
	}
}