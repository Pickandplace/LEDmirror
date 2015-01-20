/*
 * PWM_functions.h
 *
 * Created: 1/2/2015 3:34:43 PM
 *  Author: Jean
 */ 


#ifndef PWM_FUNCTIONS_H_
#define PWM_FUNCTIONS_H_

extern  volatile uint8_t leds[12];

void increaseAllOneIncrement(volatile uint8_t *leds);


void increaseAllToValueOneIncrement(uint8_t value,volatile uint8_t *leds);


void increaseAllToValue(uint8_t value, volatile uint8_t *leds);


void ledsZero( volatile uint8_t *leds);



void assignAllToValue(uint8_t value, volatile uint8_t *leds);

void decreaseAllOneIncrement( volatile uint8_t *leds);

#endif /* PWM_FUNCTIONS_H_ */