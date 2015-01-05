/*
 * PWM_functions.h
 *
 * Created: 1/2/2015 3:34:43 PM
 *  Author: Jean
 */ 


#ifndef PWM_FUNCTIONS_H_
#define PWM_FUNCTIONS_H_

extern uint8_t leds[12];

void increaseAllOneIncrement(uint8_t *leds);


void increaseAllToValueOneIncrement(uint8_t value, uint8_t *leds);


void increaseAllToValue(uint8_t value, uint8_t *leds);


void ledsZero(uint8_t *leds);



void assignAllToValue(uint8_t value, uint8_t *leds);

void decreaseAllOneIncrement(uint8_t *leds);

#endif /* PWM_FUNCTIONS_H_ */