/*
 * AT42QT2160.h
 *
 * Created: 6/22/2014 6:13:16 PM
 *  Author: Jean
 * Under the CC BY-NC-SA license
 * Jean Wlodarski
 * Pickandplace.wordpress.com
 */ 


#ifndef AT42QT2160_H_
#define AT42QT2160_H_

#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
#define TWI_SPEED        50000
#define TWI_MASTER_ADDR  0x50
#define DATA_LENGTH     20



#define AT42QT2160_ADD	0x44
#define AT42QT2160_ADD_NTHR	38

#define AT42QT2160_ADD_BURST_LENGTH	54
#define AT42QT2160_ADD_DETECT_INTEGRATOR	17
#define AT42QT2160_ADD_DRIFT_CALIBRATE	10
#define AT42QT2160_ADD_DRIFT_COMP	13
#define AT42QT2160_ADD_KEY_CONTROL	19
#define AT42QT2160_ADD_SLIDER_CONTROL	20
#define AT42QT2160_ADD_SLIDER_OPTIONS	21
#define AT42QT2160_ADD_GPIO_DIRECTION	73

#define AT42QT2160_NTHR_VAL	9
#define AT42QT2160_BURST_LENGTH_VAL	16
#define BURST_LENGTH    8
#define BURST_REPETITION    4


void AT42QT2160init(void);
uint8_t AT42QT2160readSlider(void);
uint8_t AT42QT2160readStatusBytes(void);

#endif /* AT42QT2160_H_ */