/*******************************************************************************
 * Bluetooth Mod of a DR 80 CR Headset
Copyright (C) 2017 Jean Wlodarski
 KaZjjW at gmail dot com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/

#ifndef SYSTEM_H
#define SYSTEM_H
//#define USB_LINK
//#define DEBUG



#include <xc.h>
#include <stdbool.h>


#include "io_mapping.h"
#include "fixed_address_memory.h"
#include "usb_config.h"

#include "usart.h"
#include "stdint.h"
#define MAIN_RETURN void
/*ACQT2:ACQT0: A/D Acquisition Time Select bits 
 * 111 = 20 TAD
 * 110 = 16 TAD
 * 101 = 12 TAD
 * 100 = 8 TAD 
 * 011 = 6 TAD 
 * 010 = 4 TAD 
 * 001 = 2 TAD 
 * 000 = 0 TAD
*/
#define ADC_ACQ_TIME 0b110
/*ADCS2:ADCS0: A/D Conversion Clock Select bits
 * 111 = FRC (clock derived from A/D RC oscillator)(1) 
 * 110 = FOSC/64
 * 101 = FOSC/16
 * 100 = FOSC/4
 * 011 = FRC (clock derived from A/D RC oscillator)(1) 
 * 010 = FOSC/32
 * 001 = FOSC/8
 * 000 = FOSC/2
 */
#define ADC_CLOCK_SELECT 0b000

#define HLVDL_SETTING 0b1000

#define COMPARE_REG_VAL 0xFF00

#define USB_POWERED 0
#define BATTERY_POWERED 1
#define SLOW_CLOCK  0
#define FAST_CLOCK  1
#define LED_OFF 1
#define LED_ON  0
/*** System States **************************************************/
typedef enum
{
    SYSTEM_STATE_USB_START,
    SYSTEM_STATE_USB_SUSPEND,
    SYSTEM_STATE_USB_RESUME,
    SYSTEM_STATE_NO_USB
} SYSTEM_STATE;

typedef enum
{
    LEM_SLEEP,
    LEM_ON,
    LEM_PAIRING,
    LEM_PLAY
} LEM_STATE;

/*********************************************************************
* Function: void SYSTEM_Initialize( SYSTEM_STATE state )
*
* Overview: Initializes the system.
*
* PreCondition: None
*
* Input:  SYSTEM_STATE - the state to initialize the system into
*
* Output: None
*
********************************************************************/
void SYSTEM_Initialize( SYSTEM_STATE state );

/*********************************************************************
* Function: void SYSTEM_Tasks(void)
*
* Overview: Runs system level tasks that keep the system running
*
* PreCondition: System has been initialized with SYSTEM_Initialize()
*
* Input: None
*
* Output: None
*
********************************************************************/
//void SYSTEM_Tasks(void);
#define SYSTEM_Tasks();


// *****************************************************************************
/* Device state structure

*/
typedef struct LEM
{
    uint16_t    pot_val;    //Value of the volume 
    uint16_t    pot_val_old;    //Past value of the volume 
    uint8_t     HLVD_dir;   // Direction of the HLV Detector
    uint8_t     clock_speed;    //Fast (USB) or slow clock (Battery)
	uint8_t		state;		//LEM_STATE enum
} LEM_state_t;


//void Button_int(LEM_state_t *s);
void ADC_int(LEM_state_t *s);
void Enable_int0(uint8_t en);
void Configure_ADC(uint8_t en);
void Configure_HLVD(uint8_t direction);
void HLVD_int(LEM_state_t *s);
void Configure_CCP2(uint8_t en);
void SlowClock(LEM_state_t *s);
void FastClock(LEM_state_t *s);
uint8_t delayms(LEM_state_t *s ,uint16_t del);
void long_button_push(LEM_state_t *s);
void short_button_push(LEM_state_t *s);
void service_button(LEM_state_t *s);
void very_long_button_push(LEM_state_t *s);
void initIO(uint8_t en);
void HPowerOff(LEM_state_t *s);
#endif //SYSTEM_H
