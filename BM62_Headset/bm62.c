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

#include "bm62.h"
#include "app_device_cdc_to_uart.h"
#include "usart.h"
volatile unsigned char bm62_data_tx[BM62_RX_BUFFER];

extern uint8_t sendUSB;
extern LEM_state_t LEM_state;
void Send_volume(LEM_state_t *s)
{
    //bm62SendVolume((uint8_t)s->pot_val);
    bm62AbsVolume((uint8_t)s->pot_val);
}

void VolumeUp(LEM_state_t *s)
{
    char buf[]={0xAA, 0x00, 0x07, 0x23, 0x00, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xD7};
    
    bm62Puts(buf, 11);
}

void VolumeDown(LEM_state_t *s)
{
   char buf[]={0xAA, 0x00, 0x07, 0x23, 0x00, 0x01, 0x02, 0xFF, 0xFF, 0xFF, 0xD6};
    
   bm62Puts(buf, 11);

}

void WakeUp(LEM_state_t *s)
{
    char buf1[]={0xAA, 0x00, 0x03, 0x02, 0x00, 0x51, 0xAA };
    char buf2[]={0xAA, 0x00, 0x03, 0x02, 0x00, 0x52, 0xA9 };
    initIO(1);
    SlowClock(s);
    CCPR2H = (((COMPARE_REG_VAL)>>8) & 0xFF);
    CCPR2L = ((COMPARE_REG_VAL) & 0xFF);
    Configure_ADC(1);
    Configure_CCP2(1);
    INTCONbits.PEIE = 1;
    bm62Puts(buf1, 7);
    bm62Puts(buf2, 7);
    s->state = LEM_ON;
}
void Wakedown(LEM_state_t *s)
{  
    char buf1[]={0xAA, 0x00, 0x03, 0x02, 0x00, 0x53, 0xA8 };
    char buf2[]={0xAA, 0x00, 0x03, 0x02, 0x00, 0x54, 0xA7 };
    bm62Puts(buf1, 7);
    bm62Puts(buf2, 7);   
    s->state = LEM_SLEEP;
}
void Pair(LEM_state_t *s)
{
    char buf[]={0xAA, 0x00, 0x03, 0x02, 0x00, 0x5D, 0x9E};    
    bm62Puts(buf, 7);
    s->state = LEM_PAIRING;
}
void PairStop(LEM_state_t *s)
{
    char buf[7];
    buf[0] = 0xAA;   
    buf[1] = 0x00;
    buf[2] = 0x03;
    buf[3] = 0x02;
    buf[4] = 0x00;
    buf[5] = 0x6B;
    buf[6] = calculateChecksum(&buf[1], &buf[5]);
    bm62Puts(buf, 7);
    s->state = LEM_ON;
}
void TogglePause(LEM_state_t *s)
{
    char buf[7];
    buf[0] = 0xAA;   
    buf[1] = 0x00;
    buf[2] = 0x03;
    buf[3] = 0x04;//music control
    buf[4] = 0x04;//reserved ??
    buf[5] = 0x07;//play pause toggle
    buf[6] = calculateChecksum(&buf[1], &buf[5]);
    bm62Puts(buf, 7);
}

void PlayPlay(LEM_state_t *s)
{
    N_BTN_VOL_UP = 1;
    N_BTN_VOL_UP = 0;
    delayms(s,LONG_BUTTON_PUSH);
    N_BTN_VOL_UP = 1;
    s->state = LEM_PLAY;
}

void bm62Puts(char *c, uint8_t len)
{
    uint8_t ptr = 0;
    
    WAKE_BM62 = 0;
    WAKE_BM62 = 1;
    delayms(&LEM_state,2);
    WAKE_BM62 = 0;
    delayms(&LEM_state,4);
    WAKE_BM62 = 1;
    delayms(&LEM_state,4);
    
    if(LEM_state.clock_speed == SLOW_CLOCK)
    {
        OSCCONbits.IRCF0 = 1;
        OSCCONbits.IRCF1 = 1;
        OSCCONbits.IRCF2 = 1;//8MHz
        OSCTUNEbits.INTSRC = 1; //32KHz INT oscillator
        OSCCONbits.SCS0 = 0;
        OSCCONbits.SCS1 = 1;
        while(OSCCONbits.IOFS == 0);

        UART_TRISRx=1;				// RX
        UART_TRISTx=0;				// TX
        TXSTA = 0x24;       	// TX enable BRGH=1
        RCSTA = 0x90;       	// Single Character RX
        SPBRG = 16;
        SPBRGH = 0x00;      	//  
        BAUDCON = 0x08;     	// BRG16 = 1
    }
        while(ptr < len)
        {
            while(TXSTAbits.TRMT == 0);
            TXREG = c[ptr];
            while(TXSTAbits.TRMT == 0);
            ptr++;
        }
    if(LEM_state.clock_speed == SLOW_CLOCK)
    { 
        OSCCONbits.IRCF0 = 0;
        OSCCONbits.IRCF1 = 0;
        OSCCONbits.IRCF2 = 1;//1MHz
        OSCTUNEbits.INTSRC = 1; //32KHz INT oscillator
        OSCCONbits.SCS0 = 0;
        OSCCONbits.SCS1 = 1;
        while(OSCCONbits.IOFS == 0);
        
    }
    WAKE_BM62 = 0;
    delayms(&LEM_state,50);

}

void bm62SendVolume(uint8_t volume)
{
    char buffer[11];
    buffer[0] = 0xAA;
    buffer[1] = 0x00; //MSB Length
    buffer[2] = 0x07; //LSB Length
    buffer[3] = 0x23; //Command
    buffer[4] = 0x00; //database 0 ?
    buffer[5] = 0x01; //A2DP gain
    buffer[6] = 0x03; //Absolute gain level
    buffer[7] = (volume & 0x0F); //
    buffer[8] = (volume & 0x0F); //
    buffer[9] = (volume & 0x0F); //
    buffer[10] = calculateChecksum(&buffer[1], &buffer[9]); //checksum
    bm62Puts(buffer, 11);
         
}
void bm62AbsVolume(uint8_t volume)
{
    char buffer[11];
    buffer[0] = 0xAA;
    buffer[1] = 0x00; //MSB Length
    buffer[2] = 0x07; //LSB Length
    buffer[3] = 0x23; //Command
    buffer[4] = 0x00; //database 0 ?
    buffer[5] = 0x01; //A2DP gain
    buffer[6] = 0x04; //Absolute gain level
    buffer[7] = ((volume<<2) & 0x7F); //
    buffer[8] = ((volume<<2) & 0x7F); //
    buffer[9] = ((volume<<2) & 0x7F); //
    buffer[10] = calculateChecksum(&buffer[1], &buffer[9]); //checksum
    bm62Puts(buffer, 11);
         
}
uint8_t calculateChecksum(uint8_t* startByte, uint8_t* endByte)
{
    uint8_t checksum = 0;
    while(startByte <= endByte)
    {
        checksum += *startByte;
        startByte++;
    }
    checksum = ~checksum + 1;
    return checksum;
}