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

#include "system.h"
#include "usb.h"
#include <xc.h>
#include "bm62.h"



/** CONFIGURATION Bits **********************************************/

// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 3       // PLL Prescaler Selection bits (Divide by 3 (12 MHz oscillator input))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 2       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HSPLL_HS  // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = ON      // USB Voltage Regulator Enable bit (USB voltage regulator enabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = ON    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.




extern LEM_state_t LEM_state;
extern uint8_t button_pushed;
extern uint16_t adc;
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
void SYSTEM_Initialize( SYSTEM_STATE state )
{   
   
    initIO(1);
    
    
    //Configure_HLVD(LEM_state.HLVD_dir);
    

    switch(state)
    {
        case SYSTEM_STATE_USB_START:
#if defined(DEBUG)
                LEM_state.state = LEM_SLEEP;
                Enable_int0(1);
                Configure_ADC(1);
                Configure_CCP2(1);
#endif
            break;
			
        case SYSTEM_STATE_USB_SUSPEND: 
            break;
            
        case SYSTEM_STATE_USB_RESUME:
            break;
        case SYSTEM_STATE_NO_USB:
            LEM_state.HLVD_dir = BATTERY_POWERED;
            Enable_int0(1);
            Configure_ADC(1);
            Configure_CCP2(1);
            INTCONbits.GIE = 1;
            INTCONbits.PEIE = 1;
            break;
    }
}

			
			

void interrupt SYS_InterruptHigh(void)
{
    
    
    if((INTCONbits.INT0IF == 1) && (INTCONbits.INT0IE) )
    {
        INTCONbits.INT0IF = 0;
#ifdef USB_LINK
        button_pushed = 1; //
#else
        service_button(&LEM_state);
#endif
        
    }
    if( (PIR1bits.ADIF == 1) && (PIE1bits.ADIE == 1))
        ADC_int(&LEM_state);
    if( (PIR2bits.HLVDIF == 1) && (PIE2bits.LVDIE == 1))
        HLVD_int(&LEM_state);
#if  defined(USB_LINK) || defined(DEBUG)
        USBDeviceTasks();
#endif
}

/*********************************************************************
* Function: void service_button(LEM_state_t *s)
*
* Overview: Main button Interrupt function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void service_button(LEM_state_t *s)
{
    uint16_t mscpt = 0;
    initIO(1);
    while(PORTBbits.RB0 == 0)
    {
        delayms(s, (uint16_t)1);
        if (mscpt > 600)
            break;
        else        
            mscpt++;
    }
    if(mscpt > 600)
    {
        mscpt = 0;
        LED_RED = LED_ON;
        while(PORTBbits.RB0 == 0)
        {
            delayms(s, (uint16_t)1);
            if (mscpt > 1800)
                break;
            else        
                mscpt++;
        }
        
        
        if(mscpt < 1400)
            long_button_push(s);
        else
            very_long_button_push(s);
        
        while(PORTBbits.RB0 == 0);
    }
    else
    {
        LED_RED = LED_ON;
        short_button_push(s);
        while(PORTBbits.RB0 == 0);
    }
    INTCONbits.INT0IF = 0;
    
}


/*********************************************************************
* Function: void ADC_int(void)
*
* Overview: ADC Interrupt function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void ADC_int(LEM_state_t *s)
{

    PIR1bits.ADIF = 0;
    s->pot_val = (ADRESH << 8) | (ADRESL);
    adc = s->pot_val;
    s->pot_val >>= 3;
    if(s->pot_val != s->pot_val_old)
    {
        Send_volume(s);
        s->pot_val_old = s->pot_val;
    }


}
/*********************************************************************
* Function: void Enable_int0(void)
*
* Overview: Main button Interrupt configuration function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void Enable_int0(uint8_t en)
{
    if (en == 1)
    {
        INTCONbits.INT0IF = 0;
        INTCON2bits.INTEDG0 = 0; //Falling Edge
        INTCON2bits.nRBPU = 0; //PortB Pull-ups
        INTCONbits.INT0IE = 1;
    }
    else
    {
        INTCON2bits.INTEDG0 = 0; //Falling Edge
        INTCON2bits.nRBPU = 0; //PortB Pull-ups
        INTCONbits.INT0IE = 0;
        INTCONbits.INT0IF = 0;
    }
}

/*********************************************************************
* Function: void Enable_int0(void)
*
* Overview: Data coming from the BM62 Interrupt configuration function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void Enable_int1(uint8_t en)
{
    if (en == 1)
    {
        INTCON3bits.INT2IF = 0;
        INTCON2bits.INTEDG2 = 0; //Falling Edge
        INTCON2bits.nRBPU = 0; //PortB Pull-ups
        INTCON3bits.INT2IE = 1;
    }
    else
    {
        INTCON2bits.INTEDG2 = 0; //Falling Edge
        INTCON2bits.nRBPU = 0; //PortB Pull-ups
        INTCON3bits.INT2IE = 0;
        INTCON3bits.INT2IF = 0;
    }
}

/*********************************************************************
* Function: void Configure_ADC(void)
*
* Overview: ADC configuration function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void Configure_ADC(uint8_t en)
{
    ADCON1 = 0x0E;  //All digital except AN0
    ADCON0bits.CHS0 = 0;    //Channel 0 (AN0))
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON2bits.ADFM = 1;    //Right justified
    ADCON2 = (1 << 7) | (ADC_ACQ_TIME << 3) | (ADC_CLOCK_SELECT << 0); //Right justified result
    ADCON0bits.ADON = en & 0x01;
    
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = en & 0x01;
    //delayus(3);
    LEM_state.pot_val = 0;
    ADCON0bits.GO_DONE = 1;
    
}

/*********************************************************************
* Function: void Configure_HLVD(uint8_t direction)
*
* Overview: Main button Interrupt configuration function.
*
* PreCondition: None
*
* Input:  direction of the voltage
*
* Output: None
*
********************************************************************/
void Configure_HLVD(uint8_t direction)
{
    HLVDCONbits.HLVDEN = 0;
    HLVDCON = ((direction & 0x01) << 7) | (1 << 5) | (HLVDL_SETTING);
    HLVDCONbits.HLVDEN = 1;
    PIR2bits.HLVDIF = 0;
    while(HLVDCONbits.IRVST == 0);
    _delay(10);
    PIR2bits.HLVDIF = 0;
    PIE2bits.LVDIE = 1;
    PIR2bits.HLVDIF = 0;
}


/*********************************************************************
* Function: void HLVD_int(void)
*
* Overview: High Low Voltage Detector Interrupt function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void HLVD_int(LEM_state_t *s)
{
    PIR2bits.HLVDIF = 0;
    if(s->HLVD_dir == USB_POWERED)
    {
        
        FastClock(&LEM_state);
        s->HLVD_dir = BATTERY_POWERED; //Next interrupt of HLVD will be when USB unplugged
        Configure_HLVD(s->HLVD_dir);
        LED_RED = LED_ON;
    }
    else
    {
        
        SlowClock(&LEM_state);
        s->HLVD_dir = USB_POWERED; //Next interrupt of HLVD will be when USB plugged back
        Configure_HLVD(s->HLVD_dir);
        LED_RED = LED_OFF;
    }
    
}

/*********************************************************************
* Function: void Configure_CCP1(void)
*
* Overview: Main button Interrupt configuration function.
*
* PreCondition: None
*
* Input:  None
*
* Output: None
*
********************************************************************/
void Configure_CCP2(uint8_t en)
{
    //Compare mode: trigger special event, reset timer, start A/D conversion on CCP1 
    CCP2CON = 0b00001011;
    CCPR2H = ((COMPARE_REG_VAL>>8) & 0xFF);
    CCPR2L = (COMPARE_REG_VAL & 0xFF);
    //8bit write ; not osc source ; 1:8 prescaler ; osc off ; Fosc/4
    T1CON = (0<<7) | (0<<6) | (0b11<<4) | (0<<3) | (1<<2)| (0<<1) | (en&0x01);
    TMR1H = ((COMPARE_REG_VAL >> 8) & 0xFF);
    TMR1L = (COMPARE_REG_VAL & 0xFF) +1;
}



/*******************************************************************
 * Function:    void SlowClock(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    
 *
 * Overview:    Switches the OSC to RCOSC, 32KHz Qz
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void SlowClock(LEM_state_t *s)
{
    USBDeviceDetach();
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF2 = 1;//1MHz
    OSCTUNEbits.INTSRC = 1; //32KHz INT oscillator
    OSCCONbits.SCS0 = 0;
    OSCCONbits.SCS1 = 1;
    while(OSCCONbits.IOFS == 0);
    s->clock_speed = SLOW_CLOCK;
}
/*******************************************************************
 * Function:    void FastClock(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    
 *
 * Overview:    Switches the OSC to PLL
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FastClock(LEM_state_t *s)
{
    OSCCONbits.SCS0 = 0;
    OSCCONbits.SCS1 = 0;
    while(OSCCONbits.OSTS == 0);

    UART_TRISRx=1;				// RX
    UART_TRISTx=0;				// TX
    TXSTA = 0x24;       	// TX enable BRGH=1
    RCSTA = 0x90;       	// Single Character RX
    SPBRG = 0x67;
    SPBRGH = 0x00;      	// 0x0271 for 48MHz -> 19200 baud
    BAUDCON = 0x08;     	// BRG16 = 1
        
    s->clock_speed = FAST_CLOCK;


}


/*******************************************************************
 * Function:    void delayus(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
uint8_t delayms(LEM_state_t *s ,uint16_t del)
{
    if(del == 0)
        return 0;
    if(s->clock_speed == FAST_CLOCK)
    {
        while(del > 0)
        {
            _delay(12000);
            del--;
        }
    }
    else
    {
        while(del > 0)
        {
            _delay(250);//1MHz INTOSC
            del--;
        }
    }
    return(1);
}
/*******************************************************************
 * Function:    void long_button_push(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void long_button_push(LEM_state_t *s)
{
    HPowerOff(s);
}

/*******************************************************************
 * Function:    void short_button_push(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void short_button_push(LEM_state_t *s)
{
    LED_RED = LED_ON;
        //Check is the BM62 is in power off state:
    if((WAKE_PIC == 0) && (PORTCbits.RC6 == 0) && (s->state != LEM_SLEEP))
        s->state = LEM_SLEEP;
    switch(s->state)
    {
        case LEM_SLEEP:
            WAKE_BM62 = 1;
            delayms(s,30);
            RST_N = 1;
            delayms(s,400);
            
            WakeUp(s);
            break;
        case LEM_ON:
            WakeUp(s);
            break;
        case LEM_PAIRING:
            PairStop(s);
            break;
        case LEM_PLAY:
            TogglePause(s);
            break;                
    }   
//    WakeUp(s);
    delayms(s,100);
    LED_RED = LED_OFF;
}

void very_long_button_push(LEM_state_t *s)
{
   
    switch(s->state)
    {
        case LEM_SLEEP:
            WAKE_BM62 = 1;
            delayms(s,30);
            RST_N = 1;
            delayms(s,400);
            WakeUp(s);
            Pair(s);
            break;
        case LEM_ON:
            Pair(s);
            break;
        case LEM_PAIRING:
            Pair(s);
            break;
        case LEM_PLAY:
            Pair(s);
            break;                
    }
    

    delayms(s,100);
    LED_RED = LED_OFF;
    delayms(s,100);
    LED_RED = LED_ON;
    delayms(s,100);
    LED_RED = LED_OFF;
}

void HPowerOff(LEM_state_t *s)
{
    LED_RED = LED_OFF;
    Wakedown(s);
    WAKE_BM62 = 0;
    
    delayms(s,200);
    LED_RED = LED_ON;
    delayms(s,100);
    LED_RED = LED_OFF;
    delayms(s,200);
    LED_RED = LED_ON;
    delayms(s,100);
    LED_RED = LED_OFF;
    delayms(s,200);
    LED_RED = LED_ON;
    delayms(s,100);
    LED_RED = LED_OFF;
    delayms(s,1200);
    RST_N = 0;
    Configure_ADC(0);
    Configure_CCP2(0);
    INTCONbits.PEIE = 0;
    initIO(0);
    OSCCONbits.IDLEN = 0;
    SLEEP();
}

void initIO(uint8_t en)
{
    if(en == 1)
    {
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0xFF;
    TRISD = 0xFD;
    TRISE = 0xFF;
    LATDbits.LATD1 = 0;
    TRISAbits.TRISA3 = 0;   //Reset_N
    TRISAbits.TRISA4 = 0;   //LED3 (Red))
    RST_N = 1;
    LED_RED = LED_OFF;
    
    TRISBbits.TRISB1 = 0;   //WUP
    WAKE_BM62 = 0;
    
    TRISBbits.TRISB4 = 0;
    N_BTN_VOL_UP = 1;
    TRISBbits.TRISB5 = 0;
    N_BTN_VOL_DN = 1;
    TRISCbits.TRISC2 = 0;
    N_BTN_FORWARD = 1;
    TRISAbits.TRISA1 = 0;
    N_BTN_BACKWARD = 1;    
    TRISAbits.TRISA2 = 0;
    N_BTN_PAIR = 1;  
    }
    else
    {
        TRISA = 0x01;
        TRISB = 0x01;
        TRISC = 0x00;
        TRISD = 0x00;
        TRISE = 0x00;
        LATA = 0x10; //switch off led3
        LATB = 0;
        LATC = 0;
        LATD = 0;
        LATDbits.LATD1 = 1; //Disable the potentiometer
        LATE = 0;
        
    }
}