/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */
/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <stdint.h>
//#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <asf.h>
#include "main.h"
#include <twi_master_driver.h>
#include "AT42QT2160.h"
#include <conf_usb.h>
#include "ui.h"
#include "uart.h"
#include "PWM_functions.h"


#define F_SYS   32000000
#define BAUDRATE        300000
#define TWI_BAUDSETTING TWI_BAUD(F_SYS, BAUDRATE)
#define STARTUP_LIGHT_INTENSITY 50
#define ANIMATION_EEPROM_ADDRESS 0x0
#define MY_ADC ADCA
#define MY_ADC_CH ADC_CH0

#define HEATING_ON	PORTD_OUTSET = PIN0_bm;
#define HEATING_OFF	PORTD_OUTCLR = PIN0_bm;
#define HEATING_TGL	PORTD_OUTTGL = PIN0_bm;


static void PWMcounterCallback(void);
static void adc_init(void);


struct adc_config adc_conf;
struct adc_channel_config adcch_conf;
	
uint8_t USB_port;
uint8_t led1,led2,led3,led4,led5,led6,led7,led8,led9,led10,led11,led12,slider;
uint8_t pwm0, slider;


TWI_Master_t twiMaster;
uint8_t twiBuffer[10];

uint8_t animation,indexAnimation2,StartupLight;
uint8_t leds[12];

static volatile bool main_b_cdc_enable = false;

char ascii[3];

uint8_t Counter125ms, timeCheck, ADCstartTimer, HeatingTimer, HeatingTimer2;
enum months_names {JANUARY, FEBRUARY, MARCH, APRIL, MAY, JUNE, JULY, AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER};
enum clock_modes {CLOCK_OFF, CLOCK_ON};
struct calendar_date date = {
	.second = 00,
	.minute = 28,
	.hour = 16,
	.date = 01,
	.month = JANUARY,
	.year = 2015
};

struct calendar_date  *now ;

uint16_t Thermistor1;
uint8_t Temperature1, Temperature1Prec;
uint16_t therm[] = {4165,3776,3393,3026,2682,2366,2080,1826,1602,1407,1238,1093,968 ,862,772,694,628,572,524,483,448,418,392,369,350,333};
static void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	uint8_t i;
	Thermistor1 = result;
	//adcch_disable_interrupt(&adcch_conf);
	
	for(i=0;i<126;i++)
	{
		if(Thermistor1 > therm[i])
			{
				Temperature1 = i*5;
				if(i>0)
					if( (Thermistor1 - therm[i] ) < ((therm[i-1] - therm[i]) >> 1) )
						Temperature1 += 3;
				Temperature1Prec = ((therm[i-1] - therm[i]) >> 1);
				break;
			}
	}
}


ISR(TWIE_TWIM_vect)
    {
      TWI_MasterInterruptHandler(&twiMaster);
    }

static void GeneralPurposeTmrCallback(void)
{
	PORTD_DIRSET = PIN1_bm;
	PORTD_OUTSET = PIN1_bm;
	Counter125ms++;
	timeCheck++;
	ADCstartTimer++;
	HeatingTimer++;
	PORTD_OUTCLR = PIN1_bm;
}

static void PWMcounterCallback(void)
{

	if(pwm0 >= 255)
		pwm0 = 1;
	else
		pwm0++;
	
	if(leds[0] >= pwm0)
		PORTB_OUTSET = PIN0_bm;
	else
		PORTB_OUTCLR = PIN0_bm;
	if(leds[1] >= pwm0)
	PORTB_OUTSET = PIN1_bm;
	else
	PORTB_OUTCLR = PIN1_bm;
	
	if(leds[2] >= pwm0)
	PORTB_OUTSET = PIN2_bm;
	else
	PORTB_OUTCLR = PIN2_bm;
	
	if(leds[3] >= pwm0)
	PORTB_OUTSET = PIN3_bm;
	else
	PORTB_OUTCLR = PIN3_bm;
	
	if(leds[4] >= pwm0)
	PORTB_OUTSET = PIN4_bm;
	else
	PORTB_OUTCLR = PIN4_bm;
	
	if(leds[5] >= pwm0)
	PORTB_OUTSET = PIN5_bm;
	else
	PORTB_OUTCLR = PIN5_bm;
	
	if(leds[6] >= pwm0)
	PORTB_OUTSET = PIN6_bm;
	else
	PORTB_OUTCLR = PIN6_bm;
	
	if(leds[7] >= pwm0)
	PORTB_OUTSET = PIN7_bm;
	else
	PORTB_OUTCLR = PIN7_bm;
	
	if(leds[8] >= pwm0)
	PORTC_OUTSET = PIN0_bm;
	else
	PORTC_OUTCLR = PIN0_bm;
	
	if(leds[9] >= pwm0)
	PORTC_OUTSET = PIN1_bm;
	else
	PORTC_OUTCLR = PIN1_bm;
	
	if(leds[10] >= pwm0)
	PORTC_OUTSET = PIN2_bm;
	else
	PORTC_OUTCLR = PIN2_bm;
	
 	if(leds[11] >= pwm0)
 	PORTC_OUTSET = PIN3_bm;
 	else
 	PORTC_OUTCLR = PIN3_bm;

}

static void AnimationTimer(void)
{
	uint8_t i;
	switch (animation)
	{
		//Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY at the same time
		case 1:
		if(leds[0] < StartupLight)
		increaseAllOneIncrement(leds);
		else{
			tc_disable(&TCC1);
			animation++;
		nvm_write_char(INT_EEPROM,ANIMATION_EEPROM_ADDRESS,animation);}
		break;
		
		case 2://Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY one after the other
		if(leds[indexAnimation2] < StartupLight)
		leds[indexAnimation2]++;
		else
		indexAnimation2++;
		if(indexAnimation2 > 11){
			tc_disable(&TCC1);
			animation++;
		nvm_write_char(INT_EEPROM,ANIMATION_EEPROM_ADDRESS,animation);}
		break;
		
		case 3://Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY one after the other, the second starting at the half of the previous
		if(leds[indexAnimation2] < StartupLight)
		leds[indexAnimation2]++;
		else
		indexAnimation2++;
		if( (leds[indexAnimation2] > (StartupLight /2) ) && (indexAnimation2<10) )
		leds[indexAnimation2+1]++;
		
		if(indexAnimation2 > 11){
			tc_disable(&TCC1);
			animation++;
		nvm_write_char(INT_EEPROM,ANIMATION_EEPROM_ADDRESS,animation);}
		break;
		
		case 4://Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY one after the other, slowed down
		
		tc_write_period(&TCC1, 1024);
		leds[indexAnimation2]++;
		indexAnimation2++;
		
		if (indexAnimation2 >11)
		indexAnimation2 = 0;
		
		if(leds[0] >= StartupLight ){
			tc_disable(&TCC1);
			animation=1;
		nvm_write_char(INT_EEPROM,ANIMATION_EEPROM_ADDRESS,animation);}
		break;
		
		default:
		
		break;
	}
	
}

int main (void)
{
	uint8_t   oldSlider, vbat_status, taps, ClockMode;
	uint32_t time_now,tmp, time_old;
	struct calendar_date *now;
	status_code_t status;
	
	
	PORTC_OUTCLR = 0xFF;
	PORTB_OUTCLR = 0xFF;
	PORTC_DIR = 0xff;
	PORTB_DIR = 0xff;
	PORTC_OUTCLR = 0xFF;
	PORTB_OUTCLR = 0xFF;
	PORTD_OUTCLR = PIN0_bm;
	PORTD_DIRSET = PIN0_bm;
	board_init();
	pmic_init();
	
	sysclk_init();
	cpu_irq_enable();
	sleepmgr_init();
	
		now = malloc(sizeof(struct calendar_date));
	*now = date;
	
	vbat_status = rtc_vbat_system_check(false);
	time_now = calendar_date_to_timestamp(now);
	//rtc_init();
	//rtc_set_time(calendar_date_to_timestamp(&date));
	/* 
	* Depending on the VBAT system check appropriate actions need to 
	* be taken.
	* In this example we re-initialize the VBAT system in all
	* error cases.
	*/
	switch (vbat_status)
	{
	case VBAT_STATUS_OK:
		time_now = rtc_get_time();
		calendar_timestamp_to_date( time_now , now);
		
		if( (now->month >= MARCH) && (now->month <= AUGUST)) //Summer
		{
			if((now->hour >= 7) && (now->hour <= 18)) //Day
				StartupLight = 60;
			else //Night
				StartupLight = 40;
		}
		else //Winter
		{
			if((now->hour > 8) && (now->hour <= 16)) //Day
				StartupLight = 60;
			else //Night
				StartupLight = 40;
		}
		break;
	case VBAT_STATUS_NO_POWER: // fall through
	case VBAT_STATUS_BBPOR: // fall through
	case VBAT_STATUS_BBBOD: // fall through
	case VBAT_STATUS_XOSCFAIL: // fall through
	default:
		rtc_init();
		rtc_set_time(calendar_date_to_timestamp(&date));
		StartupLight = 50;
		break;
	}

	slider = StartupLight;
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	PMIC.CTRL |= PMIC_HILVLEN_bm;
	sei();
	
	status = nvm_init(INT_EEPROM);
	if(status == STATUS_OK)
	{
		nvm_read_char(INT_EEPROM,ANIMATION_EEPROM_ADDRESS,&animation);
		if (animation > 4)
		{
			animation = 1;
		}
	}
	else
	{
		animation = 1;
	}
	
	
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, PWMcounterCallback);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 200);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_MED);
	cpu_irq_enable();
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV2_gc);
	
	tc_enable(&TCC1);
	tc_set_overflow_interrupt_callback(&TCC1, AnimationTimer);
	tc_set_wgm(&TCC1, TC_WG_NORMAL);
	tc_write_period(&TCC1, 1024);
	tc_set_overflow_interrupt_level(&TCC1, TC_INT_LVL_LO);
	tc_write_clock_source(&TCC1, TC_CLKSEL_DIV256_gc);
	tc_disable(&TCC1);
	irq_initialize_vectors();

	sysclk_enable_peripheral_clock(&TWI_MASTER);
	TWI_MasterInit(&twiMaster, &TWIE, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);
	TWIE.CTRL = 6;

	
	PORTE_DIRCLR = PIN2_bm;
	PORTE_PIN2CTRL = PORT_OPC_PULLUP_gc;
	TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
	
	AT42QT2160init();
	
	AT42QT2160readStatusBytes();
	slider =  AT42QT2160readSlider();
	
	USB_port = 0;
	ui_init();
	udc_start();

	Counter125ms = 0;
	ADCstartTimer = 0;
	tc_enable(&TCD0);
	tc_set_overflow_interrupt_callback(&TCD0, GeneralPurposeTmrCallback);
	tc_set_wgm(&TCD0, TC_WG_NORMAL);
	tc_write_period(&TCD0, 0xFFFF);
	tc_set_overflow_interrupt_level(&TCD0, TC_INT_LVL_MED);
	tc_write_clock_source(&TCD0, TC_CLKSEL_DIV64_gc);	//125.4ms
	tc_reset(&TCD0);

	tc_enable(&TCC1);
	tc_set_overflow_interrupt_callback(&TCC1, AnimationTimer);
	tc_set_wgm(&TCC1, TC_WG_NORMAL);
	tc_write_period(&TCC1, 1024);
	tc_set_overflow_interrupt_level(&TCC1, TC_INT_LVL_HI);
	tc_write_clock_source(&TCC1, TC_CLKSEL_DIV256_gc);
	tc_disable(&TCC1);
	
	oldSlider = 0;
	taps = 0;
	ClockMode = CLOCK_OFF;
	indexAnimation2 = 0;
	//MenuPos = 0;
	adc_init(); 
	adc_enable(&MY_ADC);
	tc_enable(&TCC1);
	
	
	HEATING_ON
	
	while(1)
	{

		
		if( !(PORTE.IN & PIN2_bm))
		{
			slider =  AT42QT2160readSlider();
	/*		if( ((slider | 0x03) == (oldSlider | 0x03)) && (slider > 10 ) && (slider < 200 ))
			{
				switch (taps)
				{
					case 0 :
					Counter125ms = 0;
					taps++;
					break;
					
					case 1 :
					if ( (Counter125ms >= 1) &&( Counter125ms <= 5) ){
						taps++;
					tmp = tc_read_count(&TCD0);}
					else
					{
						Counter125ms = 0;
						taps = 0;
					}
					break;
					
					case 2 :
					if ( Counter125ms >= 2)
					{
						ClockMode = ClockMode ^ 1;
						Counter125ms = 0;
					}
					taps = 0;
					Counter125ms = 0;
					break;
				}
			}
			else
			{
				taps = 0;
				oldSlider = slider;
				Counter125ms = 0;
			}*/
			if(slider < 5){
				slider = 0;
				assignAllToValue(slider, leds);
			}
			else{
				time_now = rtc_get_time();
				calendar_timestamp_to_date( time_now , now);
				//cli();
				assignAllToValue(slider, leds);
				/*if (ClockMode == CLOCK_ON)
				{
					if(now->hour > 11)
					leds[now->hour - 12] = 1;
					else
					leds[now->hour] = 1;
					
					leds[((now->minute) / 5)] = slider + 30;
				}
				sei();*/
				
			}
			AT42QT2160readStatusBytes();
			if(Counter125ms > 20)
			Counter125ms = 0;
		}
		/*
		if((slider >= 5) && (ClockMode == CLOCK_ON))
		{
			time_now = rtc_get_time();
			calendar_timestamp_to_date( time_now , now);
			cli();
			assignAllToValue(slider, leds);
			if(now->hour > 11)
			leds[now->hour - 12] = 1;
			else
			leds[now->hour] = 1;
			
			leds[((now->minute) / 5)] = slider + 30;
			sei();
		}
		*/
		if ((timeCheck >= 3) && (USB_port == 1))
		{
			time_now = rtc_get_time();
			if (time_now != time_old)
			{
				time_old = time_now;
				Display_ui();
			}
			timeCheck = 0;
		}
		
		if (ADCstartTimer >= 32)
		{
			ADCstartTimer = 0;
			adcch_enable_interrupt(&adcch_conf);
			adc_start_conversion(&MY_ADC, MY_ADC_CH);
			
		}
		
		if(HeatingTimer >= 172) //30 seconds
		{
			HeatingTimer = 0;
			HeatingTimer2++;
			if((HeatingTimer2 >= 6) && ((PORTD_IN & PIN0_bm) == PIN0_bm))//turn off after 3 minutes
			{
				HeatingTimer2 = 0;
				HEATING_OFF
			}
			if((HeatingTimer2 >= 2) && ((PORTD_IN & PIN0_bm) == 0))//turn on after 1 minute
			{
				HeatingTimer2 = 0;
				HEATING_ON
			}
			
		}
		
	}
	
	
}


void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
	return;
	ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	ui_wakeup_enable();
}
#endif

bool main_cdc_enable(uint8_t port)
{	
	main_b_cdc_enable = true;
	// Open communication
	//uart_open(port);
	//Display_ui();
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	//uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
		USB_port = 1;
		}else{
		// Host terminal has close COM
		ui_com_close(port);
		USB_port = 0;
	}
}

static void adc_init(void)
{

	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,
	ADC_REF_BANDGAP);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 5000UL);
	adc_set_callback(&MY_ADC, &adc_handler);
	adc_write_configuration(&MY_ADC, &adc_conf);
	
	adcch_enable_interrupt(&adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	//adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	adc_start_conversion(&MY_ADC, MY_ADC_CH);
}