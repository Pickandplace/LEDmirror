/**
 * \file
 *
 * \brief LED Mirror main.c 
 * Under the CC BY-NC-SA license
 * Jean Wlodarski
 * Pickandplace.wordpress.com
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

#define ANIMATION_EEPROM_ADDRESS 0x01
#define EEPROM_FLAG_ADDRESS 0x00
#define EEPROM_FLAG 0xDE
#define EEPROM_PWM_FREQ_LSB_ADDRESS 0x02
#define EEPROM_PWM_FREQ_MSB_ADDRESS 0x03
#define PWM_FREQ_DEFAULT 300

#define EEPROM_PROFILE_ADDRESS 0x01

#define MY_ADC ADCA
#define MY_ADC_CH ADC_CH0

#define HEATING_ON	PORTD_OUTSET = PIN0_bm;
#define HEATING_OFF	PORTD_OUTCLR = PIN0_bm;
#define HEATING_TGL	PORTD_OUTTGL = PIN0_bm;

void enter_bootloader(void); //Jump to Bootloader
static void PWMcounterCallback(void);
static void adc_init(void);


struct adc_config adc_conf;
struct adc_channel_config adcch_conf;


volatile uint8_t USB_port, GO_TO_BOOTLOADER;
volatile uint8_t led1,led2,led3,led4,led5,led6,led7,led8,led9,led10,led11,led12,slider;
volatile uint8_t pwm0;


TWI_Master_t twiMaster;
uint8_t twiBuffer[10];

uint8_t indexAnimation2,StartupLight,StartupTimer;
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

struct saved_data profile = {
 .animation = 0,						
 .pwm_freq = 300,	
 .at42qt2160_negative_threshold = 9,
 .at42qt2160_burst_length = 16,
 .at42qt2160_burst_repetition = 4,
 .at42qt2160_detect_integrator_NDIL = 6
};

struct saved_data  *profile_ptr ;

uint16_t Thermistor1;
uint8_t Temperature1, Temperature1Prec,NewPWMval;
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

	Counter125ms++;
	timeCheck++;
	ADCstartTimer++;
	HeatingTimer++;
	if(StartupTimer <= 100)
		StartupTimer++;

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
	PORTC_OUTSET = PIN1_bm;
	else
	PORTC_OUTCLR = PIN1_bm;
	
	if(leds[9] >= pwm0)
	PORTC_OUTSET = PIN2_bm;
	else
	PORTC_OUTCLR = PIN2_bm;
	
	if(leds[10] >= pwm0)
	PORTC_OUTSET = PIN3_bm;
	else
	PORTC_OUTCLR = PIN3_bm;
	
 	if(leds[11] >= pwm0)
 	PORTC_OUTSET = PIN4_bm;
 	else
 	PORTC_OUTCLR = PIN4_bm;

}

static void AnimationTimer(void)
{
	uint8_t i;
	switch (profile.animation)
	{
		//Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY at the same time
		case 1:
		if(leds[0] < StartupLight)
		increaseAllOneIncrement(leds);
		else{
			tc_disable(&TCC1);
			profile.animation++;
			Save_profile(&profile);
		}
		break;
		
		case 2://Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY one after the other
		if(leds[indexAnimation2] < StartupLight)
		leds[indexAnimation2]++;
		else
		indexAnimation2++;
		if(indexAnimation2 > 11){
			tc_disable(&TCC1);
			profile.animation++;
			Save_profile(&profile);
		}
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
			profile.animation++;
			Save_profile(&profile);
		}
		break;
		
		case 4://Increase all the LEDs from 0 to STARTUP_LIGHT_INTENSITY one after the other, slowed down
		
		tc_write_period(&TCC1, 1024);
		leds[indexAnimation2]++;
		indexAnimation2++;
		
		if (indexAnimation2 >11)
		indexAnimation2 = 0;
		
		if(leds[0] >= StartupLight ){
			tc_disable(&TCC1);
			profile.animation=1;
			Save_profile(&profile);
		}
		break;
		
		case 5:
			decreaseAllOneIncrement(leds);
			if(leds[0] == 10 )
			{
				tc_disable(&TCC1);
				profile.animation = 1;
			}
		
		default:
		
		break;
	}
	
}

int main (void)
{
	uint8_t   oldSlider, vbat_status, ClockMode,lsbMsbTmp;
	uint32_t time_now, time_old;
	struct calendar_date *now;
	status_code_t status;
	
	GO_TO_BOOTLOADER = 0;
	PORTC_OUTCLR = 0xFF;
	PORTB_OUTCLR = 0xFF;
	PORTC_DIR = 0xff;
	PORTB_DIR = 0xff;
	PORTC_OUTCLR = 0xFF;
	PORTB_OUTCLR = 0xFF;
	PORTD_OUTCLR = PIN0_bm;
	PORTD_DIRSET = PIN0_bm;
	HEATING_OFF
	PORTC_PIN0CTRL |= 0x80; 
	PORTC_PIN1CTRL |= 0x80; 
	PORTC_PIN2CTRL |= 0x80; 
	PORTC_PIN3CTRL |= 0x80; 
	PORTC_PIN4CTRL |= 0x80; 
	PORTC_PIN5CTRL |= 0x80; 
	PORTC_PIN6CTRL |= 0x80; 
	PORTC_PIN7CTRL |= 0x80; 
	
	PORTD_PIN0CTRL |= 0x80;
	PORTD_PIN1CTRL |= 0x80;
	PORTD_PIN2CTRL |= 0x80;
	PORTD_PIN3CTRL |= 0x80;
	PORTD_PIN4CTRL |= 0x80;
	PORTD_PIN5CTRL |= 0x80;
	PORTD_PIN6CTRL |= 0x80;
	PORTD_PIN7CTRL |= 0x80;
	PORTE_DIRCLR = PIN2_bm;
	PORTE_PIN2CTRL = PORT_OPC_PULLUP_gc;
	TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
	
	assignAllToValue(0, leds);
	oldSlider = 0;
	ClockMode = CLOCK_OFF;
	USB_port = 0;
	Counter125ms = 0;
	ADCstartTimer = 0;	
	StartupTimer = 0;
	
			
	board_init();
	pmic_init();
	
	sysclk_init();
	cpu_irq_enable();
	irq_initialize_vectors();
	sleepmgr_init();
	
	
	
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_1KCLK);
	wdt_enable();
	
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
	PORTC_OUTSET = PIN0_bm;
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
	sysclk_enable_peripheral_clock(&TWI_MASTER);
	TWI_MasterInit(&twiMaster, &TWIE, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);
	TWIE.CTRL = 6;

	

	
	Load_profile(&profile);
	Save_profile(&profile);
	wdt_reset();
	AT42QT2160init(profile);
	wdt_reset();
	AT42QT2160readStatusBytes();
	
	wdt_reset();

	indexAnimation2 = 0;
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, PWMcounterCallback);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, profile.pwm_freq);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_HI);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV2_gc);
	
	ui_init();
	udc_start();

	tc_enable(&TCC1);
	tc_set_overflow_interrupt_callback(&TCC1, AnimationTimer);
	tc_set_wgm(&TCC1, TC_WG_NORMAL);
	tc_write_period(&TCC1, 1024);
	tc_set_overflow_interrupt_level(&TCC1, TC_INT_LVL_LO);
	tc_write_clock_source(&TCC1, TC_CLKSEL_DIV256_gc);


	tc_enable(&TCD0);
	tc_set_overflow_interrupt_callback(&TCD0, GeneralPurposeTmrCallback);
	tc_set_wgm(&TCD0, TC_WG_NORMAL);
	tc_write_period(&TCD0, 0xFFFF);
	tc_set_overflow_interrupt_level(&TCD0, TC_INT_LVL_MED);
	tc_write_clock_source(&TCD0, TC_CLKSEL_DIV64_gc);	//125.4ms
	tc_reset(&TCD0);


	
	adc_init(); 
	adc_enable(&MY_ADC);
	
	HEATING_OFF
	
	while(1)
	{

		wdt_reset();
		if((Temperature1 > 85) && (StartupTimer > 30))//Overheating, switch off
		{
			profile.animation = 5;
			tc_write_period(&TCC1, 12000);
			tc_enable(&TCC1);
			StartupTimer = 0;
		}
			
		if( !(PORTE.IN & PIN2_bm))
		{
			slider =  255 - AT42QT2160readSlider();
			
			if(slider < 5)
				slider = 0;
			if(StartupTimer > 10)
				assignAllToValue(slider, leds);

			AT42QT2160readStatusBytes();
			if(Counter125ms > 20)
			Counter125ms = 0;
		}
		
		
		if ((timeCheck >= 3) && (USB_port == 1))
		{
			timeCheck = 0;
			Display_ui(&profile);
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
			if((HeatingTimer2 >= 10) && ((PORTD_IN & PIN0_bm) == PIN0_bm))//turn off after 5 minutes
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
		
		if(GO_TO_BOOTLOADER == 1)
		{
			enter_bootloader();
		}
		if( NewPWMval == 1)
		{
			NewPWMval = 0;
			Save_profile(&profile);
			cli();
			tc_disable(&TCC0);
			tc_enable(&TCC0);
			tc_set_overflow_interrupt_callback(&TCC0, PWMcounterCallback);
			tc_set_wgm(&TCC0, TC_WG_NORMAL);
			tc_write_period(&TCC0, profile.pwm_freq);
			tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_HI);
	
			tc_write_clock_source(&TCC0, TC_CLKSEL_DIV2_gc);
			sei();
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
		//HEATING_OFF
		//assignAllToValue(1, leds);
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


void enter_bootloader(void)
{
	HEATING_OFF
	assignAllToValue(0, leds);
	udc_stop(); /*Required to stop USB interrupts messing you up before the vectors have been moved */
	cli();
	PR_PRGEN = 0; //disable power save
	/* Turn off internal 32kHz for RTC */
	OSC.CTRL &= ~OSC_RC32KEN_bm;

	// Disable all Timers and Interrupts to avoid
	// conflicts with Bootloader
	CLK.RTCCTRL = 0;
	RTC32.CTRL =0;
	RTC32.INTCTRL = 0;
	TCD0.CTRLA = 0;
	TCD0.INTCTRLA = 0;
	TCD1.CTRLA = 0;
	TCD1.INTCTRLA = 0;
	TCC0.CTRLA = 0;
	TCC0.INTCTRLA = 0;
	TCC1.CTRLA = 0;
	TCC1.INTCTRLA = 0;
	TCE0.CTRLA = 0;
	TCE0.INTCTRLA = 0;

	/* Jump to 0x401FC = BOOT_SECTION_START + 0x1FC which is
	 * the stated location of the bootloader entry (AVR1916).
	 * Note the address used is the word address = byte addr/2.
	 * My ASM fu isn't that strong, there are probably nicer
	 * ways to do this with, yennow, defined symbols .. */

	asm ("ldi r30, 0xFE\n"  /* Low byte to ZL */
		  "ldi r31, 0x00\n" /* mid byte to ZH */
		  "ldi r24, 0x02\n" /* high byte to EIND which lives */
		  "out 0x3c, r24\n" /* at addr 0x3c in I/O space */
		  "eijmp":  :: "r24", "r30", "r31");
		
	
}

uint8_t Save_profile(struct saved_data *profile)
{

	nvm_write_char(INT_EEPROM,EEPROM_FLAG_ADDRESS,EEPROM_FLAG);
	
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+0,profile->animation);
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+1,(profile->pwm_freq& 0x00FF));
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+2,((profile->pwm_freq& 0xFF00)>> 8));
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+3,profile->at42qt2160_burst_length);
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+4,profile->at42qt2160_burst_repetition);
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+5,profile->at42qt2160_detect_integrator_NDIL);
	nvm_write_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+6,profile->at42qt2160_negative_threshold);
	return(1);
}

uint8_t Load_profile(struct saved_data *profile)
{
	uint8_t flag,tmpRead;
	uint16_t tmp;
	
	flag = nvm_init(INT_EEPROM);
	if(flag == STATUS_OK)
	{
		nvm_read_char(INT_EEPROM,EEPROM_FLAG_ADDRESS,&flag);
		if(flag == EEPROM_FLAG)
		{
			nvm_read_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+0,&profile->animation);
			nvm_read_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+1,&tmpRead);
			tmp = (uint8_t)tmpRead;
			nvm_read_char(INT_EEPROM,EEPROM_PROFILE_ADDRESS+2,&tmpRead);
			tmp = tmp | (tmpRead << 8);
			if((tmp > 999) || (tmp < 128))
				tmp = PWM_FREQ_DEFAULT;
			profile->pwm_freq = tmp;
			return(1);
		}
	}
	return(0);
}