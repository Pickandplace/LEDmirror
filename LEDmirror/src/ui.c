/*
 * ui.c
 *
 * Created: 1/2/2015 3:15:54 PM
 * Under the CC BY-NC-SA license
 * Jean Wlodarski
 * Pickandplace.wordpress.com
 */ 
#include <asf.h>
#include "ui.h"
#include "PWM_functions.h"
#include <string.h>
void Display_ui(struct saved_data *profile)
{
	uint8_t i,MenuPos;
	char rx_char;
	char WELCOME_STRING[] = "\r\n\r\nMenu:\r\n1. Set intensity\r\n2. Set date\r\n3. Set time\r\n4. Turn heating ON\r\n5. Turn heating OFF\r\n6. Set PWM freq\r\n7. Set 42QT2160 Negative Threshold\r\n8. Set 42QT2160 Burst Length\r\n9. Set 42QT2160 Burst Repetition\r\na. Set 42QT2160 NDIL\r\n0. Launch Bootloader\r\n";
	char MENU1_STRING[] = "Intensity (0..255) :";
	char MENU2_STRING[] = "DD/MM/YY :";
	char MENU3_STRING[] = "hh:mm :";
	char MENU6_STRING[] = "Frequency (128..999) :";
	char MENU7_STRING[] = "Negative Threshold (7..12) :";
	char MENU8_STRING[] = "Burst Length (8..32) :";
	char MENU9_STRING[] = "Burst Repetition (1..63) :";
	char MENUa_STRING[] = "NDIL (1..31) :";
	char SLIDER_STRING[] = "\r\nSlider position: ";
	char THERM1_STRING[] = "\r\nThermistor1: ";
	char TEMP1_STRING[] = "\r\nTemperature1: ";
	char PWM_FREQ_STRING[] = "\r\nPWM Freq: ";
	char HEATING_ON_STRING[] = "\r\nHeating ON ";
	char HEATING_OFF_STRING[] = "\r\nHeating OFF ";
	char AT42_NEG_TH[] = "\r\n AT42QT26 Negative Threshold: ";
	char AT42_BURST_LEN[] = "\r\n AT42QT26 Burst Length: ";
	char AT42_BURST_REP[] = "\r\n AT42QT26 Burst Repetition: ";
	char AT42_NDIL[] = "\r\n AT42QT26 NDIL: ";
	//char MENU4_STRING[] = "Display date and time";
	char rx_buffer[10];
	struct calendar_date  *timeDate ;
	uint32_t timeDateuint,time;
	
	timeDate = malloc(sizeof(struct calendar_date));
	//calendar_timestamp_to_date( time_now , now);
	
	time = rtc_get_time();
	
		udi_cdc_putc('\r');
		udi_cdc_putc('\n');
		udi_cdc_putc('\r');
		udi_cdc_putc('\n');
		udi_cdc_putc('\r');
		udi_cdc_putc('\n');
		displayTime(time);
		udi_cdc_write_buf(SLIDER_STRING,sizeof(SLIDER_STRING));
		itoa((int)profile->animation,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));
		
		udi_cdc_write_buf(THERM1_STRING,sizeof(THERM1_STRING));
		itoa(Thermistor1,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));
		
		udi_cdc_write_buf(TEMP1_STRING,sizeof(TEMP1_STRING));
		itoa(Temperature1,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));
		
		udi_cdc_write_buf(PWM_FREQ_STRING,sizeof(PWM_FREQ_STRING));
		itoa(profile->pwm_freq,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));
		
		udi_cdc_write_buf(AT42_NEG_TH,sizeof(AT42_NEG_TH));
		itoa(profile->at42qt2160_negative_threshold,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));	

		udi_cdc_write_buf(AT42_BURST_REP,sizeof(AT42_BURST_REP));
		itoa(profile->at42qt2160_burst_repetition,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));

		udi_cdc_write_buf(AT42_BURST_LEN,sizeof(AT42_BURST_LEN));
		itoa(profile->at42qt2160_burst_length,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));
		
		udi_cdc_write_buf(AT42_NDIL,sizeof(AT42_NDIL));
		itoa(profile->at42qt2160_detect_integrator_NDIL,rx_buffer,10);
		udi_cdc_write_buf(rx_buffer,strlen(rx_buffer));			
		
		if ((PORTD_IN & PIN0_bm) == PIN0_bm)
		{
			udi_cdc_write_buf(HEATING_ON_STRING,sizeof(HEATING_ON_STRING));
		}
		else
		{
			udi_cdc_write_buf(HEATING_OFF_STRING,sizeof(HEATING_OFF_STRING));
		}
		
		udi_cdc_write_buf(WELCOME_STRING,sizeof(WELCOME_STRING));
	wdt_disable();
	if(udi_cdc_is_rx_ready()==1)
	{
		rx_char = udi_cdc_getc();
		NewPWMval = 0;	
		switch (rx_char)
		{
			case 13:
			//if(MenuPos == 0)
			//udi_cdc_putc(12);
			udi_cdc_putc('\r');
			udi_cdc_putc('\n');
			udi_cdc_write_buf(WELCOME_STRING,sizeof(WELCOME_STRING));
			//MenuPos = 1;

			break;
			case '1':
			udi_cdc_write_buf(MENU1_STRING,sizeof(MENU1_STRING));
			i = 0;
			rx_buffer[0] = 48;
			while(rx_char != 13)
			{
				if(udi_cdc_is_rx_ready()==1)
				{
					rx_char = udi_cdc_getc();
					udi_cdc_putc(rx_char);
					rx_buffer[i] = rx_char;
					i++;
					if (i>5)
					break;
				}
			}
			udi_cdc_putc(10);
			udi_cdc_putc(13);
			assignAllToValue((uint8_t)numberFromAscii(rx_buffer), leds);
			udi_cdc_putc(10);
			udi_cdc_putc(13);
			MenuPos = 0;
			break;
				
			case '2':
			udi_cdc_write_buf(MENU2_STRING,sizeof(MENU2_STRING));
			i = 0;
			while(rx_char != 13) 
			{
				if(udi_cdc_is_rx_ready()==1)
				{
					rx_char = udi_cdc_getc();
					rx_buffer[i] = rx_char;
					i++;
					udi_cdc_putc(rx_char);
				}
			}
			if(i == 9)
			{
				timeDateuint = rtc_get_time();
				calendar_timestamp_to_date(timeDateuint, timeDate);
				timeDate->date = (rx_buffer[0] - 48)*10 + (rx_buffer[1] - 48) - 1; //Atmel days start at 0
				timeDate->month = (rx_buffer[3] - 48)*10 + (rx_buffer[4] - 48) - 1; //Atmel month starts at 0
				timeDate->year = (rx_buffer[6] - 48)*10 + (rx_buffer[7] - 48)  + 2000; 
				timeDateuint = calendar_date_to_timestamp(timeDate);
				rtc_init();
				rtc_set_time(timeDateuint);
			}
			MenuPos = 0;
			break;
				
			case '3':
			udi_cdc_write_buf(MENU3_STRING,sizeof(MENU3_STRING));
			i = 0;
			while(rx_char != 13)
			{
				if(udi_cdc_is_rx_ready()==1)
				{
					rx_char = udi_cdc_getc();
					rx_buffer[i] = rx_char;
					i++;
					udi_cdc_putc(rx_char);
				}
					
			}
			if(i == 6)
			{
				timeDateuint = rtc_get_time();
				calendar_timestamp_to_date(timeDateuint, timeDate);
				timeDate->hour = (rx_buffer[0] - 48)*10 + (rx_buffer[1] - 48) ; 
				timeDate->minute = (rx_buffer[3] - 48)*10 + (rx_buffer[4] - 48) ; 
				timeDate->second = 0 ; 
				timeDateuint = calendar_date_to_timestamp(timeDate);
				rtc_init();
				rtc_set_time(timeDateuint);
			}
			MenuPos = 0;
			break;
				
			case '4':
				HEATING_ON
				MenuPos = 0;
			break;
				
			case '5':
				HEATING_OFF
				MenuPos = 0;
			break;
			
			case '6':
				udi_cdc_write_buf(MENU6_STRING,sizeof(MENU6_STRING));
				i = 0;
				rx_buffer[0] = 48;
				while(rx_char != 13)
				{
					if(udi_cdc_is_rx_ready()==1)
					{
						rx_char = udi_cdc_getc();
						udi_cdc_putc(rx_char);
						rx_buffer[i] = rx_char;
						i++;
						if (i>5)
						break;
					}
				}
			
				profile->pwm_freq = numberFromAscii(rx_buffer);
				if(profile->pwm_freq < 1000)
					NewPWMval = 1;
				udi_cdc_putc(10);
				udi_cdc_putc(13);
				MenuPos = 0;
			break;
			
			case '7':
				udi_cdc_write_buf(MENU7_STRING,sizeof(MENU7_STRING));
				i = 0;
				rx_buffer[0] = 48;
				while(rx_char != 13){
					if(udi_cdc_is_rx_ready()==1){
						rx_char = udi_cdc_getc();
						udi_cdc_putc(rx_char);
						rx_buffer[i] = rx_char;
						i++;
						if (i>5)
						break;
					}
				}
				profile->at42qt2160_negative_threshold = numberFromAscii(rx_buffer);
				NewPWMval = 1;
				udi_cdc_putc(10);
				udi_cdc_putc(13);
				MenuPos = 0;
			break;
			
			case '8':
				udi_cdc_write_buf(MENU8_STRING,sizeof(MENU8_STRING));
				i = 0;
				rx_buffer[0] = 48;
				while(rx_char != 13){
					if(udi_cdc_is_rx_ready()==1){
						rx_char = udi_cdc_getc();
						udi_cdc_putc(rx_char);
						rx_buffer[i] = rx_char;
						i++;
						if (i>5)
						break;
					}
				}
				profile->at42qt2160_burst_length = numberFromAscii(rx_buffer);
				NewPWMval = 1;
				udi_cdc_putc(10);
				udi_cdc_putc(13);
				MenuPos = 0;
			break;	
			
			case '9':
				udi_cdc_write_buf(MENU9_STRING,sizeof(MENU9_STRING));
				i = 0;
				rx_buffer[0] = 48;
				while(rx_char != 13){
					if(udi_cdc_is_rx_ready()==1){
						rx_char = udi_cdc_getc();
						udi_cdc_putc(rx_char);
						rx_buffer[i] = rx_char;
						i++;
						if (i>5)
						break;
					}
				}
				profile->at42qt2160_burst_repetition = numberFromAscii(rx_buffer);
				NewPWMval = 1;
				udi_cdc_putc(10);
				udi_cdc_putc(13);
				MenuPos = 0;
			break;

			case 'a':
				udi_cdc_write_buf(MENUa_STRING,sizeof(MENUa_STRING));
				i = 0;
				rx_buffer[0] = 48;
				while(rx_char != 13){
					if(udi_cdc_is_rx_ready()==1){
						rx_char = udi_cdc_getc();
						udi_cdc_putc(rx_char);
						rx_buffer[i] = rx_char;
						i++;
						if (i>5)
						break;
					}
				}
				profile->at42qt2160_detect_integrator_NDIL = numberFromAscii(rx_buffer);
				NewPWMval = 1;
				udi_cdc_putc(10);
				udi_cdc_putc(13);
				MenuPos = 0;
			break;
											
			case '0':
				GO_TO_BOOTLOADER = 1;
				MenuPos =0;
			break;
			
			default:
			udi_cdc_putc(rx_char);
			break;
		}
	}
	if(GO_TO_BOOTLOADER == 0)
	{
		wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_1KCLK);
		wdt_enable();
	}
	
}

void displayTime(uint32_t time_now)
{
	struct calendar_date  *now ;
	char buff[6];
	
	now = malloc(sizeof(struct calendar_date));
	calendar_timestamp_to_date( time_now , now);
	itoa((now->date)+1,buff,10);
	udi_cdc_write_buf(buff,strlen(buff));
	udi_cdc_putc('/');
	itoa((now->month + 1),buff,10);
	udi_cdc_write_buf(buff,strlen(buff));
	udi_cdc_putc('/');
	itoa(now->year,buff,10);
	udi_cdc_write_buf(buff,strlen(buff));
	udi_cdc_putc(' ');
	itoa(now->hour,buff,10);
	if(now->hour < 10)
	{
		buff[1] = buff[0];
		buff[0] = '0';
		buff[2] = 0;
	}
	udi_cdc_write_buf(buff,strlen(buff));
	udi_cdc_putc(':');
	itoa(now->minute,buff,10);
	if(now->minute < 10)
	{
		buff[1] = buff[0];
		buff[0] = '0';
		buff[2] = 0;
	}
	udi_cdc_write_buf(buff,strlen(buff));
	udi_cdc_putc(':');
	itoa(now->second,buff,10);
	if(now->second < 10)
	{
		buff[1] = buff[0];
		buff[0] = '0';
		buff[2] = 0;
	}
	udi_cdc_write_buf(buff,strlen(buff));
	//udi_cdc_putc('\r');
	
}


void Val8bitToASCII(uint8_t value, char*ascii)
{
	if (value < 10)
	{
		ascii[0] = 48;
		ascii[1] = 48;
		ascii[2] = 48+value;
		return;
	}
	
	if (value < 100)
	{
		ascii[0] = 48;
		ascii[1] = 48 + (value / 10);
		ascii[2] = 48+ (value % 10);
		return;
	}
	if (value < 200)
	{
		ascii[0] = 48 + (value / 100);
		ascii[1] = 48 + ( (value - 100) / 10);
		ascii[2] = 48+ ((value - 100) % 10);
		return;
	}
	else
	{
		ascii[0] = 48 + (value / 100);
		ascii[1] = 48 + ( (value - 200) / 10);
		ascii[2] = 48+ ((value - 200) % 10);
		return;
	}
}

volatile uint16_t numberFromAscii(char buffer[])
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		if(buffer[i] == 13)
			b