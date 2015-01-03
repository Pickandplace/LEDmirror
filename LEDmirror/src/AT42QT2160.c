/*
 * AT42QT2160.c
 *
 * Created: 6/22/2014 7:34:41 PM
 *  Author: Jean
 */ 
#include <asf.h>
#include "AT42QT2160.h"
#include "twi_master_driver.h"
TWI_Master_t twiMaster;

void AT42QT2160init(void)
{
	uint8_t NTHRconf[] = {
		AT42QT2160_ADD_NTHR,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL,
		AT42QT2160_NTHR_VAL
	};
	
	 uint8_t DisableKeys[] = {AT42QT2160_ADD_BURST_LENGTH + 8,0,0,0,0,0,0,0,0};
	 uint8_t SliderKeys[] = {
		AT42QT2160_ADD_BURST_LENGTH,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL,
		AT42QT2160_BURST_LENGTH_VAL
	};
	
	 uint8_t Control[] = {AT42QT2160_ADD_SLIDER_CONTROL,0x18};
	 uint8_t Resolution[] = {AT42QT2160_ADD_SLIDER_OPTIONS,0x00};
	 uint8_t NDIL[] = {AT42QT2160_ADD_DETECT_INTEGRATOR,0x06};
	 uint8_t GPIOS[] = {AT42QT2160_ADD_GPIO_DIRECTION,0x1C};
	 uint8_t Burst[] = {AT42QT2160_ADD_DRIFT_COMP,BURST_REPETITION};
	 uint8_t Awake[] = {AT42QT2160_ADD_KEY_CONTROL,0x04};
	 uint8_t Cal[] = {AT42QT2160_ADD_DRIFT_CALIBRATE,0x01};
	
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, NTHRconf,9);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, DisableKeys, 9);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, SliderKeys, 9);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, Control, 2);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, Resolution, 2);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, NDIL, 2);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, GPIOS, 2);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, Burst, 2);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, Awake, 2);
	while (twiMaster.status != TWIM_STATUS_READY);
	TWI_MasterWrite(&twiMaster, AT42QT2160_ADD, Cal, 2);
	while (twiMaster.status != TWIM_STATUS_READY);

}

uint8_t AT42QT2160readSlider(void)
{

	uint8_t data_slider_received[1];
	data_slider_received[0] = 5;
	
	TWI_MasterWriteRead(&twiMaster, AT42QT2160_ADD, data_slider_received, 1,1);
	while (twiMaster.result != TWIM_RESULT_OK && twiMaster.status != TWIM_STATUS_READY);
	
	return(twiMaster.readData[0]);
}

uint8_t AT42QT2160readStatusBytes(void)
{
	uint8_t data_status_received[1];
	data_status_received[0] = 2;
	
	
	TWI_MasterWriteRead(&twiMaster, AT42QT2160_ADD, data_status_received, 1,5);
	while (twiMaster.result != TWIM_RESULT_OK && twiMaster.status != TWIM_STATUS_READY);
	return(twiMaster.readData[0]);
}