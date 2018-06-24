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

/** INCLUDES *******************************************************/
#include "system.h"

#include "app_device_cdc_to_uart.h"


#include "usb.h"
#include "usb_device.h"
#include "usb_device_cdc.h"


LEM_state_t LEM_state;
uint8_t button_pushed;
uint16_t adc;
/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
MAIN_RETURN main(void)
{
    button_pushed = 0;
    
    //SYSTEM_Initialize(SYSTEM_STATE_USB_START);
#ifdef USB_LINK
    FastClock(&LEM_state);
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);
    USBDeviceInit();
    USBDeviceAttach();
#else
    SlowClock(&LEM_state);
    SYSTEM_Initialize(SYSTEM_STATE_NO_USB);  
    LEM_state.state = LEM_SLEEP;
#endif

    
    while(1)
    {
        
#ifdef USB_LINK
        SYSTEM_Tasks();
        
       
        
        /* If the USB device isn't configured yet, we can't really do anything
         * else since we don't have a host to talk to.  So jump back to the
         * top of the while loop. */
        if( USBGetDeviceState() < CONFIGURED_STATE )
        {
            /* Jump back to the top of the while loop. */
            continue;
        }

        /* If we are currently suspended, then we need to see if we need to
         * issue a remote wakeup.  In either case, we shouldn't process any
         * keyboard commands since we aren't currently communicating to the host
         * thus just continue back to the start of the while loop. */
        if( USBIsDeviceSuspended()== true )
        {
            /* Jump back to the top of the while loop. */
            continue;
        }
        if( USBGetDeviceState() == CONFIGURED_STATE )
        {
        }

        if(button_pushed == 1)
        {
            service_button(&LEM_state);
            button_pushed = 0;
        }
        APP_DeviceCDCEmulatorTasks();
#else
        if(A2DP_LINK == 1)
        {
            if(LEM_state.state != LEM_PLAY)
            {
                LEM_state.state = LEM_PLAY;
                EN_POT
                Configure_ADC(1);
                Configure_CCP2(1);
                CCPR2H = (((COMPARE_REG_VAL>>5)>>8) & 0xFF);
                CCPR2L = ((COMPARE_REG_VAL>>5) & 0xFF);
            }
        }
        else
        {
            if(LEM_state.state == LEM_PLAY)
               LEM_state.state = LEM_ON;     
            OSCCONbits.IDLEN = 1;
            SLEEP();
        }
        if(LEM_state.state == LEM_SLEEP)
            HPowerOff(&LEM_state);
        OSCCONbits.IDLEN = 1;
        SLEEP();
#endif


    }//end while
}//end main




/*******************************************************************************
 End of File
*/
