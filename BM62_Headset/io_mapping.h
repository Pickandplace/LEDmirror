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


#define IND1    PORTDbits.RD7   //P3_0
#define A2DP_LINK    PORTDbits.RD6   //P1_5
#define IND3    PORTDbits.RD5   //P0_4
#define IND4    PORTCbits.RC2   //P0_1
#define IND5    PORTAbits.RA2   //P0_2
#define IND6    PORTAbits.RA1   //P0_3
#define IND7    PORTBbits.RB5   //P0_5
#define IND8    PORTBbits.RB4   //P2_7

#define RST_N       LATAbits.LATA3   
#define LED_RED     LATAbits.LATA4 
#define WAKE_BM62   LATBbits.LATB1   //MFB
#define WAKE_PIC    PORTBbits.RB2   //P0_0
#define N_BTN_VOL_UP	LATBbits.LATB4
#define N_BTN_VOL_DN	LATBbits.LATB5
#define N_BTN_FORWARD	LATCbits.LATC2
#define N_BTN_BACKWARD	LATAbits.LATA1
#define N_BTN_PAIR		LATAbits.LATA2
#define EN_POT LATDbits.LATD1 = 0;
#define DIS_POT LATDbits.LATD1 = 1;


/*
 *					B0	    B1	    B2	    B3	    B4	    B5
 *					MFB		P0_2	P2_7	P0_5	P0_1	P0_3
 * 
 *  Short Press	    B0	    B1	    B2	    B3	    B4	    B5
 *					Toggle	Pairing	V+		V-		Fw		Bck	
 *					Active
 * 
 *  Long Press	    B0	    B1	    B2	    B3	    B4	    B5
 *							Stop	Play	Connect	FFw		Fbck
 */
