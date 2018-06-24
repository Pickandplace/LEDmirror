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

#ifndef BM62_H
#define BM62_H

#include <xc.h>
#include <stdbool.h>


#include "usart.h"
#include "stdint.h"
#include "system.h"

#define LONG_BUTTON_PUSH	1200	//in ms
#define SHORT_BUTTON_PUSH	300	

#define BM62_RX_BUFFER 10
void Send_volume(LEM_state_t *s);

void VolumeUp(LEM_state_t *s);

void VolumeDown(LEM_state_t *s);

void WakeUp(LEM_state_t *s);
void Wakedown(LEM_state_t *s);
void Pair(LEM_state_t *s);

void TogglePause(LEM_state_t *s);

void PlayPlay(LEM_state_t *s);
void bm62Puts(char *c, uint8_t len);
void bm62SendVolume(uint8_t volume);
void PairStop(LEM_state_t *s);
uint8_t calculateChecksum(uint8_t* startByte, uint8_t* endByte);
void bm62AbsVolume(uint8_t volume);
#endif //BM62_H
