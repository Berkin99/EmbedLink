/*
 *       ______          __             ____    _       __
 *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
 *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
 *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
 *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
 *
 *  EmbedLink Firmware
 *  Copyright (c) 2024 Yeniay RD, All rights reserved.
 *  _________________________________________________________
 *
 *  EmbedLink Firmware is free software: you can redistribute
 *  it and/or  modify it under  the  terms of the  GNU Lesser
 *  General Public License as  published by the Free Software
 *  Foundation,  either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  EmbedLink  Firmware is  distributed  in the  hope that it
 *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with EmbedLink Firmware. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "system.h"
#include "sysconfig.h"
#include "systime.h"
#include "rtos.h"

#ifndef SYSTIME
#error "/SYSTEM> SYSTIME Timer should be defined!"
#endif

#ifdef SYSTIME

uint32_t millis(void){
	return SYSTIME.Instance->CNT/1000;
}

uint32_t micros(void){
	return SYSTIME.Instance->CNT;
}

void delay(uint32_t ms){
#ifdef RTOS_H_
	if(ms == 0) return;
	taskDelay(ms);
#else
	uint32_t dt = millis() + ms;
	while (millis() < dt);
#endif
}

void delayUs(uint32_t us){
	delay(us / 1000);
	uint32_t dt = micros() + (us % 1000);
	while (micros() < dt);
}

#endif
