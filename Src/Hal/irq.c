
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

#include "irq.h"

#include "system.h"
#include "sysconfig.h"

irq_t irq1;

void irqInit(){
    #ifdef HIRQ1
    irq1.idx = HIRQ1;
    irq1.signal = semaphoreCreate();
    #endif
}

int8_t irqWait(irq_t* irq, uint32_t timeout){
    return semaphoreTake(irq->signal, timeout);
}

irq_t* HAL_GPIO_EXTI_Parent(uint16_t pin){
	#ifdef HIRQ1
	if(pin == irq1.idx) return &irq1;
	#endif
	return NULL;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin){
    irq_t* parent = HAL_GPIO_EXTI_Parent(pin);
    if(parent == NULL) return;
    semaphoreGiveISR(parent->signal);
}

