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
#include "systime.h"
#include "sysconfig.h"

#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "uart.h"

void systemTask(void* argv);
static uint8_t sysInit;

void systemLaunch(void){
    /* Init Code */
    if(sysInit) return;
    sysInit = 1;

    // Initialize peripherals
    //i2cInit();
    //spiInit();
    //flashInit();
    uartInit();
    
    systemTask(NULL);    
}

void systemTask(void* argv){
    while(1){
        /* Loop Code */
    }
}

void systemWaitReady(void){

}

void systemErrorCall(void){
    while(1);
}
