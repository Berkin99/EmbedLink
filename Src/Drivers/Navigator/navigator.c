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

#include <string.h>
#include "systime.h"
#include "sysconfig.h"
#include "uart.h"
#include "navigator.h"

#ifdef NEOM8N_UART
#include "navigator_neom8n.h"
#endif
#ifdef ZEDF9P_UART
#include "navigator_zedf9p.h"
#endif

#define NAVIGATOR_ADD(NAME) {\
	.Name = navigatorName##NAME, \
	.Init = &navigatorInit##NAME,\
	.Test = &navigatorTest##NAME,\
	.Calibrate = &navigatorCalibrate##NAME,\
	.Acquire = &navigatorAcquire##NAME,\
	.IsReady = &navigatorIsReady##NAME,\
	.WaitDataReady = &navigatorWaitDataReady##NAME,\
},

static const navigator_t navList[] ={
	#ifdef NEOM8N_UART
		NAVIGATOR_ADD(NEOM8N)
	#endif
	#ifdef ZEDF9P_UART
		NAVIGATOR_ADD(ZEDF9P)
	#endif
};

static const uint8_t navLen = sizeof(navList)/sizeof(navigator_t);

void navigatorInit(void){
    for(uint8_t i = 0; i < navLen; i++){
        if(navList[i].Init() == OK) serialPrint("[+] Navigator %s init OK\n",navList[i].Name);
        else serialPrint("[-] Navigator %s init ERROR\n",navList[i].Name);
    }
}

void navigatorTest(void){
    for(uint8_t i = 0; i < navLen; i++){
        if(navList[i].Test() == OK) serialPrint("[+] Navigator %s test OK\n",navList[i].Name);
        else serialPrint("[-] Navigator %s test ERROR\n",navList[i].Name);
    }
}

int8_t navigatorIsReady(void){
    for(uint8_t i = 0; i < navLen; i++){
        if(navList[i].IsReady() != TRUE) return FALSE;
    }
    return TRUE;
}

int8_t navigatorGet(char* name, navigator_t** pnavigator){
    for(uint8_t i = 0; i<navLen; i++){
        if(strcmp(navList[i].Name, name) == 0){
            *pnavigator = &navList[i];
            return OK;
        };
    }
    return E_NOT_FOUND;
}

uint8_t navigatorSize(void){
    return navLen;
}
