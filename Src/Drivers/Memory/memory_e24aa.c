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

#include <sysdefs.h>
#include "sysconfig.h"
#include "systime.h"
#include "memory_e24aa.h"
#include "uart.h"

#ifdef E24AA_I2C

#include "i2c.h"

#define E24AA_I2C_ADDRESS	(0x50)
#define E24AA_MEMORY_OFFSET (0x10)
#define E24AA_MAX_MEMORY	(8192) /* 8Kbyte memory */

static uint8_t isInit;
static uint8_t devAddr;

int8_t memoryInitE24AA(void){

	if(isInit) return E_OVERWRITE;

	devAddr = E24AA_I2C_ADDRESS;

	isInit = 1;
	return memoryReadE24AA(0, &isInit, 1);

}

int8_t memoryTestE24AA(void){
	uint8_t tmp;
	return memoryReadE24AA(0, &tmp, 1);
}

int8_t memoryReadE24AA(memory_t key, uint8_t* pRxData, int8_t len){

//	serialPrint("[R %ld, %d]", key, len);

	key += E24AA_MEMORY_OFFSET;
	if(key > E24AA_MAX_MEMORY) return E_OVERFLOW;
	int8_t status;
	uint8_t mem[2];
	mem[0] = (uint8_t) (key >> 8);
	mem[1] = (uint8_t)  key;
	i2cTransmit(&E24AA_I2C, devAddr, mem, 2);
	status = i2cReceive(&E24AA_I2C, devAddr, pRxData, len);

//	for (int i = 0; i < len; ++i) {
//		serialPrint("%x:", pRxData[i]);
//	}
//	serialPrint("\n");

	if(status == HAL_OK) return OK;
	return E_CONNECTION;
}

int8_t memoryWriteE24AA(memory_t key, uint8_t* pTxData, int8_t len){
//	serialPrint("[W %ld, %d]", key, len);
//
//	for (int i = 0; i < len; ++i) {
//		serialPrint("%x:", pTxData[i]);
//	}
//	serialPrint("\n");

	key += E24AA_MEMORY_OFFSET;
	if(key > E24AA_MAX_MEMORY) return E_OVERFLOW;
	int8_t status;

	status = i2cMemWrite16(&E24AA_I2C, devAddr, key, pTxData, len);
	delay(10);

	if(status == HAL_OK) return OK;
	return E_CONNECTION;
}

#endif
