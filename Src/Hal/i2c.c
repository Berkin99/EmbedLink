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

#include "systime.h"
#include "system.h"
#include "i2c.h"

#define I2C_TIMEOUT	(200)

i2c_t i2c1;
i2c_t i2c2;
i2c_t i2c3;

void i2cInit(void){
	#ifdef HI2C1
	i2c1.handle = &HI2C1;
	i2c1.mutex  = mutexCreate();
	i2c1.cplt   = semaphoreCreate();
	#endif
	#ifdef HI2C2
	i2c2.handle = &HI2C2;
	i2c2.mutex  = mutexCreate();
	i2c2.cplt   = semaphoreCreate();
	#endif
	#ifdef HI2C3
	i2c1.handle = &HI2C3;
	i2c1.mutex  = mutexCreate();
	i2c1.cplt   = semaphoreCreate();
	#endif
}

int8_t i2cReceive(i2c_t* i2c, uint8_t devAddr, uint8_t* pRxData, uint8_t len){
	if(mutexTake(i2c->mutex, I2C_TIMEOUT) != RTOS_OK) return HAL_ERROR;
	int8_t status = HAL_I2C_Master_Receive_IT(i2c->handle, devAddr << 1, pRxData, len);
	if(semaphoreTake(i2c->cplt, I2C_TIMEOUT) != RTOS_OK){
		mutexGive(i2c->mutex);
		return HAL_ERROR;
	}
	mutexGive(i2c->mutex);
	return status;
}

int8_t i2cTransmit(i2c_t* i2c, uint8_t devAddr, uint8_t* pTxData, uint8_t len ){
	if(mutexTake(i2c->mutex, I2C_TIMEOUT) != RTOS_OK) return HAL_ERROR;
	int8_t status = HAL_I2C_Master_Transmit_IT(i2c->handle, devAddr << 1, pTxData, len);
	if(semaphoreTake(i2c->cplt, I2C_TIMEOUT) != RTOS_OK){
		mutexGive(i2c->mutex);
		return HAL_ERROR;
	}
	mutexGive(i2c->mutex);
	return status;
}

int8_t i2cMemRead(i2c_t* i2c, uint8_t devAddr, uint8_t memAddr, uint8_t* pRxData, uint8_t len ){
	if(mutexTake(i2c->mutex, I2C_TIMEOUT) != RTOS_OK) return HAL_ERROR;
	int8_t status = HAL_I2C_Mem_Read_IT(i2c->handle, devAddr << 1, memAddr, I2C_MEMADD_SIZE_8BIT, pRxData, len);
	if(semaphoreTake(i2c->cplt, I2C_TIMEOUT) != RTOS_OK){
		mutexGive(i2c->mutex);
		return HAL_ERROR;
	}
	mutexGive(i2c->mutex);
	return status;
}

int8_t i2cMemWrite(i2c_t* i2c, uint8_t devAddr, uint8_t memAddr, uint8_t* pTxData, uint8_t len ){
	if(mutexTake(i2c->mutex, I2C_TIMEOUT) != RTOS_OK) return HAL_ERROR;
	int8_t status = HAL_I2C_Mem_Write_IT(i2c->handle, devAddr << 1, memAddr, I2C_MEMADD_SIZE_8BIT, pTxData, len);
	if(semaphoreTake(i2c->cplt, I2C_TIMEOUT) != RTOS_OK){
		mutexGive(i2c->mutex);
		return HAL_ERROR;
	}
	mutexGive(i2c->mutex);
	return status;
}

int8_t i2cMemRead16(i2c_t* i2c, uint8_t devAddr, uint16_t  memAddr, uint8_t* pRxData, uint8_t len){
	if(mutexTake(i2c->mutex, I2C_TIMEOUT) != RTOS_OK) return HAL_ERROR;
	int8_t status = HAL_I2C_Mem_Read_IT(i2c->handle, devAddr << 1, memAddr, I2C_MEMADD_SIZE_16BIT, pRxData, len);
	if(semaphoreTake(i2c->cplt, I2C_TIMEOUT) != RTOS_OK){
		mutexGive(i2c->mutex);
		return HAL_ERROR;
	}
	mutexGive(i2c->mutex);
	return status;
}

int8_t i2cMemWrite16(i2c_t* i2c, uint8_t devAddr, uint16_t  memAddr, uint8_t* pTxData, uint8_t len){
	if(mutexTake(i2c->mutex, I2C_TIMEOUT) != RTOS_OK) return HAL_ERROR;
	int8_t status = HAL_I2C_Mem_Write_IT(i2c->handle, devAddr << 1, memAddr, I2C_MEMADD_SIZE_16BIT, pTxData, len);
	if(semaphoreTake(i2c->cplt, I2C_TIMEOUT) != RTOS_OK){
		mutexGive(i2c->mutex);
		return HAL_ERROR;
	}
	mutexGive(i2c->mutex);
	return status;
}

i2c_t* HAL_I2C_Parent(I2C_HandleTypeDef* hi2c){
	#ifdef HI2C1
	if(hi2c == &HI2C1) return &i2c1;
	#endif
	#ifdef HI2C2
	if(hi2c == &HI2C2) return &i2c2;
	#endif
	#ifdef HI2C3
	if(hi2c == &HI2C3) return &i2c3;
	#endif
	return NULL;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c){
	i2c_t* parent = HAL_I2C_Parent(hi2c);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->cplt);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c){
	i2c_t* parent = HAL_I2C_Parent(hi2c);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->cplt);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c){
	i2c_t* parent = HAL_I2C_Parent(hi2c);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->cplt);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c){
	i2c_t* parent = HAL_I2C_Parent(hi2c);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->cplt);
}
