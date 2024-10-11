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
#include "spi.h"
#include <rtos.h>

#define SPI_TIMEOUT (100)

spi_t spi1;
spi_t spi2;
spi_t spi3;

void spiInit(void){
	#ifdef HSPI1
	spi1.handle = &HSPI1;
	spi1.mutex  = mutexCreate();
	spi1.rxCplt = semaphoreCreate();
	spi1.txCplt = semaphoreCreate();
	#endif
	#ifdef HSPI2
	spi2.handle = &HSPI2;
	spi2.mutex  = mutexCreate();
	spi2.rxCplt = semaphoreCreate();
	spi2.txCplt = semaphoreCreate();
	#endif
	#ifdef HSPI3
	spi3.handle = &HSPI3;
	spi3.mutex  = mutexCreate();
	spi3.rxCplt = semaphoreCreate();
	spi3.txCplt = semaphoreCreate();
	#endif
}

int8_t spiBeginTransaction(spi_t* spi){
	return mutexTake(spi->mutex, SPI_TIMEOUT);
}

int8_t spiEndTransaction(spi_t* spi){
	return mutexGive(spi->mutex);
}

int8_t spiReceive(spi_t* spi ,uint8_t* pRxData, uint8_t len){
	int8_t status = HAL_SPI_Receive_IT(spi->handle, pRxData, len);
	semaphoreTake(spi->rxCplt, SPI_TIMEOUT);
	return status;
}

int8_t spiTransmit(spi_t* spi ,uint8_t* pTxData, uint8_t len){
	int8_t status = HAL_SPI_Transmit_IT(spi->handle, pTxData, len);
	semaphoreTake(spi->txCplt, SPI_TIMEOUT);
	return status;
}

int8_t spiTransmitReceive(spi_t* spi ,uint8_t* pRxData, uint8_t* pTxData, uint8_t len){
	int8_t status = HAL_SPI_TransmitReceive_IT(spi->handle, pTxData, pRxData, len);
	if(semaphoreTake(spi->txCplt, SPI_TIMEOUT) == pdTRUE) return HAL_OK;
	return status;
}

spi_t* HAL_SPI_Parent(SPI_HandleTypeDef* hspi){
	#ifdef HSPI1
	if(hspi == &HSPI1) return &spi1;
	#endif
	#ifdef HSPI2
	if(hspi == &HSPI2) return &spi2;
	#endif
	#ifdef HSPI3
	if(hspi == &HSPI3) return &spi3;
	#endif
	return NULL;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi){
	spi_t* parent = HAL_SPI_Parent(hspi);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->rxCplt);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi){
	spi_t* parent = HAL_SPI_Parent(hspi);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->txCplt);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
	spi_t* parent = HAL_SPI_Parent(hspi);
	if(parent == NULL) return;
	semaphoreGiveISR(parent->txCplt);
}
