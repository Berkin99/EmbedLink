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

#include "spi.h"
#include "system.h"
#include "sysconfig.h"
#include "gpio.h"

#ifdef HAL_SPI_MODULE_ENABLED

#define SPI_TIMEOUT (1000)

struct spi_s {
    SPI_HandleTypeDef *handle;
};

spi_t spi1;
spi_t spi2;
spi_t spi3;

void spiInit(void)
{
#ifdef HSPI1
    spi1.handle = &HSPI1;
#endif
#ifdef HSPI2
    spi2.handle = &HSPI2;
#endif
#ifdef HSPI3
    spi3.handle = &HSPI3;
#endif
#ifdef SPI_CS_HIGH
    pin_t spis[] = SPI_CS_HIGH;
    for (int i = 0; i < (sizeof(spis) / sizeof(pin_t)); i++) {
        pinWrite(spis[i], HIGH);
    }
#endif
}

void spiBeginTransaction(spi_t* spi){
    (void)spi;
}

void spiEndTransaction(spi_t* spi){
    (void)spi;
}

int8_t spiReceive(spi_t* spi, uint8_t* pRxData, uint16_t len){
    if (HAL_SPI_Receive(spi->handle, pRxData, len, SPI_TIMEOUT) != HAL_OK) return E_CONNECTION;
    return OK;
}

int8_t spiTransmit(spi_t* spi, const uint8_t* pTxData, uint16_t len){
    if (HAL_SPI_Transmit(spi->handle, (uint8_t*)pTxData, len, SPI_TIMEOUT) != HAL_OK) return E_CONNECTION;
    return OK;
}

int8_t spiTransmitReceive(spi_t* spi, uint8_t* pRxData, const uint8_t* pTxData, uint16_t len){
    if (HAL_SPI_TransmitReceive(spi->handle, (uint8_t*)pTxData, pRxData, len, SPI_TIMEOUT) != HAL_OK) return E_CONNECTION;
    return OK;
}

#endif
