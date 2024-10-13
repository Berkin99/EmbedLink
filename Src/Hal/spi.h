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

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "rtos.h"

typedef struct spi_s{
	void*       handle;
	mutex_t     mutex;
	semaphore_t rxCplt;
	semaphore_t txCplt;
}spi_t;

extern spi_t spi1;
extern spi_t spi2;
extern spi_t spi3;

void   spiInit             (void);
void   spiBeginTransaction (spi_t* spi);
void   spiEndTransaction   (spi_t* spi);
int8_t spiReceive          (spi_t* spi, uint8_t* pRxData, uint16_t len);
int8_t spiTransmit         (spi_t* spi, uint8_t* pTxData, uint16_t len);
int8_t spiTransmitReceive  (spi_t* spi, uint8_t* pRxData, uint8_t* pTxData, uint16_t len);

#endif /* SPI_H_ */
