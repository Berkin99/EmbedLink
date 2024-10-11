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

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "rtos.h"

typedef struct i2c_s{
	void*       handle;
	mutex_t     mutex;
	semaphore_t cplt;
}i2c_t;

extern i2c_t i2c1;
extern i2c_t i2c2;
extern i2c_t i2c3;

void   i2cInit       (void);
int8_t i2cReceive    (i2c_t* i2c, uint8_t devAddr, uint8_t* pRxData, uint8_t len);
int8_t i2cTransmit   (i2c_t* i2c, uint8_t devAddr, uint8_t* pTxData, uint8_t len);
int8_t i2cMemRead    (i2c_t* i2c, uint8_t devAddr, uint8_t  memAddr, uint8_t* pRxData, uint8_t len);
int8_t i2cMemWrite   (i2c_t* i2c, uint8_t devAddr, uint8_t  memAddr, uint8_t* pTxData, uint8_t len);
int8_t i2cMemRead16  (i2c_t* i2c, uint8_t devAddr, uint16_t memAddr, uint8_t* pRxData, uint8_t len);
int8_t i2cMemWrite16 (i2c_t* i2c, uint8_t devAddr, uint16_t memAddr, uint8_t* pTxData, uint8_t len);

#endif /* I2C_H_ */
