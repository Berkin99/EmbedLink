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
#include "i2c.h"

#ifdef HAL_I2C_MODULE_ENABLED

#define I2C_TIMEOUT (200)

struct i2c_s {
    I2C_HandleTypeDef *handle;
};

i2c_t i2c1;
i2c_t i2c2;
i2c_t i2c3;

void i2cInit(void){
#ifdef HI2C1
    i2c1.handle = &HI2C1;
#endif
#ifdef HI2C2
    i2c2.handle = &HI2C2;
#endif
#ifdef HI2C3
    i2c3.handle = &HI2C3;
#endif
}

int8_t i2cReceive(i2c_t* i2c, uint8_t devAddr, uint8_t* pRxData, uint16_t len){
    if (HAL_I2C_Master_Receive(
            i2c->handle,
            devAddr << 1,
            pRxData,
            len,
            I2C_TIMEOUT) != HAL_OK) return E_CONNECTION;

    return OK;
}

int8_t i2cTransmit(i2c_t* i2c, uint8_t devAddr, uint8_t* pTxData, uint16_t len){
    if (HAL_I2C_Master_Transmit(
            i2c->handle,
            devAddr << 1,
            pTxData,
            len,
            I2C_TIMEOUT) != HAL_OK) return E_CONNECTION;

    return OK;
}

int8_t i2cMemRead(i2c_t* i2c, uint8_t devAddr, uint8_t memAddr, uint8_t* pRxData, uint16_t len){
    if (HAL_I2C_Mem_Read(
            i2c->handle,
            devAddr << 1,
            memAddr,
            I2C_MEMADD_SIZE_8BIT,
            pRxData,
            len,
            I2C_TIMEOUT) != HAL_OK) return E_CONNECTION;

    return OK;
}

int8_t i2cMemWrite(i2c_t* i2c, uint8_t devAddr, uint8_t memAddr, uint8_t* pTxData, uint16_t len){
    if (HAL_I2C_Mem_Write(
            i2c->handle,
            devAddr << 1,
            memAddr,
            I2C_MEMADD_SIZE_8BIT,
            pTxData,
            len,
            I2C_TIMEOUT) != HAL_OK) return E_CONNECTION;

    return OK;
}

int8_t i2cMemRead16(i2c_t* i2c, uint8_t devAddr, uint16_t memAddr, uint8_t* pRxData, uint16_t len){
    if (HAL_I2C_Mem_Read(
            i2c->handle,
            devAddr << 1,
            memAddr,
            I2C_MEMADD_SIZE_16BIT,
            pRxData,
            len,
            I2C_TIMEOUT) != HAL_OK) return E_CONNECTION;

    return OK;
}

int8_t i2cMemWrite16(i2c_t* i2c, uint8_t devAddr, uint16_t memAddr, uint8_t* pTxData, uint16_t len){
    if (HAL_I2C_Mem_Write(
            i2c->handle,
            devAddr << 1,
            memAddr,
            I2C_MEMADD_SIZE_16BIT,
            pTxData,
            len,
            I2C_TIMEOUT) != HAL_OK) return E_CONNECTION;

    return OK;
}

#endif
