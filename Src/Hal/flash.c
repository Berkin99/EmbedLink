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

#include "flash.h"
#include "system.h"

#ifdef HAL_FLASH_MODULE_ENABLED

const uint32_t FLASH_BANK1_BASE_ADDR         = FLASH_BANK1_BASE;
const uint32_t FLASH_BANK2_BASE_ADDR         = FLASH_BANK2_BASE;
const uint32_t FLASH_SECTOR_SIZE_VALUE       = FLASH_SECTOR_SIZE;
const uint32_t FLASH_SECTORS_PER_BANK_VALUE  = FLASH_SECTOR_TOTAL;
const uint32_t FLASH_BANK_SIZE_VALUE         = FLASH_BANK_SIZE;
const uint32_t FLASH_SIZE_TOTAL_VALUE        = FLASH_SIZE;
const uint32_t FLASH_WRITE_GRANULARITY_VALUE = (FLASH_NB_32BITWORD_IN_FLASHWORD * 4U);

static inline uint8_t flashGetBank(uint32_t address){
    return (address < FLASH_BANK2_BASE_ADDR) ? FLASH_BANK_1 : FLASH_BANK_2;
}

void flashInit(void){
}

int8_t flashUnlock(void){
    if (HAL_FLASH_Unlock() != HAL_OK) return E_ERROR;
    return OK;
}

int8_t flashLock(void){
    if (HAL_FLASH_Lock() != HAL_OK) return E_ERROR;
    return OK;
}

int8_t flashEraseSector(uint32_t address){
    FLASH_EraseInitTypeDef eraseCfg;
    uint32_t sectorError = 0;

    uint32_t offset = address % FLASH_SECTOR_SIZE_VALUE;
    uint32_t base   = address - offset;

    eraseCfg.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseCfg.Banks        = flashGetBank(address);
    eraseCfg.Sector       = (base - ((eraseCfg.Banks == FLASH_BANK_1) ? FLASH_BANK1_BASE_ADDR : FLASH_BANK2_BASE_ADDR)) / FLASH_SECTOR_SIZE_VALUE;
    eraseCfg.NbSectors    = 1;
    eraseCfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&eraseCfg, &sectorError) != HAL_OK) return E_ERROR;
    return OK;
}

int8_t flashEraseRange(uint32_t startAddr, uint32_t length){
    uint32_t endAddr = startAddr + length;
    for (uint32_t addr = startAddr; addr < endAddr; addr += FLASH_SECTOR_SIZE_VALUE) {
        if (flashEraseSector(addr) != OK) return E_ERROR;
    }
    return OK;
}

int8_t flashWrite(uint32_t address, const uint8_t* data, uint32_t length){
    if (address % FLASH_WRITE_GRANULARITY_VALUE) return E_ERROR;
    if ((address + length) > (FLASH_BANK2_BASE_ADDR + FLASH_BANK_SIZE_VALUE)) return E_OVERFLOW;

    if (flashUnlock() != OK) return E_ERROR;

    uint32_t remaining = length;
    uint8_t* pData     = (uint8_t*)data;

    while (remaining) {
        uint32_t chunk = (remaining >= FLASH_WRITE_GRANULARITY_VALUE) ? FLASH_WRITE_GRANULARITY_VALUE : remaining;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)pData) != HAL_OK) {
            flashLock();
            return E_ERROR;
        }

        address   += FLASH_WRITE_GRANULARITY_VALUE;
        pData     += FLASH_WRITE_GRANULARITY_VALUE;
        remaining -= chunk;
    }

    flashLock();
    SCB_CleanDCache();
    __DSB();
    __ISB();

    return OK;
}

int8_t flashRead(uint32_t address, uint8_t* data, uint32_t length){
    if ((address + length) > (FLASH_BANK2_BASE_ADDR + FLASH_BANK_SIZE_VALUE)) return E_OVERFLOW;

    const uint8_t* src = (const uint8_t*)address;
    for (uint32_t i = 0; i < length; i++) {
        data[i] = src[i];
    }

    return OK;
}

uint32_t flashGetSectorAddr(uint8_t sectorIndex){
    if (sectorIndex < FLASH_SECTORS_PER_BANK_VALUE) {
        return FLASH_BANK1_BASE_ADDR + (sectorIndex * FLASH_SECTOR_SIZE_VALUE);
    } else {
        return FLASH_BANK2_BASE_ADDR + ((sectorIndex - FLASH_SECTORS_PER_BANK_VALUE) * FLASH_SECTOR_SIZE_VALUE);
    }
}

void flashGetInfo(flash_info_t* info){
    if (!info) return;
    info->base        = FLASH_BANK1_BASE_ADDR;
    info->size        = FLASH_SIZE_TOTAL_VALUE;
    info->sector_size = FLASH_SECTOR_SIZE_VALUE;
    info->bank_count  = 2;
}

#endif
