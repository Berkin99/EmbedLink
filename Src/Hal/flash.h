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

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>

extern const uint32_t FLASH_BANK1_BASE_ADDR;
extern const uint32_t FLASH_BANK2_BASE_ADDR;
extern const uint32_t FLASH_SECTOR_SIZE_VALUE;
extern const uint32_t FLASH_SECTORS_PER_BANK_VALUE;
extern const uint32_t FLASH_BANK_SIZE_VALUE;
extern const uint32_t FLASH_SIZE_TOTAL_VALUE;
extern const uint32_t FLASH_WRITE_GRANULARITY_VALUE;

typedef struct {
    uint32_t base;
    uint32_t size;
    uint32_t sector_size;
    uint32_t bank_count;
} flash_info_t;

void     flashInit          (void);
int8_t   flashUnlock        (void);
int8_t   flashLock          (void);
int8_t   flashEraseSector   (uint32_t address);
int8_t   flashEraseRange    (uint32_t startAddr, uint32_t length);
int8_t   flashWrite         (uint32_t address, const uint8_t* data, uint32_t length);
int8_t   flashRead          (uint32_t address, uint8_t* data, uint32_t length);
uint32_t flashGetSectorAddr (uint8_t sectorIndex);
void     flashGetInfo       (flash_info_t* info);

#endif /* FLASH_H_ */
