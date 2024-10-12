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

#ifndef MEMORY_H_
#define MEMORY_H_

#include <stdint.h>
#include <stdbool.h>
#include "mem.h"

#define MEM_TIMEOUT_MS    (100)
#define MEM_INVALID 	  (0xFFFF)
#define MEM_INVALID_ID 	  {0xFFFF, 0xFFFFFFFF}

typedef uint32_t memory_t; /* Unique Key */

typedef struct{
	uint16_t index;
	memory_t key;
}memID_t;

typedef struct {
    const char* Name;
    int8_t      (*Init)(void);
    int8_t      (*Test)(void);
    int8_t      (*MemRead) (memory_t key, uint8_t* pRxData, int8_t len);
    int8_t      (*MemWrite)(memory_t key, uint8_t* pTxData, int8_t len);
}MEM_Handle_t;

void     memoryInit(void);
void     memoryTest(void);
int8_t   memoryRead (memory_t key, uint8_t* pRxData, int8_t len);
int8_t   memoryWrite(memory_t key, uint8_t* pTxData, int8_t len);
int8_t   memoryClear(void);
int8_t   memoryUpload(void);
int8_t   memoryDownload(void);
int8_t   memoryMemUpload  (char* group, char* name);
int8_t   memoryMemDownload(char* group, char* name);
mem_t*   memoryGetVar    (uint16_t index);
memID_t  memoryGetID     (char* group, char* name);
uint32_t memoryGetKey    (uint16_t index);
uint8_t  memoryTypeSize  (uint8_t type);
uint8_t  memoryGroupSize (uint16_t index);
void     memoryLogicInit (void);

#endif /* MEMORY_H_ */
