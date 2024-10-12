/**
 *    __  __ ____ _  __ ____ ___ __  __
 *    \ \/ // __// |/ //  _// _ |\ \/ /
 *     \  // _/ /    /_/ / / __ | \  /
 *     /_//___//_/|_//___//_/ |_| /_/
 *
 *         Yeniay System Firmware
 *
 *       Copyright (C) 2024 Yeniay
 *
 * This  program  is  free software:   you
 * can  redistribute it  and/or  modify it
 * under  the  terms of  the  GNU  General
 * Public  License as  published  by   the
 * Free Software Foundation, in version 3.
 *
 * You  should  have  received  a  copy of
 * the  GNU  General  Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef MEMORY_E24AA_H_
#define MEMORY_E24AA_H_

#include <stdint.h>
#include "memory.h"

#define memoryNameE24AA	"E24AA"

int8_t memoryInitE24AA(void);
int8_t memoryTestE24AA(void);
int8_t memoryReadE24AA(memory_t key, uint8_t* pRxData, int8_t len);
int8_t memoryWriteE24AA(memory_t key, uint8_t* pTxData, int8_t len);

#endif /* MEMORY_E24AA_H_ */

