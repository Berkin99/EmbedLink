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

#ifndef NRX_LOGIC_H_
#define NRX_LOGIC_H_

#include <stdbool.h>
#include <stdint.h>
#include "nrx.h"
#include "ntrp.h"

#define NRX_VARID_IS_VALID(varId) (varId.id != 0xffffu)

typedef struct nrxVarId_s {
  uint16_t id;
  uint16_t index;
} __attribute__((packed)) nrxVarId_t;

struct nrx_s* nrxGetVar(uint16_t index);
nrxVarId_t nrxGetVarId(const char* group, const char* name);

int nrxGetType(uint16_t index);

uint8_t nrxVarSize(int type);
uint8_t nrxGroupSize(int index);

void nrxLogicInit();

#endif /* nrx_LOGIC_H_ */
