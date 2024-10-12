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

#ifndef NAVIGATOR_NEOM8N_H_
#define NAVIGATOR_NEOM8N_H_

#include <stdint.h>
#include <stdbool.h>
#include "navigator.h"

#define navigatorNameNEOM8N    "NEOM8N"
#define navigatorFreqNEOM8N    (20)    /*Hz*/

int8_t navigatorInitNEOM8N(void);
int8_t navigatorTestNEOM8N(void);
void   navigatorCalibrateNEOM8N(vec_t Correction);
int8_t navigatorIsCalibratedNEOM8N(void);
int8_t navigatorAcquireNEOM8N(navigation_t* plist, uint8_t n);
int8_t navigatorIsReadyNEOM8N(void);
void   navigatorWaitDataReadyNEOM8N(void);

#endif /* NAVIGATOR_NEOM8N_H_ */
