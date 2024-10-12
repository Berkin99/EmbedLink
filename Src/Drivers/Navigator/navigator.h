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

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <stdint.h>
#include "math3d.h"
#include "navigation.h"

typedef struct {
    const char* Name;
    int8_t      Type;
    int8_t      (*Init)(void);
    int8_t      (*Test)(void);
    void        (*Calibrate)(vec_t Correction);
    int8_t      (*Acquire)(navigation_t* plist, uint8_t n);
    int8_t      (*IsReady)(void);
    void        (*WaitDataReady)(void);
}NAV_Handle_t;

void   navigatorInit(void);
void   navigatorTest(void);
int8_t navigatorIsReady(void);
int8_t navigatorGetIndex(char* name);
int8_t navigatorGetSize(void);
int8_t navigatorAcquire(uint8_t index, navigation_t* plist, uint8_t n);

#endif /* NAVIGATOR_H_ */
