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


#ifndef NAVIGATOR_ZEDF9P_H_
#define NAVIGATOR_ZEDF9P_H_

#include <stdint.h>
#include "navigator.h"

#define navigatorNameZEDF9P    "ZEDF9P"
#define navigatorFreqZEDF9P    (5)    /*Hz*/

int8_t navigatorInitZEDF9P(void);
int8_t navigatorTestZEDF9P(void);
void   navigatorCalibrateZEDF9P(vec_t Correction);
int8_t navigatorIsCalibratedZEDF9P(void);
int8_t navigatorAcquireZEDF9P(navigation_t* plist, uint8_t n);
int8_t navigatorIsReadyZEDF9P(void);
void   navigatorWaitDataReadyZEDF9P(void);

#endif /* NAVIGATOR_ZEDF9P_H_ */
