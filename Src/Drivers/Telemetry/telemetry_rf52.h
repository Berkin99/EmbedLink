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

#ifndef TELEMETRY_RF52_H_
#define TELEMETRY_RF52_H_

#include <stdint.h>
#include "telemetry.h"

#define telemetryNameRF52    "RF52"
#define telemetryTypeRF52    TRX_TRANSCEIVER

int8_t telemetryInitRF52(void);
int8_t telemetryTestRF52(void);
int8_t telemetryReceiveRF52(uint8_t* pRxBuffer, uint16_t length);
int8_t telemetryTransmitRF52(const uint8_t* pTxData, uint16_t length);
int8_t telemetryIsReadyRF52(void);
void   telemetryWaitDataReadyRF52(void);

#endif /* TELEMETRY_RF52_H_ */
