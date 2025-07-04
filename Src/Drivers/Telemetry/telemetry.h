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

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include "system.h"

typedef enum{
	TRX_EMPTY,
	TRX_RECEIVER,
	TRX_TRANSMITTER,
	TRX_TRANSCEIVER
}telemetry_e;

typedef struct{
	const char* Name;
	telemetry_e Type;
	int8_t      (*Init)(void);
	int8_t      (*Test)(void);
	void 	    (*Config)(void);
	int8_t      (*Receive)(uint8_t* pRxData, uint16_t length);
	int8_t      (*Transmit)(const uint8_t* pTxData, uint16_t length);
    int8_t      (*IsReady)(void);
	void        (*WaitDataReady)(void);
}telemetry_t;

void    telemetryInit(void);
void    telemetryTest(void);
int8_t  telemetryIsReady(void);
int8_t  telemetryGet(char* name, telemetry_t** ptelemetry);
uint8_t telemetrySize(void);
int8_t  telemetryReceive (char* name, uint8_t* pRxData, uint16_t length);
int8_t  telemetryTransmit(char* name, const uint8_t* pTxData, uint16_t length);

#endif /* TELEMETRY_H_ */
