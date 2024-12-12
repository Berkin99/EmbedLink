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

#ifndef STREAM_H_
#define STREAM_H_

#include <stdint.h>
#include "sysdefs.h"

typedef struct{
	uint8_t (*read)(void);
	uint8_t (*peek)(void);
	int8_t  (*available)(void);
	uint32_t timeout;
}stream_t;

size_t streamFind(stream_t* self, char* target, uint16_t targetLen);
size_t streamFindUntil(stream_t* self, char* target, uint16_t targetLen, char* terminator, uint16_t termLen);
size_t streamFindString(stream_t* self, char* target, uint16_t targetLen);
size_t streamRead(stream_t* self, uint8_t* buffer, uint16_t len);
size_t streamReadUntil(stream_t* self, uint8_t* buffer, uint16_t len, char* terminator, uint16_t termLen);
size_t streamReadString(stream_t* self, uint8_t* buffer, uint16_t len);

#endif /* STREAM_H_ */
