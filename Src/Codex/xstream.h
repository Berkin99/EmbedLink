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

#ifndef XSTREAM_H_
#define XSTREAM_H_

#include <stdint.h>
#include <sysdefs.h>

typedef struct{
    uint8_t (*read)(void);      /* Read function */
    uint8_t (*peek)(void);      /* Peek function */
    int8_t  (*available)(void); /* Available function */
    uint32_t timeout;
}xstream_t;

size_t xstreamFind(xstream_t* self, char* target, uint16_t targetLen);
size_t xstreamFindUntil(xstream_t* self, char* target, uint16_t targetLen, char* terminator, uint16_t termLen);
size_t xstreamFindString(xstream_t* self, char* target, uint16_t targetLen);
size_t xstreamRead(xstream_t* self, uint8_t* buffer, uint16_t len);
size_t xstreamReadUntil(xstream_t* self, uint8_t* buffer, uint16_t len, char* terminator, uint16_t termLen);
size_t xstreamReadString(xstream_t* self, uint8_t* buffer, uint16_t len);

#endif /* XSTREAM_H_ */
