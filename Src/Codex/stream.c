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

#include <string.h>
#include "stream.h"

size_t streamFind(stream_t* self, char* target, uint16_t targetLen) {
    if (self == NULL || self->available == NULL || self->read == NULL) {
        return E_NULL_PTR;
    }

    uint16_t targetIndex = 0;
    size_t   count       = 0;

    while (1) {
        if (!self->available()) return 0;
        char c = (char)self->read();
        count++;

        if(c == target[targetIndex]){
        	targetIndex++;
        	if(targetIndex == targetLen) break;
        }
        else targetIndex = 0;
    }

    return count;
}

size_t streamFindUntil(stream_t* self, char* target, uint16_t targetLen, char* terminator, uint16_t termLen) {
    if (self == NULL || target == NULL || terminator == NULL || self->available == NULL || self->read == NULL ) {
        return E_NULL_PTR;
    }

	if (targetLen == 0) return 0;

    uint16_t targetIndex = 0;
    uint16_t termIndex   = 0;
    size_t   count       = 0;

    while (1) {
        if (!self->available()) return 0;
        char c = (char)self->read();
        count++;

        if(c == target[targetIndex]){
        	targetIndex++;
        	if(targetIndex == targetLen) break;
        }
        else targetIndex = 0;

        if(termLen > 0){
            if(c == terminator[termIndex]){
            	termIndex++;
            	if(termIndex == termLen) return 0;
            }
            else termIndex = 0;
        }
    }

    return count;
}

size_t streamFindString(stream_t* self, char* target, uint16_t targetLen) {
    return streamFindUntil(self, target, targetLen, "", 0);
}

size_t streamRead(stream_t* self, uint8_t* buffer, uint16_t len) {
    if (self == NULL || buffer == NULL || self->read == NULL || self->available == NULL) {
        return E_NULL_PTR;
    }

    size_t count = 0;
    while (count < len && self->available()) {
		buffer[count] = self->read();
		count++;
    }

    return count;
}

size_t streamReadUntil(stream_t* self, uint8_t* buffer, uint16_t len, char* terminator, uint16_t termLen){
    if (self == NULL || buffer == NULL || self->read == NULL || self->available == NULL) {
        return E_NULL_PTR;
    }

    uint16_t termIndex   = 0;
    size_t   count       = 0;

    while (count < len) {
        if (!self->available()) return 0;
        char c = (char)self->read();
        buffer[count] = c;
        count++;

        if(termLen > 0){
            if(c == terminator[termIndex]){
            	termIndex++;
            	if(termIndex == termLen) break;
            }
            else termIndex = 0;
        }
    }

    return count;

}

size_t streamReadString(stream_t* self, uint8_t* buffer, uint16_t len) {
	return streamReadUntil(self, buffer, len, '\0', 1);
}
