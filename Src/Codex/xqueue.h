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

#ifndef XQUEUE_H_
#define XQUEUE_H_

#include <sysdefs.h>
#include <stdint.h>

#define XQUEUE_INIT_CAPACITY 4

typedef struct{
    size_t item_size;
    size_t capacity;
    size_t length;
    size_t front;
    size_t rear;
    void* array;
} xqueue_t;

xqueue_t xqueueNew(size_t item_size);
int8_t   xqueueEnqueue(xqueue_t* q, void* item);
int8_t   xqueueDequeue(xqueue_t* q, void* out_item);
void*    xqueueFront(xqueue_t* q);
void*    xqueueRear(xqueue_t* q);
int8_t   xqueueIsEmpty(xqueue_t* q);
size_t   xqueueSize(xqueue_t* q);
void     xqueueClear(xqueue_t* q);
void     xqueueFree(xqueue_t* q);

#endif /* XQUEUE_H_ */
 