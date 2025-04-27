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

#include <stdlib.h>
#include "xevent.h"

xeventHandle_t xeventNew(void){
    xeventHandle_t handle;
    handle.arr_capacity = XEVENT_INIT_CAPACITY;
    handle.arr_size = 0;
    handle.arr = malloc(handle.arr_capacity * sizeof(xevent_t));
    if(handle.arr != NULL){
        handle.state = XEVENT_READY;
        return handle;
    }
    handle.state = XEVENT_ERROR;
    return handle;
}

void xeventCall(xeventHandle_t* handle){
    if(handle->state != XEVENT_READY) return;

    for (uint8_t i = 0; i < handle->arr_size; i++) {
        xevent_t temp = handle->arr[i];
        temp();
    }
}

void xeventAdd(xeventHandle_t* handle, xevent_t xevent){
    if (handle->state != XEVENT_READY) {
        *handle = xeventNew();
        if (handle->state != XEVENT_READY) return;
    }

    if(handle->arr_size == XEVENT_MAX_CAPACITY) return;
    handle->arr_size++;
    if (handle->arr_size > handle->arr_capacity) {

        handle->arr_capacity *= 2;
        if((handle->arr_capacity)>XEVENT_MAX_CAPACITY) handle->arr_capacity = XEVENT_MAX_CAPACITY;
        handle->arr = realloc(handle->arr,handle->arr_capacity * sizeof(xevent_t));
    }

    if(handle->arr_size>handle->arr_capacity) return;
    handle->arr[handle->arr_size -1] = xevent;
}

void xeventRemove(xeventHandle_t* handle){
    if(handle->state != XEVENT_READY) return;

    if (!(handle->arr_size > 0)) return;
    handle->arr[handle->arr_size-1] = NULL;
    handle->arr_size--;
}

void xeventFree(xeventHandle_t* handle){
    if(handle->state != XEVENT_READY) return;
    free(handle->arr);
}
