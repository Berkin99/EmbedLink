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

#ifndef XEVENT_H_
#define XEVENT_H_

#include <stdlib.h>
#include <stdint.h>

#define XEVENT_INIT_CAPACITY         2
#define XEVENT_MAX_CAPACITY          16

typedef enum{
    XEVENT_CLOSE = 0,
    XEVENT_READY = 1,
    XEVENT_ERROR = 2
}xeventState_e;

typedef void (*xevent_t)(void);

typedef struct {
    xevent_t* arr;          /* Events for ready to call */
    uint8_t   arr_size;     /* Number of xevent_t in the array */
    uint8_t   arr_capacity; /* @arr length */
    xeventState_e state;    /* Debugging state of the xeventHandle_t */
}xeventHandle_t;

/*  @brief Creates new xeventHandle. Initial
 *  allocated memory for xevent functions is
 *  described as @XEVENT_INIT_CAPACITY
 *
 *  @return xeventHandle_t constructor
 *  @retval handle.state == XEVENT_ERROR:
 *  memory allocation error!
 *  @retval handle.state == XEVENT_READY:
 *  successful memory allocation.
 */
xeventHandle_t xeventNew(void);

/*  @brief Calls every xevent_t xevent function
 *  in the handle array (@handle->arr).
 *
 *  @param handle: object pointer.
 */
void xeventCall(xeventHandle_t* handle);

/*  @brief Adds xevent_t function in the handle
 *  array. If arr_size is exceeds the capacity
 *  of the xevent_t array, reallocates the memory
 *  as capacity multiplied by 2. If handle is not
 *  initalized (not used the construct function)
 *  @xeventNew(), it constructs the object.
 *
 *  @param handle: object pointer.
 */
void xeventAdd(xeventHandle_t* handle, xevent_t xevent);

/*  @brief Removes last xevent_t function in the
 *  handle array.
 *
 *  @param handle: object pointer.
 */
void xeventRemove(xeventHandle_t* handle);

/*  @brief Frees the allocated xevent_t array
 *  memory. Deconstruction of the object. Do not
 *  use after deconstruction.
 *
 *  @param handle: object pointer.
 */
void xeventFree(xeventHandle_t* handle);

#endif
