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

#ifndef EVENT_H_
#define EVENT_H_

#include <stdlib.h>
#include <stdint.h>

#define EVENT_INIT_CAPACITY         2
#define EVENT_MAX_CAPACITY          16

typedef enum{
    EVENT_CLOSE = 0,
    EVENT_READY = 1,
    EVENT_ERROR = 2
}eventState_e;

typedef void (*event_t)(void);

typedef struct eventHandle_s{
    event_t* arr;          /* Events for ready to call */
    uint8_t  arr_size;     /* Number of event_t in the array */
    uint8_t  arr_capacity; /* @arr length */
    eventState_e state;    /* Debugging state of the eventHandle_t */
}eventHandle_t;

/*  @brief Creates new eventHandle. Initial
 *  allocated memory for event functions is
 *  described as @EVENT_INIT_CAPACITY
 *
 *  @return eventHandle_t constructor
 *  @retval handle.state == EVENT_ERROR:
 *  memory allocation error!
 *  @retval handle.state == EVENT_READY:
 *  successful memory allocation.
 */
eventHandle_t eventNew(void);

/*  @brief Calls every event_t event function
 *  in the handle array (@handle->arr).
 *
 *  @param handle: object pointer.
 */
void eventCall(eventHandle_t* handle);

/*  @brief Adds event_t function in the handle
 *  array. If arr_size is exceeds the capacity
 *  of the event_t array, reallocates the memory
 *  as capacity multiplied by 2. If handle is not
 *  initalized (not used the construct function)
 *  @eventNew(), it constructs the object.
 *
 *  @param handle: object pointer.
 */
void eventAdd(eventHandle_t* handle, event_t event);

/*  @brief Removes last event_t function in the
 *  handle array.
 *
 *  @param handle: object pointer.
 */
void eventRemove(eventHandle_t* handle);

/*  @brief Frees the allocated event_t array
 *  memory. Deconstruction of the object. Do not
 *  use after deconstruction.
 *
 *  @param handle: object pointer.
 */
void eventFree(eventHandle_t* handle);

#endif
