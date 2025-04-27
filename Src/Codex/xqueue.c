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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xqueue.h>

static void* xqueueResize(xqueue_t* q, size_t new_cap) {
    void* new_arr = malloc(new_cap * q->item_size);
    if (!new_arr) return NULL;

    for (size_t i = 0; i < q->length; i++) {
        size_t idx = (q->front + i) % q->capacity;
        memcpy((char*)new_arr + i * q->item_size, (char*)q->array + idx * q->item_size, q->item_size);
    }
    free(q->array);
    q->array = new_arr;
    q->capacity = new_cap;
    q->front = 0;
    q->rear = q->length;
    return new_arr;
}

xqueue_t xqueueNew(size_t item_size) {
    xqueue_t q = {item_size, XQUEUE_INIT_CAPACITY, 0, 0, 0, malloc(XQUEUE_INIT_CAPACITY * item_size)};
    return q;
}

int8_t xqueueEnqueue(xqueue_t* q, void* item) {
    if (q->length == q->capacity && !xqueueResize(q, q->capacity * 2)) return 0;
    memcpy((char*)q->array + q->rear * q->item_size, item, q->item_size);
    q->rear = (q->rear + 1) % q->capacity;
    q->length++;
    return 1;
}

int8_t xqueueDequeue(xqueue_t* q, void* out_item) {
    if (q->length == 0) return 0;
    if (out_item) memcpy(out_item, (char*)q->array + q->front * q->item_size, q->item_size);
    q->front = (q->front + 1) % q->capacity;
    q->length--;
    return 1;
}

void* xqueueFront(xqueue_t* q) {
    return (q->length > 0) ? (char*)q->array + q->front * q->item_size : NULL;
}

void* xqueueRear(xqueue_t* q) {
    return (q->length > 0) ? (char*)q->array + ((q->rear - 1 + q->capacity) % q->capacity) * q->item_size : NULL;
}

int8_t xqueueIsEmpty(xqueue_t* q) {
    return q->length == 0;
}

size_t xqueueSize(xqueue_t* q) {
    return q->length;
}

void xqueueClear(xqueue_t* q) {
    q->length = 0;
    q->front = 0;
    q->rear = 0;
}

void xqueueFree(xqueue_t* q) {
    free(q->array);
    q->array = NULL;
    q->length = 0;
    q->capacity = 0;
}
