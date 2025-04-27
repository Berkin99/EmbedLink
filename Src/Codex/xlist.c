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
#include <xlist.h>

static void* xlistResize(xlist_t* self, size_t new_size) {
    return realloc(self->array, new_size * self->item_size);
}

static void xlistShift(xlist_t* self, size_t start, int shift) {
    char* arr = (char*)self->array;
    memmove(&arr[(start + shift) * self->item_size], 
            &arr[start * self->item_size], 
            (self->length - start) * self->item_size);
}

xlist_t newList(size_t item_size, int (*eq)(void*, void*), int (*lt)(void*, void*)) {
    return (xlist_t){0, item_size, NULL, eq, lt};
}

void* xlistGet(xlist_t* self, size_t idx) {
    return (idx < self->length) ? (char*)self->array + idx * self->item_size : NULL;
}

void xlistAppend(xlist_t* self, void* item) {
    self->array = xlistResize(self, self->length + 1);
    memcpy((char*)self->array + self->length * self->item_size, item, self->item_size);
    self->length++;
}

void xlistExtract(xlist_t* self, size_t idx, void* out_item) {
    if (idx >= self->length) return;
    char* arr = (char*)self->array;
    if (out_item) memcpy(out_item, &arr[idx * self->item_size], self->item_size);
    xlistShift(self, idx + 1, -1);
    self->array = xlistResize(self, --self->length);
}

void xlistPop(xlist_t* self) {
    if (self->length > 0) {
        self->array = xlistResize(self, --self->length);
    }
}

void xlistPush(xlist_t* self, void* item) {
    self->array = xlistResize(self, self->length + 1);
    xlistShift(self, 0, 1);
    memcpy(self->array, item, self->item_size);
    self->length++;
}

void xlistSwap(xlist_t* self, size_t idx1, size_t idx2) {
    if (idx1 >= self->length || idx2 >= self->length) return;
    char temp[self->item_size];
    char* arr = (char*)self->array;
    memcpy(temp, &arr[idx1 * self->item_size], self->item_size);
    memcpy(&arr[idx1 * self->item_size], &arr[idx2 * self->item_size], self->item_size);
    memcpy(&arr[idx2 * self->item_size], temp, self->item_size);
}

void xlistInsert(xlist_t* self, size_t idx, void* item) {
    if (idx > self->length) return;
    self->array = xlistResize(self, self->length + 1);
    xlistShift(self, idx, 1);
    memcpy((char*)self->array + idx * self->item_size, item, self->item_size);
    self->length++;
}

int xlistIndex(xlist_t* self, void* item) {
    for (size_t i = 0; i < self->length; i++) {
        if (self->_eq((char*)self->array + i * self->item_size, item)) {
            return (int)i;
        }
    }
    return -1;
}

size_t xlistCount(xlist_t* self, void* item) {
    size_t count = 0;
    for (size_t i = 0; i < self->length; i++) {
        if (self->_eq((char*)self->array + i * self->item_size, item)) {
            count++;
        }
    }
    return count;
}

void xlistReverse(xlist_t* self) {
    for (size_t left = 0, right = self->length - 1; left < right; left++, right--) {
        xlistSwap(self, left, right);
    }
}

void xlistSort(xlist_t* self) {
    for (size_t i = 0; i < self->length - 1; i++) {
        for (size_t j = 0; j < self->length - i - 1; j++) {
            char* arr = (char*)self->array;
            if (self->_lt(&arr[(j + 1) * self->item_size], &arr[j * self->item_size])) {
                xlistSwap(self, j, j + 1);
            }
        }
    }
}

void xlistFree(xlist_t* self) {
    free(self->array);
    self->array = NULL;
    self->length = 0;
}