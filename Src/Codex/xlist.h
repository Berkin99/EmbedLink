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

#ifndef XLIST_H_
#define XLIST_H_

#include <stdlib.h>
#include <sysdefs.h>

/**
 * @brief Structure representing a generic dynamic list.
 */
typedef struct {
    size_t length;			/**< Number of elements in the list */
    size_t item_size;		/**< Size of each element in bytes */
    void* array;			/**< Pointer to the array of elements */
    int (*_eq)(void* item1, void* item2); /**< Function pointer for equality comparison */
    int (*_lt)(void* item1, void* item2); /**< Function pointer for less-than comparison */
} xlist_t;

xlist_t xlistNew(size_t item_size, int (*eq)(void*, void*), int (*lt)(void*, void*));
void*   xlistGet(xlist_t* self, size_t idx);
void    xlistAppend(xlist_t* self, void* item);
void    xlistExtract(xlist_t* self, size_t idx, void* out_item);
void    xlistPop(xlist_t* self);
void    xlistPush(xlist_t* self, void* item);
void    xlistSwap(xlist_t* self, size_t idx1, size_t idx2);
void    xlistInsert(xlist_t* self, size_t idx, void* item);
int     xlistIndex(xlist_t* self, void* item);
size_t  xlistCount(xlist_t* self, void* item);
void    xlistReverse(xlist_t* self);
void    xlistSort(xlist_t* self);
void    xlistFree(xlist_t* self);

#endif /* XLIST_H_ */
 