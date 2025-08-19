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


#ifndef EXECUTER_H_
#define EXECUTER_H_

#include <stdint.h>
#include <stddef.h>

#define EXECUTER_ARG_MAX 16u

/* Return codes (bitwise-combinable) */
#define EXECUTER_FAIL   0
#define EXECUTER_OK     1
#define EXECUTER_TRUNC  2

/* Callback type: receives pointer to inline arg buffer */
typedef void (*executer_f)(void* arg);

/* Inline argument storage with natural alignment */
typedef union {
    uint8_t  b[EXECUTER_ARG_MAX];
} executerArg_t;

/* Queue element */
typedef struct {
    executer_f    cb;      /* function to execute */
    executerArg_t arg;     /* inline copy of user argument */
    uint8_t       argSize; /* actual copied size (0..16) */
} executerItem_t;

/* Opaque executor context */
typedef struct executer_s executer_t;

/* API */
executer_t* executerCreate(void);
void        executerFree(executer_t* self);

/* Enqueue: callback gets (void* arg) pointing to inline arg buffer */
int8_t      executerEnqueue(executer_t* self,
                            executer_f f,
                            const void* arg,
                            size_t argSize);

int8_t      executerRunNext(executer_t* self);
size_t      executerRunAll(executer_t* self, size_t maxRuns);

void        executerClear(executer_t* self);
int8_t      executerIsEmpty(executer_t* self);
size_t      executerSize(executer_t* self);

#endif /* EXECUTER_H_ */
