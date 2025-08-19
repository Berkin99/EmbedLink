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

#include "executer.h"
#include "xqueue.h"
#include <stdlib.h>
#include <string.h>

struct executer_s {
    xqueue_t queue; /* stores executerItem_t elements */
};

executer_t* executerCreate(void)
{
    executer_t* self = (executer_t*)malloc(sizeof(executer_t));
    if (!self) return NULL;
    self->queue = xqueueNew(sizeof(executerItem_t));
    return self;
}

void executerFree(executer_t* self)
{
    if (!self) return;
    xqueueFree(&self->queue);
    free(self);
}

int8_t executerEnqueue(executer_t* self, executer_f f, const void* arg, size_t argSize)
{
    if (!self || !f) return EXECUTER_FAIL;

    executerItem_t it;
    it.cb = f;

    /* Copy argument (truncate >16) and zero-fill remaining bytes */
    size_t copyLen = (arg && argSize) ? (argSize > EXECUTER_ARG_MAX ? EXECUTER_ARG_MAX : argSize) : 0u;
    if (copyLen) {
        memcpy(it.arg.b, arg, copyLen);
    }
    if (copyLen < EXECUTER_ARG_MAX) {
        memset(it.arg.b + copyLen, 0, EXECUTER_ARG_MAX - copyLen);
    }
    it.argSize = (uint8_t)copyLen;

    int8_t ok = xqueueEnqueue(&self->queue, &it);
    if (!ok) return EXECUTER_FAIL;

    return (argSize > EXECUTER_ARG_MAX) ? (EXECUTER_OK | EXECUTER_TRUNC) : EXECUTER_OK;
}

int8_t executerRunNext(executer_t* self)
{
    if (!self) return 0;

    executerItem_t it;
    if (!xqueueDequeue(&self->queue, &it)) return 0;

    if (it.cb) {
        it.cb((void*)it.arg.b);
    }
    return 1;
}

size_t executerRunAll(executer_t* self, size_t maxRuns)
{
    if (!self) return 0;
    size_t ran = 0;
    while (!xqueueIsEmpty(&self->queue) && (maxRuns == 0 || ran < maxRuns)) {
        ran += executerRunNext(self) ? 1u : 0u;
    }
    return ran;
}

void executerClear(executer_t* self)
{
    if (!self) return;
    xqueueClear(&self->queue);
}

int8_t executerIsEmpty(executer_t* self)
{
    if (!self) return 1;
    return xqueueIsEmpty(&self->queue);
}

size_t executerSize(executer_t* self)
{
    if (!self) return 0;
    return xqueueSize(&self->queue);
}
