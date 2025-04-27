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
#include <string.h>
#include <assert.h>
#include "executor.h"

/* Private function prototypes */
static void execInstructionFree(exeInstruction_t* instruction);
static int8_t execIsValid(executor_t* exe);
static void execHandleError(executor_t* exe, int8_t status);

/**
 * @brief Creates a new executor instance
 */
executor_t* executorNew(uint32_t defaultTimeoutMs) {
    executor_t* exe = (executor_t*)calloc(1, sizeof(executor_t));
    if (!exe) {
        return NULL;
    }
    
    /* Initialize instruction queue */
    exe->execq = xqueueNew(sizeof(exeInstruction_t));
    
    exe->isRunning = 0;
    exe->defaultTimeoutMs = defaultTimeoutMs;
    exe->errorHandler = NULL;
    exe->errorContext = NULL;
    exe->statsExecuted = 0;
    exe->statsFailed = 0;
    
    return exe;
}

/**
 * @brief Destroys an executor instance
 */
void executorFree(executor_t* exe) {
    if (!execIsValid(exe)) return;
    
    /* Clear all instructions and free queue */
    executorClear(exe);
    xqueueFree(&exe->execq);
    
    free(exe);
}

/**
 * @brief Creates an instruction with given parameters
 */
exeInstruction_t executorInstructionNew(const char* name, 
                                    execFunction_t exec, 
                                    void* arg, 
                                    uint32_t timeoutMs) {
    exeInstruction_t instruction = {0};
    
    /* Copy name if provided */
    if (name) {
        instruction.name = (char*)malloc(strlen(name) + 1);
        if (instruction.name) strcpy(instruction.name, name);
    } else instruction.name = NULL;
    
    instruction.exec = exec;
    instruction.arg = arg;
    instruction.timeoutMs = timeoutMs;
    
    return instruction;
}

/**
 * @brief Enqueues an instruction
 */
int8_t executorEnqueue(executor_t* exe, exeInstruction_t instruction) {
    if (!execIsValid(exe)) return EXEC_INVALID;
    
    /* Validate instruction */
    if (!instruction.exec) {
        execHandleError(exe, EXEC_INVALID);
        return EXEC_INVALID;
    }
    
    /* Enqueue the instruction */
    int8_t result = xqueueEnqueue(&exe->execq, &instruction);
    if (result != 0) {
        execHandleError(exe, EXEC_FULL);
        return EXEC_FULL;
    }
    
    return EXEC_SUCCESS;
}

/**
 * @brief Dequeues the next instruction
 */
int8_t executorDequeue(executor_t* exe, exeInstruction_t* outInstruction) {
    if (!execIsValid(exe) || !outInstruction) return EXEC_INVALID;
    
    if (xqueueIsEmpty(&exe->execq))return EXEC_EMPTY;
    
    int8_t result = xqueueDequeue(&exe->execq, outInstruction);
    return (result == 0) ? EXEC_SUCCESS : EXEC_ERROR;
}

/**
 * @brief Runs the executor processing queued instructions
 */
int8_t executorRun(executor_t* exe, uint32_t maxInstructions) {
    if (!execIsValid(exe)) return EXEC_INVALID;
    
    exe->isRunning = 1;
    uint32_t executedCount = 0;
    int8_t status = EXEC_SUCCESS;
    
    while (exe->isRunning) {
        /* Check if we've reached the maximum number of instructions */
        if (maxInstructions > 0 && executedCount >= maxInstructions) break;
        
        /* Dequeue the next instruction */
        exeInstruction_t instruction;
        status = executorDequeue(exe, &instruction);
        if (status != EXEC_SUCCESS) break; /* No more instructions */
        
        /* Execute the instruction */
        if (instruction.exec) {
            /* Use default timeout if not specified */
            if (instruction.timeoutMs == 0) instruction.timeoutMs = exe->defaultTimeoutMs;

            ////////////////////////////////////////////////////////////////////////
            uint32_t exec_end = EXEC_MILLIS() + instruction.timeoutMs;
            instruction.exec(instruction.arg);
            if(EXEC_MILLIS() < exec_end) EXEC_DELAY_MS(EXEC_MILLIS() - exec_end);
            ////////////////////////////////////////////////////////////////////////

            exe->statsExecuted++;
            executedCount++;
        } else exe->statsFailed++;
        
        /* Free instruction resources */
        execInstructionFree(&instruction);
    }
    
    exe->isRunning = 0;
    return status;
}

/**
 * @brief Stops the executor
 */
int8_t executorStop(executor_t* exe) {
    if (!execIsValid(exe)) {
        return EXEC_INVALID;
    }
    
    exe->isRunning = 0;
    return EXEC_SUCCESS;
}

/**
 * @brief Sets the error handler
 */
int8_t executorSetErrorHandler(executor_t* exe, 
                            execErrorHandler_t handler, 
                            void* context) {
    if (!execIsValid(exe)) {
        return EXEC_INVALID;
    }
    
    exe->errorHandler = handler;
    exe->errorContext = context;
    return EXEC_SUCCESS;
}

/**
 * @brief Clears all instructions
 */
int8_t executorClear(executor_t* exe) {
    if (!execIsValid(exe)) {
        return EXEC_INVALID;
    }
    
    /* Dequeue and free all instructions */
    while (!xqueueIsEmpty(&exe->execq)) {
        exeInstruction_t instruction;
        if (xqueueDequeue(&exe->execq, &instruction) == 0) {
            execInstructionFree(&instruction);
        }
    }
    
    xqueueClear(&exe->execq);
    return EXEC_SUCCESS;
}

/**
 * @brief Gets the number of instructions in the queue
 */
int8_t executorGetQueueSize(executor_t* exe, size_t* outSize) {
    if (!execIsValid(exe) || !outSize) {
        return EXEC_INVALID;
    }
    
    *outSize = xqueueSize(&exe->execq);
    return EXEC_SUCCESS;
}

/**
 * @brief Gets execution statistics
 */
int8_t executorGetStats(executor_t* exe, 
                    uint32_t* outExecuted, 
                    uint32_t* outFailed) {
    if (!execIsValid(exe)) {
        return EXEC_INVALID;
    }
    
    if (outExecuted) {
        *outExecuted = exe->statsExecuted;
    }
    
    if (outFailed) {
        *outFailed = exe->statsFailed;
    }
    
    return EXEC_SUCCESS;
}

/**
 * @brief Resets execution statistics
 */
int8_t executorResetStats(executor_t* exe) {
    if (!execIsValid(exe)) {
        return EXEC_INVALID;
    }
    
    exe->statsExecuted = 0;
    exe->statsFailed = 0;
    return EXEC_SUCCESS;
}

/* ------------------------- Private functions ------------------------- */

/**
 * @brief Frees resources associated with an instruction
 */
static void execInstructionFree(exeInstruction_t* instruction) {
    if (instruction && instruction->name) {
        free(instruction->name);
        instruction->name = NULL;
    }
}

/**
 * @brief Checks if executor instance is valid
 */
static int8_t execIsValid(executor_t* exe) {
    return exe != NULL;
}

/**
 * @brief Handles errors through error callback if set
 */
static void execHandleError(executor_t* exe, int8_t status) {
    if (exe && exe->errorHandler) {
        exe->errorHandler(exe->errorContext, status);
    }
}