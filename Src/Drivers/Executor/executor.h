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

#ifndef EXECUTOR_H_
#define EXECUTOR_H_

#include <Windows.h>
#include <stdint.h>
#include "xqueue.h"

/**
 * @brief Status codes for executor operations
 */
#define EXEC_SUCCESS     0    /* Operation completed successfully */
#define EXEC_ERROR      -1    /* Generic error occurred */
#define EXEC_EMPTY      -2    /* Queue is empty */
#define EXEC_FULL       -3    /* Queue is full */
#define EXEC_INVALID    -4    /* Invalid parameter */
#define EXEC_TIMEOUT    -5    /* Operation timed out */

#define EXEC_MILLIS() GetTickCount()
#define EXEC_DELAY_MS(x) Sleep(x)

/**
 * @brief Callback function type for instruction execution
 */
typedef void (*execFunction_t)(void* arg);

/**
 * @brief Callback function type for error handling
 */
typedef void (*execErrorHandler_t)(void* context, int8_t status);

/**
 * @brief Structure representing an executable instruction
 */
typedef struct {
    char* name;                /* Instruction name (for debugging) */
    execFunction_t exec;       /* Execution function */
    void* arg;                 /* Argument passed to exec function */
    uint32_t timeoutMs;        /* Maximum execution time in milliseconds (0 = no timeout) */
} exeInstruction_t;

/**
 * @brief Structure representing the executor instance
 */
typedef struct {
    xqueue_t execq;             /* Queue for instructions */
    volatile int8_t isRunning;  /* Flag indicating if executor is running */
    uint32_t defaultTimeoutMs;  /* Default timeout for instructions */
    execErrorHandler_t errorHandler; /* Error handler callback */
    void* errorContext;         /* Context passed to error handler */
    uint32_t statsExecuted;     /* Number of successfully executed instructions */
    uint32_t statsFailed;       /* Number of failed instruction executions */
} executor_t;

/**
 * @brief Creates a new executor instance
 * 
 * @param defaultTimeoutMs Default timeout in milliseconds (0 = no timeout)
 * @return executor_t* Pointer to the created executor instance or NULL on failure
 */
executor_t* executorNew(uint32_t defaultTimeoutMs);

/**
 * @brief Destroys an executor instance and frees all associated resources
 * 
 * @param exe Pointer to executor instance
 */
void executorFree(executor_t* exe);

/**
 * @brief Creates an instruction with the given parameters
 * 
 * @param name Instruction name (will be copied internally)
 * @param exec Function to execute
 * @param arg Argument to pass to exec function
 * @param timeoutMs Maximum execution time in milliseconds (0 = use default)
 * @return exeInstruction_t Created instruction
 */
exeInstruction_t executorInstructionNew(const char* name, 
                                    execFunction_t exec, 
                                    void* arg, 
                                    uint32_t timeoutMs);

/**
 * @brief Enqueues an instruction to be executed
 * 
 * @param exe Pointer to executor instance
 * @param instruction Instruction to be enqueued
 * @return int8_t Status code
 */
int8_t executorEnqueue(executor_t* exe, exeInstruction_t instruction);

/**
 * @brief Dequeues the next instruction to be executed
 * 
 * @param exe Pointer to executor instance
 * @param outInstruction Pointer to store the dequeued instruction
 * @return int8_t Status code
 */
int8_t executorDequeue(executor_t* exe, exeInstruction_t* outInstruction);

/**
 * @brief Runs the executor processing all queued instructions
 * 
 * @param exe Pointer to executor instance
 * @param maxInstructions Maximum number of instructions to execute (0 = no limit)
 * @return int8_t Status code
 */
int8_t executorRun(executor_t* exe, uint32_t maxInstructions);

/**
 * @brief Stops the executor from processing more instructions
 * 
 * @param exe Pointer to executor instance
 * @return int8_t Status code
 */
int8_t executorStop(executor_t* exe);

/**
 * @brief Sets the error handler for the executor
 * 
 * @param exe Pointer to executor instance
 * @param handler Error handler function
 * @param context Context passed to error handler
 * @return int8_t Status code
 */
int8_t executorSetErrorHandler(executor_t* exe, 
                            execErrorHandler_t handler, 
                            void* context);

/**
 * @brief Clears all instructions from the executor queue
 * 
 * @param exe Pointer to executor instance
 * @return int8_t Status code
 */
int8_t executorClear(executor_t* exe);

/**
 * @brief Gets the number of instructions in the executor queue
 * 
 * @param exe Pointer to executor instance
 * @param outSize Pointer to store the queue size
 * @return int8_t Status code
 */
int8_t executorGetQueueSize(executor_t* exe, size_t* outSize);

/**
 * @brief Gets execution statistics
 * 
 * @param exe Pointer to executor instance
 * @param outExecuted Pointer to store the number of executed instructions
 * @param outFailed Pointer to store the number of failed executions
 * @return int8_t Status code
 */
int8_t executorGetStats(executor_t* exe, 
                    uint32_t* outExecuted, 
                    uint32_t* outFailed);

/**
 * @brief Resets execution statistics
 * 
 * @param exe Pointer to executor instance
 * @return int8_t Status code
 */
int8_t executorResetStats(executor_t* exe);

#endif /* EXECUTOR_H_ */