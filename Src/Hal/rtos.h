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

#ifndef RTOS_H_
#define RTOS_H_

/*
 * This code provides a set of static inline wrapper functions for common RTOS features,
 * designed to offer a flexible interface that abstracts task, semaphore, mutex,
 * and queue management. By using these wrappers, the underlying RTOS-specific
 * implementation (e.g., FreeRTOS) is hidden, making it easier to port the code
 * to another RTOS if needed.
 *
 * Key features:
 * - Task creation, deletion, and delay operations are abstracted into simple functions.
 * - Semaphores and mutexes can be created and controlled with a unified interface.
 * - Queue operations, including sending and receiving, are handled with flexibility
 *   for both standard and ISR contexts.
 *
 * This abstraction ensures rapid compatibility with different RTOS implementations
 * by minimizing changes in the higher-level application code.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define RTOS_TRUE		  pdTRUE
#define RTOS_FALSE		  pdFALSE
#define RTOS_OK			  pdPASS
#define RTOS_ERROR		  pdFAIL
#define RTOS_MAX_DELAY    portMAX_DELAY
#define RTOS_MIN_STACK    configMINIMAL_STACK_SIZE

typedef SemaphoreHandle_t semaphore_t;
typedef SemaphoreHandle_t mutex_t;
typedef TaskHandle_t      task_t;
typedef QueueHandle_t     queue_t;

#define taskAllocateStatic(NAME, STACK_DEPTH, PRIORITY) \
static const int    NAME##_stackDepth = (STACK_DEPTH);\
static StackType_t  NAME##_stackBuffer[(STACK_DEPTH)];\
static StaticTask_t NAME##_taskBuffer;\
static UBaseType_t 	NAME##_priority = PRIORITY;

#define taskCreateStatic(NAME, FUNCTION, PARAMETERS) \
xTaskCreateStatic((FUNCTION), #NAME, NAME##_stackDepth, (PARAMETERS), (NAME##_priority), NAME##_stackBuffer, &NAME##_taskBuffer)

#define queueAllocateStatic(NAME, LENGTH, ITEM_SIZE)\
static const int 	 NAME##_length = (LENGTH); \
static const int 	 NAME##_itemSize = (ITEM_SIZE); \
static uint8_t 		 NAME##_storage[(LENGTH) * (ITEM_SIZE)]; \
static StaticQueue_t NAME##_queue;

#define queueCreateStatic(NAME) \
xQueueCreateStatic(NAME##_length, NAME##_itemSize, NAME##_storage, &NAME##_queue)

static inline mutex_t mutexCreate(void) {
    return xSemaphoreCreateMutex();
}

static inline semaphore_t semaphoreCreate(void) {
    return xSemaphoreCreateBinary();
}

static inline int8_t semaphoreTake(semaphore_t sem, uint32_t timeout) {
    return (int8_t)xSemaphoreTake(sem, timeout);
}

static inline int8_t semaphoreGive(semaphore_t sem) {
    return (int8_t)xSemaphoreGive(sem);
}

static inline int8_t semaphoreTakeISR(semaphore_t sem) {
    int32_t xHigherPriorityTaskWoken = pdFALSE;
    int8_t result = (int8_t)xSemaphoreTakeFromISR(sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return result;
}

static inline int8_t semaphoreGiveISR(semaphore_t sem) {
	int32_t xHigherPriorityTaskWoken = pdFALSE;
	int8_t result = (int8_t)xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return result;
}

static inline int8_t mutexTake(mutex_t mutex, uint32_t timeout) {
    return (int8_t)xSemaphoreTake(mutex, timeout);
}

static inline int8_t mutexGive(mutex_t mutex) {
    return (int8_t)xSemaphoreGive(mutex);
}

static inline queue_t queueCreate(uint32_t length, uint32_t itemSize) {
    return xQueueCreate(length, itemSize);
}

static inline void queueDelete(queue_t queue) {
    vQueueDelete(queue);
}

static inline int8_t queueSend(queue_t queue, const void* item, uint32_t timeout) {
    return (int8_t)xQueueSend(queue, item, timeout);
}

static inline int8_t queueSendFront(queue_t queue, const void* item, uint32_t timeout) {
    return (int8_t)xQueueSendToFront(queue, item, timeout);
}

static inline int8_t queueSendISR(queue_t queue, const void* item) {
	int32_t xHigherPriorityTaskWoken = pdFALSE;
	int8_t result = (int8_t)xQueueSendFromISR(queue, item, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return result;
}

static inline int8_t queueReceive(queue_t queue, void* item, uint32_t timeout) {
    return (int8_t)xQueueReceive(queue, item, timeout);
}

static inline int8_t queueReceiveISR(queue_t queue, void* item) {
	int32_t xHigherPriorityTaskWoken = pdFALSE;
	int8_t result = (int8_t)xQueueReceiveFromISR(queue, item, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return result;
}

static inline task_t taskCreate(void (*taskFunc)(void*), const char* name, uint16_t stackSize, void* parameters, int32_t priority) {
    task_t handle;
    xTaskCreate(taskFunc, name, stackSize, parameters, (UBaseType_t)priority, &handle);
    return handle;
}

static inline void taskDelete(task_t task){
	vTaskDelete(task);
}

static inline void taskDelay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline void taskDelayUntil(uint32_t* lastWakeTime, uint32_t ms) {
    vTaskDelayUntil((TickType_t*)lastWakeTime, pdMS_TO_TICKS(ms));
}

static inline void taskYield(void) {
    taskYIELD();
}

static inline int32_t taskPriorityGet(task_t task) {
    return (int32_t)uxTaskPriorityGet(task);
}

static inline void taskPrioritySet(task_t task, int32_t priority) {
    vTaskPrioritySet(task, (UBaseType_t)priority);
}

static inline void taskSuspend(task_t task) {
    vTaskSuspend(task);
}

static inline void taskResume(task_t task) {
    vTaskResume(task);
}

static inline void taskStartScheduler(void){
	vTaskStartScheduler();
}

static inline uint32_t taskGetTickCount(void){
	return xTaskGetTickCount();
}

#endif /* RTOS_H_ */
