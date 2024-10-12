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

#include "systime.h"
#include "sysconfig.h"
#include "sysdebug.h"
#include "ledseq.h"
#include "rtos.h"
#include "led.h"

typedef struct{
	uint8_t  led;
	mutex_t  mutex;
	int8_t   priority;
	const int16_t* seqstart;
	const int16_t* seq;
	int16_t  counter;
}ledseq_t;

static ledseq_t ledseqs[LED_COUNT];

static uint8_t isInit = 0;

void ledseqTask(void* argv);

int8_t ledseqInit(void){
	if(isInit) return E_OVERWRITE;

	for (uint8_t i = 0; i < LED_COUNT; ++i) {
		ledseqs[i].led      = i;
		ledseqs[i].mutex    = mutexCreate();
		ledseqs[i].priority = LEDSEQ_STOP;
		ledseqs[i].seqstart = NULL;
		ledseqs[i].seq      = NULL;
		ledseqs[i].counter  = 0;
	}

	taskCreate(ledseqTask, "LEDSEQ", LEDSEQ_TASK_STACK, NULL, LEDSEQ_TASK_PRI);
	isInit = 1;
	return OK;
}

int8_t ledseqTest(void){ return OK;}

void ledseqTask(void* argv){

	uint32_t lastWakeTime = taskGetTickCount();
	while(1){
		for (uint8_t i = 0; i < LED_COUNT; ++i) {
			/* Ledseq update */
			if(ledseqs[i].priority <= LEDSEQ_STOP) continue; /* Empty Buffer */

			if(++(ledseqs[i].counter) >= *(ledseqs[i].seq)){
				ledToggle(ledseqs[i].led);
				ledseqs[i].counter = 0;

				ledseqs[i].seq++;
				if(*(ledseqs[i].seq) == LEDSEQ_LOOP) ledseqs[i].seq = ledseqs[i].seqstart;
				if(*(ledseqs[i].seq) == LEDSEQ_STOP) ledseqStop(ledseqs[i].led);
			}
		}
		/* Ledseq constatnt refresh rate */
		taskDelayUntil(&lastWakeTime, (1000 / LEDSEQ_FREQ));
	}
}

int8_t ledseqRun(uint8_t led, int8_t priority, const int16_t* seq){
	int8_t status = mutexTake(ledseqs[led].mutex, 0);
	ledSet(led, 1);
	if(status == RTOS_OK){
		ledseqs[led].priority = priority;
		ledseqs[led].seq = seq;
		ledseqs[led].seqstart = seq;
		return OK;
	}
	return E_ERROR;
}

void ledseqStop(uint8_t led){
	ledseqs[led].priority = LEDSEQ_STOP;
	ledSet(led, 0);
	mutexGive(ledseqs[led].mutex);
}

void ledseqWait(uint8_t led){
	mutexTake(ledseqs[led].mutex, RTOS_MAX_DELAY);
	mutexGive(ledseqs[led].mutex);
}

const int16_t seqFastBlinkLoop [] = {DOT,GAP,DOT,GAP,LEDSEQ_LOOP};
const int16_t seqSlowBlinkLoop [] = {DOT,LONG_GAP,LEDSEQ_LOOP};
const int16_t seqHeartBeat     [] = {DOT,GAP,DOT,LONG_DASH,LEDSEQ_LOOP};
const int16_t seqWarningLoop   [] = {DASH,DASH_GAP,LEDSEQ_LOOP};
const int16_t seqTestPassed    [] = {DOT,GAP,DOT,GAP,DOT,GAP,DOT,LEDSEQ_STOP};
const int16_t seqTestFailed    [] = {DOT,GAP,LONG_DASH,LEDSEQ_STOP};
