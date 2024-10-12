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

#ifndef LEDSEQ_H_
#define LEDSEQ_H_

#include <stdint.h>

#define LEDSEQ_STOP     	  -1
#define LEDSEQ_LOOP			  -2
#define LEDSEQ_FREQ 	 	  100 /* 10ms */
#define LEDSEQ_TICK_TO_MS(X)  (X * 1000 / LEDSEQ_FREQ)

#define DOT 		10 			//  *
#define DASH 		(3 * DOT)	//  ***
#define LONG_DASH	(10 * DOT)	//  ********
#define GAP 		DOT			//  _
#define DASH_GAP    (3 * DOT)	//  ___
#define LONG_GAP 	(10 * DOT)	//  ________

int8_t ledseqInit(void);
int8_t ledseqTest(void);
int8_t ledseqRun (uint8_t led, int8_t priority, const int16_t* seq);
void   ledseqStop(uint8_t led);
void   ledseqWait(uint8_t led);

extern const int16_t seqFastBlinkLoop [];
extern const int16_t seqSlowBlinkLoop [];
extern const int16_t seqHeartBeat     [];
extern const int16_t seqWarningLoop   [];
extern const int16_t seqTestPassed    [];
extern const int16_t seqTestFailed	  [];

#define SEQ_ERROR_L		seqWarningLoop
#define SEQ_WAITING_L	seqSlowBlinkLoop
#define SEQ_PROCESS_L	seqFastBlinkLoop
#define SEQ_HEARTBEAT	seqHeartBeat
#define SEQ_PASSED		seqTestPassed
#define SEQ_FAILED		seqTestFailed

#endif /* LEDSEQ_H_ */
