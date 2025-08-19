/**
 * __  __ ____ _  __ ____ ___ __  __
 * \ \/ // __// |/ //  _// _ |\ \/ /
 *  \  // _/ /    /_/ / / __ | \  /
 *  /_//___//_/|_//___//_/ |_| /_/
 *
 * Yeniay Control Computer Firmware
 *
 * Copyright (C) 2022 Yeniay
 *
 * This program is free software: you
 * can redistribute it and/or modify it
 * under the terms of the GNU General
 * Public License as published by the
 * Free Software Foundation, in version 3.
 *
 * You should have received a copy of
 * the GNU General Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "watchtime.h"
#include "system.h"
#include "sysconfig.h"
#include "task.h"
#include "kinematics.h"
#include "sensor.h"

#define MAX_LIST_LENGTH 	6

taskAllocateStatic(WTIME, SYSTEM_TASK_STACK, SYSTEM_TASK_PRI);

static char 	 chr_list[MAX_LIST_LENGTH];
static uint32_t* int_list[MAX_LIST_LENGTH];
static uint8_t   list_len = 0;

void wtInit(void){
	taskCreateStatic(WTIME, wtTask, NULL);
}

void wtTask(void* argv){
	serialPrint("[+] WT Task init OK\n");

	systemWaitReady();
	
	delay(1500);

	while(1){
		for (uint8_t i = 0; i < list_len; ++i) {
			serialPrint("[%c] : %ld\n", chr_list[i], *int_list[i]);
		}
		delay(9);
	}
}

void wtADD(char chr, uint32_t* ptr){
	if(list_len > MAX_LIST_LENGTH) return;
	chr_list[list_len] = chr;
	int_list[list_len] = ptr;
	list_len++;
}
