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

#include <string.h>

#include "rtos.h"
#include "uavexe.h"
#include "xqueue.h"
#include "uavcom.h"
#include "sysconfig.h"
#include "systime.h"
#include "uart.h"

void exe_DELAY(void*);
void exe_UAVCMD(void*);
void exe_PRINT(void*);

taskAllocateStatic(UAVEXE, CONTROL_TASK_STACK, CONTROL_TASK_PRI);
    
static xqueue_t q_fid;
static uint8_t  l_flag;

void uavexeInit(void){
    l_flag = 0;
    q_fid = xqueueNew(UAVEXE_FID_P_LEN);
    taskCreateStatic(UAVEXE, uavexeTask, NULL);
}

void uavexeTask(void* argv){

    serialPrint("[>] UAVEXE Init : OK\n");
    while (1){
        /* code */
        if(l_flag){ /* Launch Flag */
            uint8_t fid_p[UAVEXE_FID_P_LEN];
            int8_t rslt = xqueueDequeue(&q_fid, fid_p);
            
            if(rslt) uavexeCMD_PARSE(fid_p);
            else     l_flag = 0;
        }
        delay(10);
    }
}


/**
 *  @brief CMD_P   : [CMD_ID, FID_P <25 byte>]    : 26byte
 *  @brief FID_P   : {FID_ID, argument <24 byte>] : 25byte
 */
void uavexeParse(uint8_t* cmd_p){
    uint8_t cmd = cmd_p[0];
    switch (cmd){
        case UAVEXE_CMD_PARSE:
            uavexeCMD_PARSE(&cmd_p[1]);
        break;
        case UAVEXE_CMD_SET:
            uavexeCMD_SET(&cmd_p[1]);
        break;
        case UAVEXE_CMD_LAUNCH:
            uavexeCMD_LAUNCH();
        break;
    }
}

void uavexeCMD_PARSE(uint8_t* fid_p){
    /* FID PACK EXECUTE */
    uint8_t fid = fid_p[0];

    switch (fid){
        case UAVEXE_FID_DELAY:
            exe_DELAY((void*)&fid_p[1]);
        break;
        case UAVEXE_FID_UAVCMD:
            exe_UAVCMD((void*)&fid_p[1]);
        break;
        case UAVEXE_FID_PRINT:
            exe_PRINT((void*)&fid_p[1]);
        break;
    }
}

void uavexeCMD_SET(uint8_t* fid_p){
    /* FID PACK Set */
    serialPrint("[>] UAVEXE CMD SET [FID] %d\n", fid_p[0]);
    xqueueEnqueue(&q_fid, fid_p);
}

void uavexeCMD_LAUNCH(void){
    serialPrint("[>] UAVEXE LAUNCH\n");
    l_flag = 1;
}

void exe_DELAY(void* data){
    uint32_t d_ms;
    memcpy((void*)&d_ms, (void*)data, sizeof(uint32_t));
    delay(d_ms);
}

void exe_UAVCMD(void* data){
    uavcomParse(data);
}

void exe_PRINT(void* data){
    serialPrint("%s", data);
}

