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
        if(l_flag){
            uint8_t data[32];
            int8_t rslt = xqueueDequeue(&q_fid, data);
            if(rslt){
                uavexeFidParse(data);
            }
            else{
                l_flag = 0;
            }
        }
        delay(10);
    }
}

/**
 *  @brief EXEPACK : [CMD_ID,  FPACK <17 byte>]    : 18byte
 *  @brief FPACK   : {FUNC_ID, argument <16 byte>] : 17byte
 */
void uavexeCmdParse(uint8_t* data){
    uint8_t cmd = data[0];
    switch (cmd){
        case UAVEXE_CMD_PARSE:
            uavexeFidParse(&data[1]);
        break;
        case UAVEXE_CMD_SET:
            uavexeFidSet(&data[1]);
        break;
    }
}

void uavexeFidParse(uint8_t* data){
    /* FID PACK EXECUTE */
    uint8_t fid = data[0];

    switch (fid){
        case UAVEXE_FID_DELAY:
            exe_DELAY((void*)&data[1]);
        break;
        case UAVEXE_FID_UAVCMD:
            exe_UAVCMD((void*)&data[1]);
        break;
        case UAVEXE_FID_PRINT:
            exe_PRINT((void*)&data[1]);
        break;
    }
}

void uavexeFidSet(uint8_t* data){
    /* FID PACK Set */
    xqueueEnqueue(&q_fid, data);
}

void uavexeLaunch(uint8_t* data){

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

