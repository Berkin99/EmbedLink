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
#include "quadconfig.h"
#include "quadcal.h"
#include "northcom.h"
#include "kinematics.h"
#include "sensor.h"
#include "led.h"
#include "ledseq.h"
#include "rc_interface.h"

void quadcalESC(quadcopter_t* pHandle){
    ledseqRun(QLED, 1, SEQ_PROCESS_L);

	ESC_MultiCalibrate(pHandle->motor, 4);
	TxMSG("[+] ESC CALIBRATED");

    ledseqStop(QLED);
    ledseqRun(QLED, 1, SEQ_PASSED);
	delay(100);
}

void quadcalCOM(void){
    ledseqRun(QLED, 1, SEQ_WAITING_L);
	int i = 0;
	while(i < 40){
		ncDebug("[+] NC CAL %d", i);
		i++;
		delay(50);
	}
    ledseqStop(QLED);
    ledseqRun(QLED, 1, SEQ_PASSED);
	delay(100);
}

void quadcalMotor(ESC_Handle_t* pMotor){
    ledseqRun(QLED, 1, SEQ_PROCESS_L);
	delay(100);

    ESC_Write(pMotor, 0.05f);
	delay(3000);
	ESC_Write(pMotor, 0);
    ledseqStop(QLED);
}

void quadcalMotors(quadcopter_t* pHandle){

	char mname[][4] = {"FR","RR","RL","FL"};
	char mrot [][4] = {"CCW","CW"};

	int i = 0;
	while(i < 4){
		ncDebug("[>] MOTOR %s CAL: %s", mname[i], mrot[i%2]);
		if(quadcalIterate() < 0){i++; continue;}
		quadcalMotor(&pHandle->motor[i]);
		i++;
	}

	TxMSG("[+] MOTORS CALIBRATED");
	delay(100);
}

void quadcalSensors(quadcopter_t* pHandle){
	int i = 0;
	while(i < sensorGetSize()){
		ncDebug("[>] %s CAL:", sensorName(i));
		if(quadcalIterate() < 0){i++; continue;}
        ledseqRun(QLED, 1, SEQ_PROCESS_L);
		sensorCalibrate(i);
	    ledseqStop(QLED);
    	delay(100);
		i++;
	}
	TxMSG("[+] SENSORS CALIBRATED");
	delay(100);
}

int8_t quadcalIterate(void){
	int8_t out = 0;
	while(1){
		if(rc.chZ.value >  0.9f){out =  1; break;}

		if(rc.chZ.value < -0.9f){out = -1; break;}
		delay(1);
	}

	while(1){
		if((rc.chZ.value < 0.2f) && (rc.chZ.value > -0.2f) ) break;
		delay(1);
	}
	TxMSG("[<]");
	return out;
}
