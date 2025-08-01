/**
 *    __  __ ____ _  __ ____ ___ __  __
 *    \ \/ // __// |/ //  _// _ |\ \/ /
 *     \  // _/ /    /_/ / / __ | \  /
 *     /_//___//_/|_//___//_/ |_| /_/
 *
 *         Yeniay System Firmware
 *
 *       Copyright (C) 2024 Yeniay
 *
 * This  program  is  free software:   you
 * can  redistribute it  and/or  modify it
 * under  the  terms of  the  GNU  General
 * Public  License as  published  by   the
 * Free Software Foundation, in version 3.
 *
 * You  should  have  received  a  copy of
 * the  GNU  General  Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

// #include "systime.h"
// #include "quadconfig.h"
// #include "quadcal.h"
// #include "northcom.h"
// #include "kinematics.h"
// #include "control.h"
// #include "sensor.h"
// #include "led.h"
// #include "ledseq.h"

// static ledseqContext_t calseq  = {QLED, 5, seq_fastblinkloop};

// void quadcalESC(quadcopter_t* pHandle){
// 	ledseqRun(&calseq);
// 	ESC_MultiCalibrate(pHandle->motor, 4);
// 	TxMSG("[+] ESC CALIBRATED");
// 	ledseqStop(&calseq);

// 	delay(100);

// }

// void quadcalCOM(void){
// 	ledseqRun(&calseq);
// 	int i = 0;
// 	while(i < 40){
// 		ncDebug("[+] NC CAL %d", i);
// 		i++;
// 		delay(50);
// 	}
// 	ledseqStop(&calseq);
// 	delay(100);
// }

// void quadcalMotor(ESC_Handle_t* pMotor){
// 	ledseqRun(&calseq);
// 	delay(100);
// 	ESC_Write(pMotor, 0.05f);
// 	delay(3000);
// 	ESC_Write(pMotor, 0);
// 	ledseqStop(&calseq);
// }

// void quadcalMotors(quadcopter_t* pHandle){

// 	char mname[][4] = {"FR","RR","RL","FL"};
// 	char mrot [][4] = {"CCW","CW"};

// 	int i = 0;
// 	while(i < 4){
// 		ncDebug("[>] MOTOR %s CAL: %s", mname[i], mrot[i%2]);
// 		if(quadcalIterate() < 0){i++; continue;}
// 		quadcalMotor(&pHandle->motor[i]);
// 		i++;
// 	}

// 	TxMSG("[+] MOTORS CALIBRATED");
// 	delay(100);
// }

// void quadcalSensors(quadcopter_t* pHandle){
// 	int i = 0;
// 	while(i < sensorGetSize()){
// 		ncDebug("[>] %s CAL:", sensorName(i));
// 		if(quadcalIterate() < 0){i++; continue;}
// 		ledseqRun(&calseq);
// 		sensorCalibrate(i);
// 		ledseqStop(&calseq);
// 		delay(100);
// 		i++;
// 	}
// 	TxMSG("[+] SENSORS CALIBRATED");
// 	delay(100);
// }

// int8_t quadcalIterate(void){
// 	int8_t out = 0;
// 	while(1){
// 		if(controller()->crange.z >  0.9f){out =  1; break;}

// 		if(controller()->crange.z < -0.9f){out = -1; break;}
// 		delay(1);
// 	}

// 	while(1){
// 		if((controller()->crange.z < 0.2f) && (controller()->crange.z > -0.2f) ) break;
// 		delay(1);
// 	}
// 	TxMSG("[<]");
// 	return out;
// }
