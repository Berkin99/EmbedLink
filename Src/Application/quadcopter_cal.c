///**
// *    __  __ ____ _  __ ____ ___ __  __
// *    \ \/ // __// |/ //  _// _ |\ \/ /
// *     \  // _/ /    /_/ / / __ | \  /
// *     /_//___//_/|_//___//_/ |_| /_/
// *
// *         Yeniay System Firmware
// *
// *       Copyright (C) 2024 Yeniay
// *
// * This  program  is  free software:   you
// * can  redistribute it  and/or  modify it
// * under  the  terms of  the  GNU  General
// * Public  License as  published  by   the
// * Free Software Foundation, in version 3.
// *
// * You  should  have  received  a  copy of
// * the  GNU  General  Public License along
// * with this program. If not, see
// * <http://www.gnu.org/licenses/>.
// */
//
//#include <stdint.h>
//#include <stdio.h>
//#include <math.h>
//
//#include "system.h"
//#include "systime.h"
//#include "FreeRTOS.h"
//#include "task.h"
//
//#include "quadcopter_cal.h"
//
//#include "esc.h"
//#include "led.h"
//#include "num.h"
//#include "kinematics.h"
//#include "quaternion.h"
//#include "madgwick.h"
//#include "filter.h"
//#include "rc_interface.h"
//#include "northcom.h"
//#include "static_mem.h"
//#include "ledseq.h"
//
///** MOTOR MAPPING :
// *  Front Right Motor : CCW : esc1 : FR
// *  Rear Right Motor  : CW  : esc2 : RR
// *  Rear Left Motor   : CCW : esc3 : RL
// *  Front Left Motor  : CW  : esc4 : FL
// */
//
//STATIC_MEM_TASK_ALLOC(quadcal,CONTROL_TASK_STACK,CONTROL_TASK_PRI)
//void calTask(void* argv);
//
//void calInit(void){
//
//	/* Get Transmorm*/
//	transform = kinematicsState();
//	gcs = kinematicsGeoState();
//
//	rc_handle = ncRemoteController();
//	/* ESC Calibrate */
//	for(uint8_t i =0; i<4; i++){
//		motor[i] = ESC_NewHandle_t(ESC_PROTOCOL_STANDARD, i);
//	}
////	ESC_MultiCalibrate(motor, 4);
//
//	STATIC_MEM_TASK_CREATE(quadcal,calTask,NULL);
//}
//
//void calTask(void* argv){
//
//	systemWaitReady();
//	/* Wait Connection */
//	while(ncLastDataTime()==0){delay(10);}
//
//	delay(1000);
//	TxMSG("<QUAD CALIBRATION>");
//
//	delay(100);
//
//	while(1){
//
//		TxMSG("Debug Accel? : Y / N");
//		CAL_ANSWER_WAIT();
//		if(CAL_ANSWER_YES) calDebug_accel();
//
//		delay(1000);
//
//		TxMSG("Debug Angle : Y / N");
//		CAL_ANSWER_WAIT();
//		if(CAL_ANSWER_YES) calDebug_angle();
//
//		delay(1000);
//
//		TxMSG("Debug Mag? : Y / N");
//		CAL_ANSWER_WAIT();
//		if(CAL_ANSWER_YES) calDebug_mag();
//
//		delay(1000);
//
//		TxMSG("Debug Motor? : Y / N");
//		CAL_ANSWER_WAIT();
//		if(CAL_ANSWER_YES) calDebug_motor();
//
//		delay(1000);
//
//	}
//}
//
//
//#define CAL_STEP_WAIT() ledseqStop(&cx_process);\
//						while(rc_handle->range.z < 0.6) delay(1);\
//					    while(rc_handle->range.z > 0.2) delay(1);\
//					    ledseqRun(&cx_process);\
//
//#define CAL_ANSWER_WAIT() ledseqStop(&cx_process); \
//						  while(rc_handle->range.z > -0.6f && rc_handle->range.z < 0.6f) delay(1);\
//						  ledseqRun(&cx_process);  \
//
//#define CAL_ANSWER_YES	(rc_handle->range.z > 0.0f)
//
////
////void calDebug_accel(void){
////
////	TxMSG("calDebug_accel");
////
////	TxMSG("Set Quad Parallel");
////	CAL_STEP_WAIT();
////	ncDebug("Acc[x:%.2f,y:%.2f]",transform->acceleration.x,transform->acceleration.y);
////}
////
////void calDebug_angle(){
////
////	TxMSG("calDebug_angle");
////
////	TxMSG("Tilt +45 Pitch");
////	CAL_STEP_WAIT();
////	if(transform->angle.x > 20) ncDebug(" > +1 .");
////	else if(transform->angle.x < -20) ncDebug(" > -1 .");
////	else ncDebug(" > error .");
////
////	TxMSG("Tilt +45 Roll");
////	CAL_STEP_WAIT();
////	if(transform->angle.y > 20) ncDebug(" > +1 .");
////	else if(transform->angle.y < -20) ncDebug(" > -1 .");
////	else ncDebug(" > error .");
////
////	TxMSG("Tilt +45 Yaw");
////	CAL_STEP_WAIT();
////	if(transform->angle.z > 20) ncDebug(" > +1 .");
////	else if(transform->angle.z < -20) ncDebug(" > -1 .");
////	else ncDebug(" > error .");
////
////	TxMSG("Printing Angle");
////	CAL_STEP_WAIT();
////
////	uint32_t tim = millis();
////	while(millis()<tim+10000){
////		ncDebug("%.2f : %.2f : %.2f",transform->angle.x,transform->angle.y,transform->angle.z);
////	}
////}
////
////void calDebug_mag(void){
////	TxMSG("calDebug_mag");
////
////	delay(200);
////
////	TxMSG("Face towards NORTH");
////
////	CAL_STEP_WAIT();
////
////	delay(200);
////
////	TxMSG("Rotate Around X axis");
////	uint8_t rotcount = 0;
////	float lastrot = 0;
////
////	float minx = 0;
////	float maxx = 0;
////	float miny = 0;
////	float maxy = 0;
////	float minz = 0;
////	float maxz = 0;
////
////	while(rotcount < 36){
////		if(gcs->magvector.x<miny) miny = gcs->magvector.y;
////		if(gcs->magvector.x>maxy) maxy = gcs->magvector.y;
////
////		if(gcs->magvector.z<minz) minz = gcs->magvector.z;
////		if(gcs->magvector.z>maxz) maxz = gcs->magvector.z;
////
////		if(fabsf(lastrot - transform->angle.x) > 20) {
////			lastrot = transform->angle.x;
////			rotcount++;
////			TxMSG(".");
////		}
////		delay(4);
////	}
////
////	TxMSG("Face towards EAST");
////	CAL_STEP_WAIT();
////
////	TxMSG("Rotate Around Y axis");
////	rotcount = 0;
////	lastrot = 0;
////
////	while(rotcount < 36){
////		if(gcs->magvector.x<minx) minx = gcs->magvector.x;
////		if(gcs->magvector.x>maxx) maxx = gcs->magvector.x;
////
////		if(gcs->magvector.z<minz) minz = gcs->magvector.z;
////		if(gcs->magvector.z>maxz) maxz = gcs->magvector.z;
////
////		if(fabsf(lastrot - transform->angle.y) > 20) {
////			lastrot = transform->angle.y;
////			rotcount++;
////			TxMSG(".");
////		}
////		delay(1);
////	}
////
////	ncDebug("X: %.2f,%.2f", minx, maxx);
////	ncDebug("Y: %.2f,%.2f", miny, maxy);
////	ncDebug("Z: %.2f,%.2f", minz, maxz);
////
////	delay(50);
////}
////
////void calDebug_motor(){
////	TxMSG("calDebug_motor");
///*
//#define ESC_ITERATION(num)	ESC_Write(&motor[num], 0.2);\
//							for(uint16_t i=0;i<500;i++){delay(2);}\
//							ESC_Write(&motor[num], 0);\
//
//*/
////	CAL_STEP_WAIT();
////	ncDebug("FRONT RIGHT");
////	CAL_STEP_WAIT();
////	ESC_ITERATION(0)
////
////	ncDebug("REAR RIGHT");
////	CAL_STEP_WAIT();
////	ESC_ITERATION(1)
////
////	ncDebug("REAR LEFT");
////	CAL_STEP_WAIT();
////	ESC_ITERATION(2)
////
////	ncDebug("FRONT LEFT");
////	CAL_STEP_WAIT();
////	ESC_ITERATION(3)
////
////	while(1){
////		float motor_power[4];
////		motor_power[0] = (-rc_handle->range.x  + rc_handle->range.y) - 1; // FR
////		motor_power[1] = (rc_handle->range.x   + rc_handle->range.y) - 1; // RR
////		motor_power[2] = (rc_handle->range.x   - rc_handle->range.y) - 1; // RL
////		motor_power[3] = (-rc_handle->range.x  - rc_handle->range.y) - 1; // FL
////		for (uint8_t i = 0; i < 4; i++) {
////			motor_power[i] = clampf32(motor_power[i], 0.0, 1);
////			ESC_Write(&motor[i], motor_power[i]/3);
////		}
////		delay(4);
////	}
////}
