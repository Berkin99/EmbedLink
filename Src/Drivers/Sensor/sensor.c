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
#include "sysconfig.h"
#include "systime.h"
#include "uart.h"
#include "sensor.h"

#ifdef MPU6500_SPI
#include "sensor_mpu6500.h"
#endif
#ifdef BMI088_SPI
#include "sensor_bmi088.h"
#endif
#ifdef BMP581_I2C
#include "sensor_bmp581.h"
#endif
#ifdef BNO055_I2C
#include "sensor_bno055.h"
#endif
#ifdef BMP388_I2C
#include "sensor_bmp388.h"
#endif
#ifdef HMC5883L_I2C
#include "sensor_hmc5883l.h"
#endif

#define SENS_ADD(SENSOR) {\
	.Name = sensorName##SENSOR,\
	.Init = &sensorInit##SENSOR,\
	.Test = &sensorTest##SENSOR,\
	.Calibrate = &sensorCalibrate##SENSOR,\
	.IsCalibrated = &sensorIsCalibrated##SENSOR,\
	.Acquire = &sensorAcquire##SENSOR,\
	.IsReady = &sensorIsReady##SENSOR,\
	.WaitDataReady = &sensorWaitDataReady##SENSOR,\
},\

static const sensor_t sensorList[] = {
	#ifdef MPU6500_SPI
	SENS_ADD(MPU6500)
	#endif
	#ifdef BMI088_SPI
	SENS_ADD(BMI088)
	#endif
	#ifdef BMP581_I2C
	SENS_ADD(BMP581)
	#endif
	#ifdef BNO055_I2C
	SENS_ADD(BNO055)
	#endif
	#ifdef BMP388_I2C
	SENS_ADD(BMP388)
	#endif
	#ifdef HMC5883L_I2C
	SENS_ADD(HMC5883L)
	#endif
};

static const uint8_t sensorLen = sizeof(sensorList)/sizeof(sensor_t);

static uint8_t qReady;
static queue_t qSense;
queueAllocateStatic(qSense, SENSE_QUEUE_SIZE, sizeof(sense_t))

void sensorInit(void){

	/* Queue Create */
	qSense = queueCreateStatic(qSense);
	qReady = 1;

	for(uint8_t i = 0; i < sensorLen; i++){
		if(sensorList[i].Init() == OK) serialPrint("[+] Sensor %s init OK\n",sensorList[i].Name);
		else serialPrint("[-] Sensor %s init ERROR\n",sensorList[i].Name);
	}
}

void sensorTest(void){
	for(uint8_t i = 0; i < sensorLen; i++){
		if(sensorList[i].Test() == OK) serialPrint("[+] Sensor %s test OK\n",sensorList[i].Name);
		else serialPrint("[-] Sensor %s test ERROR\n",sensorList[i].Name);
	}
}

int8_t sensorIsReady(void){
	for(uint8_t i = 0; i < sensorLen; i++){if(sensorList[i].IsReady() != TRUE) return FALSE;}
	return TRUE;
}

uint8_t sensorSize(void){
	return sensorLen;
}

int8_t sensorGet(const char* name, sensor_t** psensor){
	for(uint8_t i = 0; i < sensorLen; i++){
		if(strcmp(sensorList[i].Name,name) == 0){
			*psensor = &(((sensor_t*)sensorList)[i]);
			return OK;
		}
	}
	return E_NOT_FOUND;
}

int8_t sensorEnqueue(const sense_t* pSense, int8_t isISR){
	if(!qReady) return ERROR;
	int8_t result;
    if (isISR) result = queueSendISR(qSense, pSense);
    else result = queueSend(qSense, pSense, 0);
    return result;
}

int8_t sensorDequeue(sense_t* pSense, uint32_t portDelay){
	if(!qReady) return ERROR;

	if (queueReceive(qSense, pSense, portDelay) == RTOS_TRUE) return TRUE;

	return FALSE;
}
