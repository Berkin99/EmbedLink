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

#include "system.h"
#include "sysconfig.h"
#include "systime.h"

#ifdef MPU6500_SPI

#include "rtos.h"
#include "semphr.h"
#include "task.h"
#include "sensor_mpu6500.h"
#include "MPU6500.h"
#include "spi.h"
#include "nrx.h"
#include "filter.h"
#include "estimator.h"
#include "led.h"
#include "uart.h"

#define CAL_INTERVAL_MS 	3000
#define INTERFACE_TYPE 		MPU6500_SPI_INTF

static struct MPU6500_Device self;
int8_t _sensorReadMPU6500(uint8_t reg_addr, uint8_t *read_data, uint8_t len);
int8_t _sensorWriteMPU6500(uint8_t reg_addr, uint8_t *write_data, uint8_t len);

static uint8_t isInit  = 0;
static uint8_t isReady = 0;
static measurement_t macc;
static measurement_t mgyr;
static uint8_t newacc = 0;
static uint8_t newgyr = 0;
static int8_t  status = 0;

taskAllocateStatic(MPU6500,SENS_TASK_STACK,SENS_TASK_PRI);
void sensorTaskMPU6500(void* argv);

int8_t sensorInitMPU6500(void){

	if(isInit) return SYS_E_OVERWRITE;

	/* MPU6500 Object set */
	self.intf = INTERFACE_TYPE;
	self.delay_us = &delayMicroseconds;
	self.read = &_sensorReadMPU6500;
	self.write = &_sensorWriteMPU6500;
	self.chip_id = 0;

	/* Library init */
	status = MPU6500_Init(&self); /* @WARNING initialization clears object calibration data */

	if(status == 0) return SYS_E_COMM_FAIL; /* Library Error */

	/* ( eerpom search for calibration values ) */

	taskCreateStatic(MPU6500, sensorTaskMPU6500, NULL);
	delay(10);

	if(!isInit) return SYS_ERR;
	return SYS_OK; 	/* Return OK */
}

int8_t sensorTestMPU6500(void){
	if (MPU6500_Test(&self)){
		return SYS_OK;
	}
	return SYS_E_COMM_FAIL;
}

void sensorTaskMPU6500(void* argv){
	isInit = 1;

	/* Buffers */
	static int16_t accRaw[3];
	static int16_t gyrRaw[3];
	static lpf2pData accFilter[3];
	static lpf2pData gyrFilter[3];
	static float acc_temp[3];
	static float gyr_temp[3];

	/* Estimator Measurement */
	macc.type = STATE_INTERNAL_ACCELERATION;
	mgyr.type = STATE_INTERNAL_ATTITUDE;

	/* Low Pass Filter */
	for(uint8_t i =0;i<3;i++){
		lpf2pInit(&accFilter[i], sensorFreqMPU6500, 30);
		lpf2pInit(&gyrFilter[i], sensorFreqMPU6500, 80);
	}

	/* Lib settings */
	MPU6500_SetAccelRange(&self, ACCEL_RANGE_8G);
	MPU6500_SetGyroRange(&self, GYRO_RANGE_1000DPS);
	MPU6500_GyrCalibration(&self, CAL_INTERVAL_MS);


	///////////////////////////////////////////////////////////////////////////////////////// MEMORY DATA
	//MPU6500_AccCalibration(&self,CAL_INTERVAL_MS);

	// XAXIS : -4300,-130, 3870 : -4170, 0, 4000
	// YAXIS : -4120,-100, 4000 : -4020, 0, 4100
	// ZAXIS : -3880, 400, 4280 : -4280 ,0, 3880

	self.calib_data.acc_cal[0] = -130;  /* X cal parallel surface */
	self.calib_data.acc_cal[1] = -103;  /* Y cal parallel surface */
	self.calib_data.acc_cal[2] =  400; 	/* Z Calibration */

	self.calib_data.acc_scale[0][0] = 0.959f;
	self.calib_data.acc_scale[0][1] = 1.000f;
	self.calib_data.acc_scale[1][0] = 0.995f;
	self.calib_data.acc_scale[1][1] = 0.975f;
	self.calib_data.acc_scale[2][0] = 0.934f;
	self.calib_data.acc_scale[2][1] = 1.030f;

	macc.acceleration.stdDev = vrepeat(0.1);
	mgyr.attitude.stdDev     = vrepeat(0.1);

	///////////////////////////////////////////////////////////////////////////////////////// MEMORY DATA

	isReady = SYS_TRUE;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1){

		status = MPU6500_GetDataRaw(&self, accRaw, gyrRaw);

		if(status == 1){
			for (uint8_t i = 0; i < 3; i++){
				acc_temp[i] = (float) (accRaw[i] - self.calib_data.acc_cal[i]) * self.calib_data.acc_coefficient;

				if(acc_temp[i] < 0) acc_temp[i] *= self.calib_data.acc_scale[i][0];
				else acc_temp[i] *= self.calib_data.acc_scale[i][1];

				gyr_temp[i] = (float) (gyrRaw[i] - self.calib_data.gyr_cal[i]) * self.calib_data.gyr_coefficient;
			}

			for (uint8_t i = 0; i < 3; i++) {
				macc.acceleration.axis[i] = lpf2pApply(&accFilter[i], acc_temp[i]);
				mgyr.attitude.axis[i] = lpf2pApply(&gyrFilter[i], gyr_temp[i]);
			}

			newacc = 1;
			newgyr = 1;

			estimatorEnqueue(&macc, SYS_FALSE);
			estimatorEnqueue(&mgyr, SYS_FALSE);
		}

		/*Wait to complete 1000us*/
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}

void sensorCalibrateMPU6500(void){
	MPU6500_AccCalibration(&self, CAL_INTERVAL_MS);
}

int8_t sensorIsCalibratedMPU6500(void){
	/* TODO: ACC & GYR Calibration Check */
	return SYS_TRUE;
}

int8_t sensorAcquireMPU6500(measurement_t* plist, uint8_t n){

	if(newacc){
		pData = &macc;
		newacc = 0;
		return SYS_TRUE;
	}

	if(newgyr){
		pData = &mgyr;
		newgyr = 0;
		return SYS_TRUE;
	}

	return SYS_FALSE;
}

int8_t sensorIsReadyMPU6500(void){
	return isReady;
}

void sensorWaitDataReadyMPU6500(void){
	while(!newacc){ vTaskDelay(1); }
}

int8_t _sensorReadMPU6500(uint8_t reg_addr, uint8_t *read_data, uint8_t len){

	HAL_StatusTypeDef status;

	/*INFO : Do not write gpio before
	 * get the semaphore from the spiBeginTransaction()
	 * */

	spiBeginTransaction(&MPU6500_SPI);
	pinWrite(MPU6500_CS, LOW);
	spiTransmit(&MPU6500_SPI, &reg_addr, 1);
	status = spiReceive(&MPU6500_SPI, read_data, len);
	pinWrite(MPU6500_CS, HIGH);
	spiEndTransaction(&MPU6500_SPI);

	return (status == HAL_OK);
}

int8_t _sensorWriteMPU6500(uint8_t reg_addr, uint8_t *write_data, uint8_t len){
	HAL_StatusTypeDef status;

	spiBeginTransaction(&MPU6500_SPI);
	HAL_GPIO_WritePin(MPU6500_GPIO, MPU6500_CS, GPIO_PIN_RESET);

	spiTransmit(&MPU6500_SPI, &reg_addr, 1);
	status = spiTransmit(&MPU6500_SPI, write_data, len);

	HAL_GPIO_WritePin(MPU6500_GPIO, MPU6500_CS, GPIO_PIN_SET);
	spiEndTransaction(&MPU6500_SPI);

	return (status == HAL_OK); /* TRUE = 1 */
}

#endif
