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

#include <sysconfig.h>
#include <system.h>
#include <systime.h>

#ifdef HMC5883L_I2C
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "sensor_hmc5883l.h"
#include "HMC5883L.h"
#include "nrx.h"

/* Data Buffer */
static float  mag_data[3];

/* Calibration Default Values */
static int16_t mag_cal[6] 	 = {-619, 648, -338, 613, -443, 651};	/* xmin, xmax, ymin, ymax, zmin, zmax */
static float   mag_offset[3] = {0, 0, 0};
static float   mag_scale [3] = {1.0f, -1.0f, -1.0f};	/* Rotation Axis */

static uint8_t is_init  = 0;
static uint8_t is_ready = 0;
static uint8_t new_data = 0;
static uint8_t is_calibrated = 0;

STATIC_MEM_TASK_ALLOC(HMC5883L,SENSOR_TASK_STACK,SENSOR_TASK_PRI);
void _sensorTaskHMC5883L(void* argv);

uint8_t sensorInitHMC5883L(void){

	if(is_init) return 0;
	HMC5883L_Init(&HMC5883L_I2C);
	uint8_t result = HMC5883L_testConnection();
	if(result == 1){
		is_init = 1;
		STATIC_MEM_TASK_CREATE(HMC5883L, _sensorTaskHMC5883L,NULL);
		return 1;
	}
	return 0;
}

uint8_t sensor_test_HMC5883L(void){
	return HMC5883L_testConnection();
}

static int16_t heading[3]; /* heading buffer */

void _sensorTaskHMC5883L(void* argv){

	/* Get Calibration Values From Memory */

	//////////////// CALIBRATION VALUE
	//memcpy(mag_cal, memory, 6);
	is_calibrated = 0;
	//////////////// CALIBRATION VALUE

	/* y and z axis reference is x, so x scale is 1 */
	if(is_calibrated){
		mag_scale[0] *= 1.0f;
		mag_scale[1] *= (float)(mag_cal[1] - mag_cal[0]) / (mag_cal[3] - mag_cal[2]);
		mag_scale[2] *= (float)(mag_cal[1] - mag_cal[0]) / (mag_cal[5] - mag_cal[4]);

		mag_offset[0] =  (float)(mag_cal[1] - mag_cal[0]) / 2 - mag_cal[1];
		mag_offset[1] = ((float)(mag_cal[3] - mag_cal[2]) / 2 - mag_cal[3]) * mag_scale[1];
		mag_offset[2] = ((float)(mag_cal[5] - mag_cal[4]) / 2 - mag_cal[5]) * mag_scale[2];
	}

	is_ready = 1;

	while(1){

		HMC5883L_getHeading(&heading[0], &heading[1], &heading[2]);

		for (uint8_t i = 0; i < 3; ++i) {mag_data[i] = ((float)heading[i] + mag_offset[i]) * mag_scale[i];}
		new_data = 1;
		vTaskDelay(pdMS_TO_TICKS(1000/sensor_frequency_HMC5883L));
	}
}

void sensorCalibrateHMC5883L(void){
	/* calibration routine */
	static int16_t heading[3]; /* heading buffer */

	/* ledsequence */
	uint32_t capture = millis();
	while(millis() - capture < 10000){
		HMC5883L_getHeading(&heading[0], &heading[1], &heading[2]);

		if(heading[0]<mag_cal[0]) mag_cal[0] = heading[0];
		if(heading[0]>mag_cal[1]) mag_cal[1] = heading[0];
		if(heading[1]<mag_cal[2]) mag_cal[2] = heading[1];
		if(heading[1]>mag_cal[3]) mag_cal[3] = heading[1];
		if(heading[2]<mag_cal[4]) mag_cal[4] = heading[2];
		if(heading[2]>mag_cal[5]) mag_cal[5] = heading[2];

		vTaskDelay(pdMS_TO_TICKS(1000/sensor_frequency_HMC5883L));
	}
}

uint8_t sensorIsCalibratedHMC5883L(void){
	return is_calibrated;
}

uint8_t sensorAcquireHMC5883L(measurement_t* plist, uint8_t n){

}

uint8_t sensorIsReadyHMC5883L(void){
	return is_ready;
}

void sensorWaitDataReadyHMC5883L(void){
	while(!new_data) vTaskDelay(pdMS_TO_TICKS(2));
}

#endif
