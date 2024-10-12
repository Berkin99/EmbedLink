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

#include "sysconfig.h"
#include "systime.h"

#ifdef BNO055_I2C

#include "rtos.h"
#include "sensor_bno055.h"
#include "bno055.h"
#include "uart.h"
#include "i2c.h"
#include "geoconfig.h"
#include "filter.h"
#include "estimator.h"
#include "nrx.h"
#include "mem.h"
#include "memory.h"

//#define SENS_ROT_CALIBRATE_BNO055
//#define SENS_ROT_BNO055_X     0.0f
//#define SENS_ROT_BNO055_Y     0.0f  /* 4.4, ?,  ? */ /* It tooks 3 days to figure out that magnetometers can be "slightly" tilted in the IC */
//#define SENS_ROT_BNO055_Z     0.0f  /* 0.0  ?,  ? */

/* BNO055 Definitions */

static struct bno055_t bno055;
static struct bno055_mag_offset_t magcal;

s8   BNO055_I2C_bus_read  (u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8   BNO055_I2C_bus_write (u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek    (u32 msek);

/* Sensor Definitions */

#define CAL_INTERVAL_MS 		45000
#define SENS_ORIENTATION_Z		90

static measurement_t mmag;
static uint8_t       newmag;

static vec_t magBias;
static vec_t magScale;
static vec_t magRot;

static uint32_t idptr;
static uint8_t  isInit;
static uint8_t  isReady;

taskAllocateStatic(BNO055, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskBNO055(void* argv);

int8_t sensorInitBNO055(void){

	if(isInit) return E_OVERWRITE;

    bno055.bus_write  = BNO055_I2C_bus_write;
    bno055.bus_read   = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr   = BNO055_I2C_ADDR1;

    if (bno055_init(&bno055) != BNO055_SUCCESS) return E_CONF_FAIL;

    /* set the power mode as NORMAL*/
    bno055_set_power_mode     (BNO055_POWER_MODE_NORMAL);
    delay(100);
    bno055_set_mag_operation_mode(BNO055_MAG_OPERATION_MODE_HIGH_ACCURACY);
    bno055_set_operation_mode (BNO055_OPERATION_MODE_MAGONLY);

    taskCreateStatic(BNO055, sensorTaskBNO055, NULL);
    isInit = 1;

    return OK;
}

int8_t sensorTestBNO055(void){
	struct bno055_mag_t tmp;
	if(bno055_read_mag_xyz(&tmp) == BNO055_SUCCESS) return OK;
	return E_COMM_FAIL;
}

void sensorTaskBNO055(void* argv){

    /* mmag state */
    mmag.type = STATE_MAGNETIZATION;
    mmag.magnetization.stdDev = vrepeat(1.0f);

    delay(1000);

    if(sensorIsCalibratedBNO055()){
		/* Read Memory and set calibration profile */

		#ifdef SENS_ROT_CALIBRATE_BNO055
    	magRot   = mkvec(SENS_ROT_BNO055_X, SENS_ROT_BNO055_Y, SENS_ROT_BNO055_Z);
    	memoryMemUpload(sensorNameBNO055, "mr.x");
    	memoryMemUpload(sensorNameBNO055, "mr.y");
    	memoryMemUpload(sensorNameBNO055, "mr.z");
		#endif

    	bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
		bno055_write_mag_offset(&magcal);
	    bno055_set_mag_operation_mode(BNO055_MAG_OPERATION_MODE_HIGH_ACCURACY);
		bno055_set_operation_mode(BNO055_OPERATION_MODE_MAGONLY);
    }
    else{
    	magScale = vrepeat(1.0f);
    	magBias  = vzero();
    	magRot   = vzero();
    	serialPrint("[!] BNO055 is not calibrated\n");
    	isReady = 1;
    	delay(RTOS_MAX_DELAY);
    }

	TickType_t xLastWakeTime = xTaskGetTickCount();
    isReady = 1;

    while(1){
    	struct bno055_mag_float_t magfloat;
    	vec_t mheading;
    	if(bno055_convert_float_mag_xyz_uT(&magfloat) == BNO055_SUCCESS){

    		mheading.x = (magfloat.y - magBias.y) * magScale.y * -1;
    		mheading.y = (magfloat.x - magBias.x) * magScale.x;
    		mheading.z = (magfloat.z - magBias.z) * magScale.z;

    		mmag.magnetization.vector = kinematicsRotateFrame(mheading, magRot);

			newmag = 1;
        	estimatorEnqueue(&mmag, 0);
    	}

    	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / sensorFreqBNO055));
    }
}

void sensorCalibrateBNO055(void){

	vec_t mmax = vzero();
	vec_t mmin = vzero();

	uint32_t timeout = millis() + CAL_INTERVAL_MS;

	while(millis() < timeout){
		struct bno055_mag_float_t magfloat;
    	if(bno055_convert_float_mag_xyz_uT(&magfloat) == BNO055_SUCCESS){
			if(magfloat.x < mmin.x) mmin.x = magfloat.x;
			if(magfloat.x > mmax.x) mmax.x = magfloat.x;
			if(magfloat.y < mmin.y) mmin.y = magfloat.y;
			if(magfloat.y > mmax.y) mmax.y = magfloat.y;
			if(magfloat.z < mmin.z) mmin.z = magfloat.z;
			if(magfloat.z > mmax.z) mmax.z = magfloat.z;
    	}
    	delay(50);
	}

	magBias  = vdiv(vadd(mmax, mmin), 2.0f);
	magScale = veltdiv(vrepeat(50.0f), vsub(mmax, magBias));

	bno055_set_operation_mode (BNO055_OPERATION_MODE_CONFIG);
	bno055_read_mag_offset    (&magcal);
    bno055_set_mag_operation_mode(BNO055_MAG_OPERATION_MODE_HIGH_ACCURACY);
    bno055_set_operation_mode (BNO055_OPERATION_MODE_MAGONLY);

	idptr = (uint32_t)SYS_ID  (sensorNameBNO055);
	/* Set Calibration Profile to Memory*/
	memoryMemUpload(sensorNameBNO055, NULL);
}

int8_t sensorIsCalibratedBNO055(void){
	if(idptr == (uint32_t)SYS_ID(sensorNameBNO055))return TRUE;
	return FALSE;
}

int8_t sensorAcquireBNO055(measurement_t* plist, uint8_t n){
	uint8_t i = 0;
	if(newmag && n < i){
		newmag = 0;
		plist[i] = mmag;
		i++;
	}
	return i;
}

int8_t sensorIsReadyBNO055(void){
	return isReady;
}

void sensorWaitDataReadyBNO055(void){
	while(!newmag) delay(1);
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){

    uint8_t rslt;
    rslt  =  i2cTransmit(&BNO055_I2C, dev_addr, &reg_addr, 1);
    rslt |=  i2cReceive(&BNO055_I2C, dev_addr, reg_data, cnt);

    return (rslt == HAL_OK ? BNO055_SUCCESS : BNO055_ERROR);
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){

    uint8_t rslt;
    rslt = i2cMemWrite(&BNO055_I2C, dev_addr, reg_addr, reg_data, cnt);
    return (rslt == HAL_OK ? BNO055_SUCCESS : BNO055_ERROR);

}

void BNO055_delay_msek(u32 msek){
	delay(msek);
}

NRX_GROUP_START(bnoraw)
NRX_ADD(NRX_FLOAT, x, &mmag.kinv.x)
NRX_ADD(NRX_FLOAT, y, &mmag.kinv.y)
NRX_ADD(NRX_FLOAT, z, &mmag.kinv.z)
NRX_GROUP_STOP(bnoraw)

NRX_GROUP_START(bnorot)
NRX_ADD(NRX_FLOAT, x, &magRot.x)
NRX_ADD(NRX_FLOAT, y, &magRot.y)
NRX_ADD(NRX_FLOAT, z, &magRot.z)
NRX_GROUP_STOP(bnorot)

NRX_GROUP_START(bnoscl)
NRX_ADD(NRX_FLOAT, x, &magScale.x)
NRX_ADD(NRX_FLOAT, y, &magScale.y)
NRX_ADD(NRX_FLOAT, z, &magScale.z)
NRX_GROUP_STOP(bnoscl)

NRX_GROUP_START(bnobias)
NRX_ADD(NRX_FLOAT, x, &magBias.x)
NRX_ADD(NRX_FLOAT, y, &magBias.y)
NRX_ADD(NRX_FLOAT, z, &magBias.z)
NRX_GROUP_STOP(bnobiaz)

MEM_GROUP_START(BNO055)
MEM_ADD(MEM_UINT32, idptr, &idptr)
MEM_ADD(MEM_INT16, mx,   &magcal.x)
MEM_ADD(MEM_INT16, my,   &magcal.y)
MEM_ADD(MEM_INT16, mz,   &magcal.z)
MEM_ADD(MEM_INT16, mr,   &magcal.r)
MEM_ADD(MEM_FLOAT, ms.x, &magScale.x)
MEM_ADD(MEM_FLOAT, ms.y, &magScale.y)
MEM_ADD(MEM_FLOAT, ms.z, &magScale.z)
MEM_ADD(MEM_FLOAT, mb.x, &magBias.x)
MEM_ADD(MEM_FLOAT, mb.y, &magBias.y)
MEM_ADD(MEM_FLOAT, mb.z, &magBias.z)
MEM_ADD(MEM_FLOAT, mr.x, &magRot.x)
MEM_ADD(MEM_FLOAT, mr.y, &magRot.y)
MEM_ADD(MEM_FLOAT, mr.z, &magRot.z)
MEM_GROUP_STOP(BNO055)

#endif
