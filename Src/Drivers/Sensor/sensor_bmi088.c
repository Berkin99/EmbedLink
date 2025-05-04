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

#include <sysdefs.h>
#include <sysconfig.h>
#include <systime.h>
#include <rtos.h>

#ifdef   BMI088_SPI

#include "xmath.h"
#include "sensor_bmi088.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "bmi08_defs.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "geoconfig.h"
#include "filter.h"
#include "estimator.h"
#include "mem.h"
#include "memory.h"
#include "nrx.h"

/* BMI088 Settings */
#define BMI088_ACC_FS_CFG           (24)
#define BMI088_GYR_FS_CFG           (2000)
#define BMI088_DEG_PER_LSB          ((2.0f * BMI088_GYR_FS_CFG) / 65536.0f)
#define BMI088_G_PER_LSB            (((float)BMI088_ACC_FS_CFG) * 2.0f / 65536.0f)
#define BMI088_1G_IN_LSB            (65536.0f / BMI088_ACC_FS_CFG / 2.0f)
#define BMI088_CAL_INTERVAL_MS      (3000)
#define BMI088_ACC_LPF_CUTOFF_FREQ  (30)
#define BMI088_GYR_LPF_CUTOFF_FREQ  (80)
#define BMI088_SENS_ORIENTATION_Z   (90)

static pin_t bmi08AccCsPin = BMI088_ACC_CS;
static pin_t bmi08GyrCsPin = BMI088_GYR_CS;
BMI08_INTF_RET_TYPE bmi08SpiRead (uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi08SpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void                bmi08DelayUs (uint32_t period, void *intf_ptr);

struct bmi08_dev bmi08dev = {
    .intf_ptr_accel = (void*)&bmi08AccCsPin,
    .intf_ptr_gyro  = (void*)&bmi08GyrCsPin,
    .write          = bmi08SpiWrite,
    .read           = bmi08SpiRead,
    .intf           = BMI08_SPI_INTF,
    .delay_us       = bmi08DelayUs,
    .read_write_len = 32,
    .variant        = BMI088_VARIANT,
    .accel_cfg = {
        .power = BMI08_ACCEL_PM_ACTIVE,
        .range = BMI088_ACCEL_RANGE_24G,
        .bw    = BMI08_ACCEL_BW_OSR4,
        .odr   = BMI08_ACCEL_ODR_1600_HZ,
    },
    .gyro_cfg = {
        .power = BMI08_GYRO_PM_NORMAL,
        .range = BMI08_GYRO_RANGE_2000_DPS,
        .bw    = BMI08_GYRO_BW_116_ODR_1000_HZ,
        .odr   = BMI08_GYRO_BW_116_ODR_1000_HZ,
    },
};

/* SENSOR_DATA */
typedef struct {
    vec_t      data;
    vec_t      mean;
    vec_t      scale;
    vec_t      rot;
    lpf2pData  lpfdata[3];
}sensor_data_bmi088_t;

static sensor_data_bmi088_t accv, gyrv;
static sense_t accs, gyrs;

static uint32_t  idptr;
static uint8_t   isInit;
static int8_t    isReady;

taskAllocateStatic(BMI088, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskBMI088 (void* argv);

uint8_t _sensorAccData(vec_t* data);
uint8_t _sensorGyrData(vec_t* data);
void    _sensorMeanBMI088     (vec_t* accMean, vec_t* gyrMean);
void    _sensorVarianceBMI088 (vec_t accMean, vec_t gyrMean, vec_t * accVar, vec_t* gyrVar);

int8_t sensorInitBMI088(void){
    if(isInit) return E_OVERWRITE;

    uint8_t rslt;
 
    pinWrite(bmi08AccCsPin, HIGH);
    pinWrite(bmi08GyrCsPin, HIGH);

    delay(100);

    /* INITIALIZE BMI088 */
    bmi08a_set_power_mode(&bmi08dev);     /* ACCEL ACTIVATE */
    if(bmi08a_init(&bmi08dev) != BMI08_OK ){ serialPrint("[-] BMI088 ACC init err\n"); return ERROR; }
    bmi08a_set_meas_conf(&bmi08dev);     /* ACCEL SET CONFIGS */
    if(_sensorAccData(&accv.data) != OK) serialPrint("[-] BMI088 ACC config err\n");     /* ACCEL TEST */
    bmi08g_set_power_mode(&bmi08dev);     /* GYRO ACTIVATE */
    if( bmi08g_init(&bmi08dev) != BMI08_OK ){ serialPrint("[-] BMI088 GYR init err\n"); return ERROR; }
    bmi08g_set_meas_conf(&bmi08dev);     /* GYRO SET CONFIGS */
    if(_sensorGyrData(&gyrv.data) != OK) serialPrint("[-] BMI088 GYR config err\n");     /* GYRO TEST */

    /* LPF INITIALIZE */
    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accv.lpfdata[i], sensorFreqBMI088, BMI088_ACC_LPF_CUTOFF_FREQ);
        lpf2pInit(&gyrv.lpfdata[i], sensorFreqBMI088, BMI088_GYR_LPF_CUTOFF_FREQ);
    }

    /* TASK CREATE */
    taskCreateStatic(BMI088, sensorTaskBMI088, NULL);
    isInit = 1;
    return OK;
}

int8_t sensorTestBMI088(void){
    bmi08a_get_regs(BMI08_REG_ACCEL_CHIP_ID, &bmi08dev.accel_chip_id, 1, &bmi08dev);
    bmi08g_get_regs(BMI08_REG_GYRO_CHIP_ID,  &bmi08dev.gyro_chip_id,  1, &bmi08dev);

    if ((bmi08dev.gyro_chip_id == BMI08_GYRO_CHIP_ID) && (bmi08dev.accel_chip_id == BMI088_ACCEL_CHIP_ID)){
        return OK;
    }
    return ERROR;
}

void sensorTaskBMI088(void* argv){

    delay(500);

    accs.type = SENSE_IACCELERATION;
    gyrs.type = SENSE_IATTITUDE;
    
    _sensorMeanBMI088(&accv.mean, &gyrv.mean);

    /* Calculated STDDEV */
    accs.xvec.stdDev = vrepeat(0.03f);
    gyrs.xvec.stdDev = vrepeat(0.001f);

    if(!sensorIsCalibratedBMI088()){
        serialPrint("[!] BMI088 is not calibrated\n");
        accv.rot   = vzero();
        accv.scale = vrepeat(1.0f);
    }

    uint32_t xLastWakeTime = taskGetTickCount();

    isReady = 1;

    while(1){
        if(_sensorAccData(&accv.data) == OK){
            vec_t va = veltmul(kinematicsRotateFrame(accv.data, accv.rot), accv.scale);

            for (uint8_t i = 0; i < 3; ++i) {
                accs.xvec.axis[i] = lpf2pApply(&accv.lpfdata[i], va.axis[i]);
            }
            accs.xvec.timestampMs = millis();
            estimatorEnqueue(&accs, 0);
        }
        if(_sensorGyrData(&gyrv.data) == OK){
            vec_t vg = vsub(gyrv.data, gyrv.mean);
            
            for (uint8_t i = 0; i < 3; i++) {
                gyrv.xvec.axis[i] = lpf2pApply(&gyrv.lpfdata[i], vg.axis[i]);
            }
            gyrs.xvec.timestampMs = millis();
            estimatorEnqueue(&gyrs, 0);
        }
        taskDelayUntil(&xLastWakeTime, (1000 / sensorFreqBMI088));
    }
}

void sensorCalibrateBMI088(void){
	vec_t tempaccx = vzero();
	vec_t tempaccy = vzero();
	vec_t tempaccz = vzero();
	vec_t tempgyr  = vzero();

	accv.rot = xkinematicsState()->rotation.v;

	_sensorMeanBMI088(&tempaccz, &tempgyr);
	accv.scale.z = GRAVITY / tempaccz.z;

	float loopx = 0;
	float loopy = 0;

	while(loopy < 1000){
		if(kinematicsState()->rotation.x > 85 && kinematicsState()->rotation.x < 95){
			tempaccy = vmean(tempaccy, accv.data, loopy);
			loopy++;
		}
		delay(4);
	}

	while(kinematicsState()->rotation.x > 5) delay(4);

	while(loopx < 1000){
		if(kinematicsState()->rotation.y < -85 && kinematicsState()->rotation.y > -95){
			tempaccx = vmean(tempaccx, accData, loopx);
			loopx++;
		}
		delay(4);
	}

	accv.scale.y = GRAVITY / tempaccy.y;
	accv.scale.x = GRAVITY / tempaccx.x;

	idptr = (uint32_t)SYS_ID(sensorNameBMI088);
    memoryMemUpload(sensorNameBMI088, NULL);
}

void _sensorMeanBMI088(vec_t* accMean, vec_t* gyrMean){
    float i, j;
    vec_t tempacc, tempgyr;
    
    i = 0;
    j = 0;
    uint32_t intervalMs = millis() + BMI088_CAL_INTERVAL_MS;

    while(millis() < intervalMs){
        if(_sensorAccData(&tempacc) == OK){
            *accMean = vmean(*accMean, tempacc, i);
            i++;
        }
        if(_sensorGyrData(&tempgyr) == OK){
            *gyrMean = vmean(*gyrMean, tempgyr, j);
            j++;
        }
        delay(2);
    }
}

void _sensorVarianceBMI088(vec_t accMean, vec_t gyrMean, vec_t * accVar, vec_t* gyrVar){
    float i = 0;
    float j = 0;
    uint32_t intervalMs = millis() + BMI088_CAL_INTERVAL_MS;
    *accVar = vzero();
    *gyrVar = vzero();

    while(millis() < intervalMs){
        if(_sensorAccData(&accv.data) == OK){
            *accVar = vsigma2(*accVar, accv.mean, accv.data, i);
            i++;
        }
        if(_sensorGyrData(&gyrv.data) == OK){
            *gyrVar = vsigma2(*gyrVar, gyrv.mean, gyrv.data, j);
            j++;
        }
        delay(2);
    }
}

int8_t sensorIsCalibratedBMI088(void){
    /* Check the memory */
    if(idptr == SYS_ID(sensorNameBMI088)) return TRUE;
    return FALSE;
}

int8_t sensorAcquireBMI088(sense_t* plist, uint8_t n){
    return E_ERROR;
}

int8_t sensorIsReadyBMI088(void){
    return isReady;
}

void sensorWaitDataReadyBMI088(void){while()}

uint8_t _sensorAccData(vec_t* data){
    /* Output as G values ((9,81m)/s^2)*/
    struct bmi08_sensor_data bmi08AccData;
    int8_t rslt = bmi08a_get_data(&bmi08AccData, &bmi08dev);
    *data = vscl(mkvec(bmi08AccData.y, bmi08AccData.x * -1, bmi08AccData.z), BMI088_G_PER_LSB * GRAVITY);
    return (rslt == BMI08_OK) ? OK : E_ERROR;
}

uint8_t _sensorGyrData(vec_t* data){
    /* Output as deg/s values (deg/s)*/
    struct bmi08_sensor_data bmi08GyrData;
    int8_t rslt = bmi08g_get_data(&bmi08GyrData, &bmi08dev);
    *data = vscl(mkvec(bmi08GyrData.y, bmi08GyrData.x * -1, bmi08GyrData.z), BMI088_DEG_PER_LSB);
    return (rslt == BMI08_OK) ? OK : E_ERROR;
}

BMI08_INTF_RET_TYPE bmi08SpiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
    spiBeginTransaction(&BMI088_SPI);
    pinWrite(*(pin_t*)intf_ptr), LOW);
    spiTransmit(&BMI088_SPI, &reg_addr, 1);
    int8_t result = spiReceive(&BMI088_SPI, reg_data, (uint16_t)len);
    pinWrite(*(pin_t*)intf_ptr, HIGH);
    spiEndTransaction(&BMI088_SPI);
    return result;
}

BMI08_INTF_RET_TYPE bmi08SpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
    spiBeginTransaction(&BMI088_SPI);
    pinWrite(*(pin_t*)intf_ptr, LOW);
    spiTransmit(&BMI088_SPI, &reg_addr, 1);
    int8_t result = spiTransmit(&BMI088_SPI, (uint8_t*)reg_data, len);
    pinWrite(*(pin_t*)intf_ptr, HIGH);
    spiEndTransaction(&BMI088_SPI);
    return result;
}

void bmi08DelayUs(uint32_t period, void *intf_ptr){
    (void) intf_ptr;
    delayUs(period);
}

NRX_GROUP_START(bmical)
NRX_ADD(NRX_FLOAT,  accScale.x, &accScale.x)
NRX_ADD(NRX_FLOAT,  accScale.y, &accScale.y)
NRX_ADD(NRX_FLOAT,  accScale.z, &accScale.z)
NRX_ADD(NRX_FLOAT,  accRot.x,   &accRot.x)
NRX_ADD(NRX_FLOAT,  accRot.y,   &accRot.y)
NRX_ADD(NRX_FLOAT,  accRot.z,   &accRot.z)
NRX_GROUP_STOP (bmical)

MEM_GROUP_START(BMI088)
MEM_ADD(MEM_UINT32, idptr,    &idptr)
MEM_ADD(MEM_FLOAT,  accScale.x, &accScale.x)
MEM_ADD(MEM_FLOAT,  accScale.y, &accScale.y)
MEM_ADD(MEM_FLOAT,  accScale.z, &accScale.z)
MEM_ADD(MEM_FLOAT,  accRot.x,  &accRot.x)
MEM_ADD(MEM_FLOAT,  accRot.y,  &accRot.y)
MEM_ADD(MEM_FLOAT,  accRot.z,  &accRot.z)
MEM_GROUP_STOP (BMI088)

#endif /* BMI088_SPI */
