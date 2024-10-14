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
#include "rtos.h"
#include "sysconfig.h"

#ifdef   BMI088_SPI

#include "systime.h"
#include "uart.h"
#include "gpio.h"
#include "sensor_bmi088.h"
#include "spi.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "bmi08_defs.h"
#include "geoconfig.h"
#include "filter.h"
#include "estimator.h"
#include "led.h"
#include "ledseq.h"
#include "mem.h"
#include "memory.h"
#include "nrx.h"

/* BMI088 Definitions */

#define BMI088_ACC_FS_CFG   (24)
#define BMI088_ACC_RANGE    BMI088_ACCEL_RANGE_24G
#define BMI088_ACC_ODR      BMI08_ACCEL_ODR_1600_HZ
#define BMI088_ACC_BW       BMI08_ACCEL_BW_OSR4
#define BMI088_GYR_FS_CFG   (2000)
#define BMI088_GYR_RANGE    BMI08_GYRO_RANGE_2000_DPS
#define BMI088_GYR_ODR      BMI08_GYRO_BW_116_ODR_1000_HZ
#define BMI088_DEG_PER_LSB  ((2.0f * BMI088_GYR_FS_CFG) / 65536.0f)
#define BMI088_G_PER_LSB    (((float)BMI088_ACC_FS_CFG) * 2.0f / 65536.0f)
#define BMI088_1G_IN_LSB    (65536.0f / BMI088_ACC_FS_CFG / 2.0f)

static uint8_t bmi08AccIntf = 0;
static uint8_t bmi08GyrIntf = 1;

struct bmi08_dev bmi08dev;
struct bmi08_accel_int_channel_cfg accel_int_config;
struct bmi08_gyro_int_channel_cfg gyro_int_config;
struct bmi08_int_cfg bmi08_int_config;

BMI08_INTF_RET_TYPE bmi08SpiRead (uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi08SpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void                bmi08DelayUs (uint32_t period, void *intf_ptr);

/* Sensor Definitions */

#define BMI088_CAL_INTERVAL_MS      3000
#define BMI088_ACC_LPF_CUTOFF_FREQ  30
#define BMI088_GYR_LPF_CUTOFF_FREQ  80
#define BMI088_SENS_ORIENTATION_Z   90

static vec_t      accData;
static vec_t      accMean;
static vec_t      accScale;
static vec_t      accRot;
static lpf2pData  accLpfData[3];
static uint8_t    accNew;

static vec_t      gyrData;
static vec_t      gyrMean;
static lpf2pData  gyrLpfData[3];
static uint8_t    gyrNew;

static measurement_t macc;
static measurement_t mgyr;

static uint32_t  idptr;
static uint8_t   isInit;
static int8_t    isReady;

taskAllocateStatic(BMI088, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskBMI088 (void* argv);

uint8_t _sensorAccData(vec_t* data);
uint8_t _sensorGyrData(vec_t* data);
void    _sensorMeanBMI088     (vec_t* accMean, vec_t* gyrMean);
void    _sensorVarianceBMI088 (vec_t* accVar, vec_t* gyrVar);

int8_t sensorInitBMI088(void){
    if(isInit) return E_OVERWRITE;

    uint8_t rslt;
    struct bmi08_sensor_data bmi08AccData;
    struct bmi08_sensor_data bmi08GyrData;

    pinWrite(BMI088_ACC_CS, 1);
    pinWrite(BMI088_GYR_CS, 1);

    delay(100);

    /* DEVICE */
    bmi08dev.write          = bmi08SpiWrite;
    bmi08dev.read           = bmi08SpiRead;
    bmi08dev.intf           = BMI08_SPI_INTF;
    bmi08dev.delay_us       = bmi08DelayUs;
    bmi08dev.read_write_len = 32;
    bmi08dev.variant        = BMI088_VARIANT;
    bmi08dev.intf_ptr_accel = &bmi08AccIntf;
    bmi08dev.intf_ptr_gyro  = &bmi08GyrIntf;

    /* ACCEL POWER */
    bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    bmi08a_set_power_mode(&bmi08dev);
    if( bmi08a_init(&bmi08dev) != BMI08_OK ){ serialPrint("[-] BMI088 ACC init err\n"); return ERROR; }

    /* ACCEL CONFIG */
    bmi08dev.accel_cfg.bw    = BMI088_ACC_BW;
    bmi08dev.accel_cfg.range = BMI088_ACC_RANGE;
    bmi08dev.accel_cfg.odr   = BMI088_ACC_ODR;
    bmi08a_set_meas_conf(&bmi08dev);

    /* ACCEL TEST */
    rslt = bmi08a_get_data(&bmi08AccData, &bmi08dev);
    if(rslt != BMI08_OK) serialPrint("[-] BMI088 ACC config err\n");

    /* GYRO POWER */
    bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    bmi08g_set_power_mode(&bmi08dev);
    if( bmi08g_init(&bmi08dev) != BMI08_OK ){ serialPrint("[-] BMI088 GYR init err\n"); return ERROR; }

    /* GYRO CONFIG */
    bmi08dev.gyro_cfg.odr   = BMI088_GYR_ODR;
    bmi08dev.gyro_cfg.range = BMI088_GYR_RANGE;
    bmi08dev.gyro_cfg.bw    = BMI088_GYR_ODR;
    bmi08g_set_meas_conf(&bmi08dev);

    /* GYRO TEST */
    rslt = bmi08g_get_data(&bmi08GyrData, &bmi08dev);
    if(rslt != BMI08_OK) serialPrint("[-] BMI088 GYR config err\n");

//    /* BMI088 INTERRUPT */
//    bmi08_int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_1;
//    bmi08_int_config.accel_int_config_1.int_type = BMI08_ACCEL_INT_DATA_RDY;
//    bmi08_int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
//    bmi08_int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
//    bmi08_int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
//
//    /* Setting the interrupt configuration */
//    rslt = bmi08a_set_int_config(&bmi08_int_config.accel_int_config_1, &bmi08dev);
//    if(rslt!= BMI08_OK) serialPrint("[-] BMI088 INT config err\n");

    /* LPF Initialize */
    for (uint8_t i = 0; i < 3; ++i) {
        lpf2pInit(&gyrLpfData[i], sensorFreqBMI088, BMI088_GYR_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpfData[i], sensorFreqBMI088, BMI088_ACC_LPF_CUTOFF_FREQ);
    }

    /* OK */
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

void   sensorTaskBMI088(void* argv){

    /* Calculated STDDEV */
    macc.type = STATE_INTERNAL_ACCELERATION;
    mgyr.type = STATE_INTERNAL_ATTITUDE;

    delay(500);
    _sensorMeanBMI088(&accMean, &gyrMean);
    macc.kinv.stdDev = vrepeat(0.03f);
    mgyr.kinv.stdDev = vrepeat(0.001f);

    if(!sensorIsCalibratedBMI088()){
        serialPrint("[!] BMI088 is not calibrated\n");
        accRot = vzero();
        accScale = vrepeat(1.0f);
    }

    uint32_t xLastWakeTime = taskGetTickCount();
    isReady = 1;

    while(1){

        if(_sensorAccData(&accData)){
        	accData = kinematicsRotateFrame(accData, accRot);
            accData = veltmul(accData, accScale);

            for (uint8_t i = 0; i < 3; ++i) {
                macc.acceleration.axis[i] = lpf2pApply(&accLpfData[i], accData.axis[i]);
            }
            estimatorEnqueue(&macc, 0);
            accNew = 1;
        }

        if(_sensorGyrData(&gyrData)){
            gyrData = vsub(gyrData, gyrMean);
            for (uint8_t i = 0; i < 3; i++) {
                mgyr.attitude.axis[i] = lpf2pApply(&gyrLpfData[i], gyrData.axis[i]);
            }
            estimatorEnqueue(&mgyr, 0);
            gyrNew = 1;
        }

        /* TODO: IRQ Based waiting needed */
        taskDelayUntil(&xLastWakeTime, (1000 / sensorFreqBMI088));
    }
}

void sensorCalibrateBMI088(void){
	vec_t tempaccx = vzero();
	vec_t tempaccy = vzero();
	vec_t tempaccz = vzero();
	vec_t tempgyr;

	accRot = kinematicsState()->rotation.vector;

	_sensorMeanBMI088(&tempaccz, &tempgyr);
	accScale.z = GRAVITY / tempaccz.z;

	float loopx = 0;
	float loopy = 0;

	while(loopy < 1000){
		if(kinematicsState()->rotation.x > 85 && kinematicsState()->rotation.x < 95){
			tempaccy = vmean(tempaccy, accData, loopy);
			loopy++;
		}
		delay(4);
	}

	while(kinematicsState()->rotation.x > 5 ) delay(4);

	while(loopx < 1000){
		if(kinematicsState()->rotation.y < -85 && kinematicsState()->rotation.y > -95){
			tempaccx = vmean(tempaccx, accData, loopx);
			loopx++;
		}
		delay(4);
	}


	accScale.y = GRAVITY / tempaccy.y;
	accScale.x = GRAVITY / tempaccx.x;

	idptr = (uint32_t)SYS_ID(sensorNameBMI088);
    memoryMemUpload(sensorNameBMI088, NULL);
}

void _sensorMeanBMI088(vec_t* accMean, vec_t* gyrMean){
    float i = 0;
    float j = 0;
    uint32_t intervalMs = millis() + BMI088_CAL_INTERVAL_MS;

    while(millis() < intervalMs){
        if(_sensorAccData(&accData)){
            *accMean = vmean(*accMean, accData, i);
            i++;
        }
        if(_sensorGyrData(&gyrData)){
            *gyrMean = vmean(*gyrMean, gyrData, j);
            j++;
        }
        delay(2);
    }
}

void _sensorVarianceBMI088(vec_t* accVar, vec_t* gyrVar){
    float i = 0;
    float j = 0;
    uint32_t intervalMs = millis() + BMI088_CAL_INTERVAL_MS;
    *accVar = vzero();
    *gyrVar = vzero();

    while(millis() < intervalMs){
        if(_sensorAccData(&accData)){
            *accVar = vsigma2(*accVar, accMean, accData, i);
            i++;
        }
        if(_sensorGyrData(&gyrData)){
            *gyrVar = vsigma2(*gyrVar, gyrMean, gyrData, j);
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

int8_t sensorAcquireBMI088(measurement_t* plist, uint8_t n){
    uint8_t i = 0;
    if(accNew && i < n){plist[i] = macc; accNew = 0; i++;}
    if(gyrNew && i < n){plist[i] = mgyr; gyrNew = 0; i++;}
    return i;
}

int8_t sensorIsReadyBMI088(void){return isReady;}

void sensorWaitDataReadyBMI088(void){while(!accNew) delay(2);}

uint8_t _sensorAccData(vec_t* data){
    /* Output as G values ((9,81m)/s^2)*/
    struct bmi08_sensor_data bmi08AccData;
    int8_t rslt = bmi08a_get_data(&bmi08AccData, &bmi08dev);
    *data = vscl(mkvec(bmi08AccData.y, bmi08AccData.x * -1, bmi08AccData.z), BMI088_G_PER_LSB * GRAVITY);
    return (rslt == BMI08_OK);
}

uint8_t _sensorGyrData(vec_t* data){
    /* Output as deg/s values (deg/s)*/
    struct bmi08_sensor_data bmi08GyrData;
    int8_t rslt = bmi08g_get_data(&bmi08GyrData, &bmi08dev);
    *data = vscl(mkvec(bmi08GyrData.y, bmi08GyrData.x * -1, bmi08GyrData.z), BMI088_DEG_PER_LSB);
    return (rslt == BMI08_OK);
}

BMI08_INTF_RET_TYPE bmi08SpiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint16_t intfpin = (*((uint8_t*)intf_ptr) == 0) ? BMI088_ACC_CS : BMI088_GYR_CS;

    spiBeginTransaction( &BMI088_SPI );
    pinWrite(intfpin, 0);
    spiTransmit(&BMI088_SPI, &reg_addr, 1);
    int8_t result = spiReceive(&BMI088_SPI, reg_data, (uint16_t)len);
    pinWrite(intfpin, 1);
    spiEndTransaction(&BMI088_SPI);
    return result;
}

BMI08_INTF_RET_TYPE bmi08SpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){

    uint16_t intfpin = (*((uint8_t*)intf_ptr) == 0) ? BMI088_ACC_CS : BMI088_GYR_CS;

    spiBeginTransaction(&BMI088_SPI);
    pinWrite(intfpin, 0);
    spiTransmit(&BMI088_SPI, &reg_addr, 1);
    int8_t result = spiTransmit(&BMI088_SPI, (uint8_t*)reg_data, len);
    pinWrite(intfpin, 1);
    spiEndTransaction(&BMI088_SPI);

    return result;
}

void bmi08DelayUs(uint32_t period, void *intf_ptr){
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
