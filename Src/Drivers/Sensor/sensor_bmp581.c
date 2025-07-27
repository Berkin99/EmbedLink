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
#include "rtos.h"

#ifdef   BMP581_I2C

#include "xmath.h"
#include "sensor.h"
#include "bmp5.h"
#include "i2c.h"
#include "uart.h"
#include "filter.h"

#define sensorNameBMP581       "BMP581"
#define sensorFreqBMP581       (100)    /* Hz */

static struct bmp5_dev bmp5dev;

static struct {
    float pressure;
    float temperature;
    float pressure_mean;
    float temperature_mean;
    float pressure_scale;
} bmp581_data;

static sense_t pressureSense;
static sense_t tempSense;

static uint8_t isInit = 0;
static int8_t isReady = 0;

taskAllocateStatic(BMP581, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskBMP581(void* argv);

static int8_t bmp581_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static int8_t bmp581_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void   bmp581_delay_us(uint32_t period, void *intf_ptr);

/* ------------------- Sensor Interface ------------------- */
int8_t sensorInitBMP581(void) {
    if (isInit) return E_OVERWRITE;

    i2cInit();

    /* BMP5 Device Setup */
    bmp5dev.intf_ptr = (void*)&BMP581_I2C;
    bmp5dev.read     = bmp581_i2c_read;
    bmp5dev.write    = bmp581_i2c_write;
    bmp5dev.intf     = BMP5_I2C_INTF;
    bmp5dev.delay_us = bmp581_delay_us;
    uint8_t reg_data;

    bmp5_get_regs(BMP5_REG_CHIP_ID, &reg_data, 1, &bmp5dev);
    if (reg_data != 0x50) {
        serialPrint("[-] BMP581 init ERROR\r\n");
        return ERROR;
    }

    bmp5_init(&bmp5dev);
    
    /* Power mode & Config */
    struct bmp5_osr_odr_press_config press_cfg = {0};
    bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp5dev);
    bmp5_get_osr_odr_press_config(&press_cfg, &bmp5dev);

    press_cfg.press_en = BMP5_ENABLE;
    press_cfg.odr = BMP5_ODR_20_HZ;
    press_cfg.osr_p = BMP5_OVERSAMPLING_64X;
    bmp5_set_osr_odr_press_config(&press_cfg, &bmp5dev);

    struct bmp5_iir_config iir_cfg = {
        .set_iir_t = BMP5_IIR_FILTER_COEFF_1,
        .set_iir_p = BMP5_IIR_FILTER_COEFF_3,
        .shdw_set_iir_t = BMP5_ENABLE,
        .shdw_set_iir_p = BMP5_ENABLE
    };
    bmp5_set_iir_config(&iir_cfg, &bmp5dev);

    bmp5_set_power_mode(BMP5_POWERMODE_CONTINOUS, &bmp5dev);

    /* Task Create */
    taskCreateStatic(BMP581, sensorTaskBMP581, NULL);

    isInit = 1;
    return OK;
}

int8_t sensorTestBMP581(void) {
    uint8_t chip_id = 0;
    bmp5_get_regs(0x01, &chip_id, 1, &bmp5dev); // 0x01 = Chip ID register
    return (chip_id == 0x50) ? OK : ERROR; // BMP581 için chip_id genellikle 0x50
}

void sensorTaskBMP581(void* argv) {
    delay(100);

    pressureSense.type        = SENSE_PRESSURE;
    tempSense.type            = SENSE_TEMPERATURE;
    pressureSense.xvec.stdDev = vnew(1.0f, 99999.0f, 99999.0f);
    tempSense.xvec.stdDev     = vnew(0.01f, 99999.0f, 99999.0f);

    isReady = 1;
    uint32_t xLastWakeTime = taskGetTickCount();

    struct bmp5_osr_odr_press_config press_cfg = {0};
    bmp5_get_osr_odr_press_config(&press_cfg, &bmp5dev);

    while (1) {
        struct bmp5_sensor_data sensor_data = {0};
        if (bmp5_get_sensor_data(&sensor_data, &press_cfg, &bmp5dev) == 0) {
            bmp581_data.pressure = sensor_data.pressure / 100.0f;
            pressureSense.xvec.x = bmp581_data.pressure;
            pressureSense.xvec.timestampMs = millis();

            bmp581_data.temperature = sensor_data.temperature;
            tempSense.xvec.x = bmp581_data.temperature;
            tempSense.xvec.timestampMs = millis();

            sensorEnqueue(&pressureSense, 0);
            sensorEnqueue(&tempSense, 0);
        }
        taskDelayUntil(&xLastWakeTime, (1000 / sensorFreqBMP581));
    }
}

void sensorCalibrateBMP581(void) {
    float p_sum = 0, t_sum = 0;
    for (int i = 0; i < 100; i++) {
        struct bmp5_sensor_data sdata = {0};
        struct bmp5_osr_odr_press_config press_cfg = {0};
        bmp5_get_osr_odr_press_config(&press_cfg, &bmp5dev);
        if (bmp5_get_sensor_data(&sdata, &press_cfg, &bmp5dev) == 0) {
            p_sum += sdata.pressure;
            t_sum += sdata.temperature;
        }
        delay(5);
    }
    bmp581_data.pressure_mean = p_sum / 100;
    bmp581_data.temperature_mean = t_sum / 100;
    bmp581_data.pressure_scale = 1.0f;
}

int8_t sensorIsCalibratedBMP581(void) {

    return FALSE;
}

int8_t sensorAcquireBMP581(sense_t* plist, uint8_t n) {

    return E_ERROR;
}

int8_t sensorIsReadyBMP581(void) {
    return isReady;
}

void sensorWaitDataReadyBMP581(void) {
    while (!isReady);
}

/* ------------------- Low-level I2C Interface ------------------- */
static int8_t bmp581_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    return i2cMemRead((i2c_t*)intf_ptr, BMP581_I2C_ADDR, reg_addr, reg_data, (uint16_t)len);
}

static int8_t bmp581_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    return i2cMemWrite((i2c_t*)intf_ptr, BMP581_I2C_ADDR, reg_addr, (uint8_t*)reg_data, (uint16_t)len);
}

static void bmp581_delay_us(uint32_t period, void *intf_ptr) {
    (void) intf_ptr;
    delayUs(period);
}

#endif /* BMP581_I2C */
