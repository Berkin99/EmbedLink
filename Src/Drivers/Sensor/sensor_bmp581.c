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

#ifdef BMP581_I2C

#include "xmath.h"
#include "sensor.h"
#include "bmp5.h"
#include "i2c.h"
#include "uart.h"
#include "filter.h"
#include "sensor_bmp581.h"

#define BMP581_I2C_ADDR           (0x46)
#define BMP581_CHIP_ID            (0x50)
#define PRESSURE_FILTER_ALPHA     (0.1f)
#define TEMPERATURE_FILTER_ALPHA  (0.05f)
#define REFERENCE_PRESSURE_MBAR   (1005.00f)
#define HPASCAL_TO_MBAR_FACTOR    (1.0f)
#define BIAS_SAMPLES              (10)

/* Global state */
static struct bmp5_dev bmp5dev;
static struct bmp5_osr_odr_press_config press_cfg;
static struct {
    float pressure_filtered;
    float temperature_filtered;
    float pressure_bias;
    float pressure_mbar;
} sensor_data;

static sense_t pressureSense, tempSense;
static uint8_t isInit = 0, isReady = 0;

void sensorTaskBMP581(void* argv);
taskAllocateStatic(BMP581, SENS_TASK_STACK, SENS_TASK_PRI);

/* Function prototypes */
static int8_t setup_device(void);
static int8_t configure_sensor(void);
static int8_t calculate_bias_and_init_filters(void);
static int8_t process_sensor_reading(void);
static float apply_lowpass_filter(float new_val, float *filtered_val, float alpha);
static void setup_sensor_outputs(void);

/* I2C Interface functions */
static int8_t bmp581_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr);
static int8_t bmp581_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr);
static void bmp581_delay_us(uint32_t period, void *ptr);

/* ==================== Public Interface ==================== */

int8_t sensorInitBMP581(void) {
    if (isInit) return E_OVERWRITE;

    if (setup_device() != OK) return ERROR;
    if (configure_sensor() != OK) return ERROR;
    if (calculate_bias_and_init_filters() != OK) return ERROR;
    
    taskCreateStatic(BMP581, sensorTaskBMP581, NULL);
    isInit = 1;
    return OK;
}

int8_t sensorTestBMP581(void) {
    uint8_t chip_id = 0;
    bmp5_get_regs(BMP5_REG_CHIP_ID, &chip_id, 1, &bmp5dev);
    return (chip_id == BMP581_CHIP_ID) ? OK : ERROR;
}

int8_t sensorIsReadyBMP581(void) {
    return isReady;
}

void sensorWaitDataReadyBMP581(void) {
    while (!isReady);
}

void sensorTaskBMP581(void* argv) {
    delay(100);
    setup_sensor_outputs();
    isReady = 1;
    
    uint32_t xLastWakeTime = taskGetTickCount();
    
    while (1) {
        if (process_sensor_reading() == OK) {
            pressureSense.xvec.x = sensor_data.pressure_mbar;
            pressureSense.xvec.timestampMs = millis();
            tempSense.xvec.x = sensor_data.temperature_filtered;
            tempSense.xvec.timestampMs = millis();
            
            sensorEnqueue(&pressureSense, 0);
            sensorEnqueue(&tempSense, 0);
        }
        taskDelayUntil(&xLastWakeTime, (1000 / sensorFreqBMP581));
    }
}

/* ==================== Private Implementation ==================== */

static int8_t setup_device(void) {
    /* Configure device interface */
    bmp5dev.intf_ptr = (void*)&BMP581_I2C;
    bmp5dev.read = bmp581_i2c_read;
    bmp5dev.write = bmp581_i2c_write;
    bmp5dev.intf = BMP5_I2C_INTF;
    bmp5dev.delay_us = bmp581_delay_us;

    /* Verify chip ID */
    uint8_t chip_id;
    bmp5_get_regs(BMP5_REG_CHIP_ID, &chip_id, 1, &bmp5dev);
    if (chip_id != BMP581_CHIP_ID) {
        serialPrint("[-] BMP581 init ERROR\n");
        return ERROR;
    }

    /* Initialize device */
    bmp5_init(&bmp5dev);

    return OK;
}

static int8_t configure_sensor(void) {
    /* Set standby mode for configuration */
    bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp5dev);
    
    /* Configure pressure measurement */
    bmp5_get_osr_odr_press_config(&press_cfg, &bmp5dev);
    press_cfg.press_en = BMP5_ENABLE;
    press_cfg.odr = BMP5_ODR_10_HZ;
    press_cfg.osr_p = BMP5_OVERSAMPLING_32X;
    
    if (bmp5_set_osr_odr_press_config(&press_cfg, &bmp5dev) != BMP5_OK) {
        serialPrint("[-] BMP581 pressure config ERROR\n");
        return ERROR;
    }

    /* Configure IIR filter */
    struct bmp5_iir_config iir_cfg = {
        .set_iir_t = BMP5_IIR_FILTER_COEFF_1,
        .set_iir_p = BMP5_IIR_FILTER_COEFF_7,
        .shdw_set_iir_t = BMP5_DISABLE,
        .shdw_set_iir_p = BMP5_ENABLE
    };
    
    if (bmp5_set_iir_config(&iir_cfg, &bmp5dev) != BMP5_OK) {
        serialPrint("[-] BMP581 IIR config ERROR\n");
        return ERROR;
    }

    /* Start continuous measurement */
    bmp5_set_power_mode(BMP5_POWERMODE_CONTINOUS, &bmp5dev);
    return OK;
}

static int8_t calculate_bias_and_init_filters(void) {
    delay(100); /* Sensor stabilization */
    
    float pressure_sum = 0;
    int valid_readings = 0;
    bool filter_init = false;

    for (int i = 0; i < BIAS_SAMPLES; i++) {
        struct bmp5_sensor_data raw_data = {0};
        if (bmp5_get_sensor_data(&raw_data, &press_cfg, &bmp5dev) == BMP5_OK) {
            float pressure_mbar = raw_data.pressure / 100.0f;
            
            if (!filter_init) {
                /* Initialize filters with first reading */
                sensor_data.pressure_filtered = pressure_mbar;
                sensor_data.temperature_filtered = raw_data.temperature;
                filter_init = true;
            } else {
                /* Apply filtering during bias calculation */
                apply_lowpass_filter(pressure_mbar, &sensor_data.pressure_filtered, PRESSURE_FILTER_ALPHA);
                apply_lowpass_filter(raw_data.temperature, &sensor_data.temperature_filtered, TEMPERATURE_FILTER_ALPHA);
            }
            
            pressure_sum += sensor_data.pressure_filtered;
            valid_readings++;
        }
        delay(10);
    }

    if (valid_readings == 0) {
        serialPrint("[-] BMP581 bias calculation failed\n");
        return ERROR;
    }

    /* Calculate bias from filtered average */
    float avg_pressure = pressure_sum / valid_readings;
    sensor_data.pressure_bias = REFERENCE_PRESSURE_MBAR - avg_pressure;
    
    serialPrint("[+] BMP581 avg: %.2f mBar, bias: %.2f mBar\n", 
                avg_pressure, sensor_data.pressure_bias);
    return OK;
}

static int8_t process_sensor_reading(void) {
    struct bmp5_sensor_data raw_data = {0};
    if (bmp5_get_sensor_data(&raw_data, &press_cfg, &bmp5dev) != BMP5_OK) {
        return ERROR;
    }

    /* Apply filtering and bias correction */
    float pressure_mbar = raw_data.pressure / 100.0f;
    apply_lowpass_filter(pressure_mbar, &sensor_data.pressure_filtered, PRESSURE_FILTER_ALPHA);
    apply_lowpass_filter(raw_data.temperature, &sensor_data.temperature_filtered, TEMPERATURE_FILTER_ALPHA);
    
    sensor_data.pressure_mbar = sensor_data.pressure_filtered + sensor_data.pressure_bias;
    return OK;
}

static float apply_lowpass_filter(float new_val, float *filtered_val, float alpha) {
    *filtered_val = alpha * new_val + (1.0f - alpha) * (*filtered_val);
    return *filtered_val;
}

static void setup_sensor_outputs(void) {
    pressureSense.type = SENSE_PRESSURE;
    pressureSense.xvec.stdDev = vnew(1.0f, 99999.0f, 99999.0f);
    
    tempSense.type = SENSE_TEMPERATURE;
    tempSense.xvec.stdDev = vnew(0.01f, 99999.0f, 99999.0f);
}

/* ==================== I2C Interface ==================== */

static int8_t bmp581_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr) {
    return i2cMemRead((i2c_t*)ptr, BMP581_I2C_ADDR, reg, data, (uint16_t)len);
}

static int8_t bmp581_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr) {
    return i2cMemWrite((i2c_t*)ptr, BMP581_I2C_ADDR, reg, (uint8_t*)data, (uint16_t)len);
}

static void bmp581_delay_us(uint32_t period, void *ptr) {
    (void)ptr;
    delayUs(period);
}

/* ==================== Stub Functions ==================== */

void sensorCalibrateBMP581(void) { /* Not implemented */ }
int8_t sensorIsCalibratedBMP581(void) { return FALSE; }
int8_t sensorAcquireBMP581(sense_t* plist, uint8_t n) { return E_ERROR; }

#endif /* BMP581_I2C */