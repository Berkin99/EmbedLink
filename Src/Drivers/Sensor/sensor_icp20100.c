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

 /*
 * ICP20100 Pressure Sensor Driver - Clean Implementation
 * Optimized for accuracy and maintainability
 */


 /*
 * ICP20100 Pressure Sensor Driver - Clean Implementation
 * Optimized for accuracy and maintainability
 */

#include <sysdefs.h>
#include <sysconfig.h>
#include <systime.h>
#include "rtos.h"

#ifdef ICP20100_I2C

#include "xmath.h"
#include "sensor.h"
#include "icp20100.h"
#include "i2c.h"
#include "uart.h"
#include "filter.h"
#include "sensor_icp20100.h"

#define ICP20100_I2C_ADDR         (0x63)
#define PRESSURE_FILTER_ALPHA     (0.25f)
#define TEMPERATURE_FILTER_ALPHA  (0.08f)
#define REFERENCE_PRESSURE_MBAR   (1005.00f)
#define KPA_TO_MBAR_FACTOR        (10.0f)
#define BIAS_SAMPLES              (50)
#define SETTLING_SAMPLES          (14)

/* Global state */
static struct icp20100_dev icp20100dev;
static struct {
    float pressure_filtered;
    float temperature_filtered;
    float pressure_bias;
    float pressure_mbar;
} sensor_data;

static sense_t pressureSense, tempSense;
static uint8_t isInit = 0, isReady = 0, settling_count = 0, needs_settling = 0;

taskAllocateStatic(ICP20100, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskICP20100(void* argv);

/* Function prototypes */
static int8_t setup_device(void);
static int8_t calculate_bias_and_init_filters(void);
static int8_t process_sensor_reading(void);
static float apply_lowpass_filter(float new_val, float *filtered_val, float alpha);
static void setup_sensor_outputs(void);

/* I2C Interface functions */
static int8_t icp20100_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr);
static int8_t icp20100_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr);
static void icp20100_delay_us(uint32_t period, void *ptr);

/* ==================== Public Interface ==================== */

int8_t sensorInitICP20100(void) {
    if (isInit) return E_OVERWRITE;

    if (setup_device() != OK) return ERROR;
    if (calculate_bias_and_init_filters() != OK) return ERROR;
    
    taskCreateStatic(ICP20100, sensorTaskICP20100, NULL);
    isInit = 1;
    return OK;
}

int8_t sensorTestICP20100(void) {
    return (icp20100dev.chip_id == ICP20100_CHIP_ID) ? OK : ERROR;
}

int8_t sensorIsReadyICP20100(void) {
    return isReady;
}

void sensorWaitDataReadyICP20100(void) {
    while (!isReady);
}

void sensorTaskICP20100(void* argv) {
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
        taskDelayUntil(&xLastWakeTime, (1000 / sensorFreqICP20100));
    }
}

/* ==================== Private Implementation ==================== */

static int8_t setup_device(void) {
    /* Configure device interface */
    icp20100dev.intf_ptr = (void*)&ICP20100_I2C;
    icp20100dev.read = icp20100_i2c_read;
    icp20100dev.write = icp20100_i2c_write;
    icp20100dev.intf = ICP20100_I2C_INTF;
    icp20100dev.delay_us = icp20100_delay_us;

    /* Initialize and verify device */
    if (icp20100_init(&icp20100dev) != ICP20100_OK) {
        serialPrint("[-] ICP20100 init ERROR\n");
        return ERROR;
    }

    if (icp20100dev.chip_id != ICP20100_CHIP_ID) {
        serialPrint("[-] ICP20100 chip ID ERROR\n");
        return ERROR;
    }

    /* Boot sequence for Version A */
    if (icp20100dev.version == ICP20100_VERSION_A) {
        if (icp20100_boot_sequence(&icp20100dev) != ICP20100_OK) {
            serialPrint("[-] ICP20100 boot sequence ERROR\n");
            return ERROR;
        }
    }

    /* Configure for high accuracy mode */
    struct icp20100_config config = {
        .meas_mode = ICP20100_MODE_4,
        .meas_type = ICP20100_MEAS_MODE_CONTINUOUS,
        .power_mode = ICP20100_POWER_MODE_NORMAL,
        .fifo_mode = ICP20100_FIFO_PRESSURE_FIRST,
        .drive_strength = ICP20100_DRIVE_STRENGTH_1_8V_2MA
    };

    if (icp20100_set_config(&config, &icp20100dev) != ICP20100_OK) {
        serialPrint("[-] ICP20100 config ERROR\n");
        return ERROR;
    }

    /* Check if settling is needed */
    if (config.meas_mode <= ICP20100_MODE_3) {
        needs_settling = 1;
        settling_count = 0;
    }

    return OK;
}

static int8_t calculate_bias_and_init_filters(void) {
    delay(100); /* Sensor stabilization */
    
    float pressure_sum = 0;
    int valid_readings = 0;
    bool filter_init = false;

    for (int i = 0; i < BIAS_SAMPLES; i++) {
        struct icp20100_data raw_data;
        if (icp20100_get_data(&raw_data, &icp20100dev) == ICP20100_OK) {
            float pressure_mbar = raw_data.pressure * KPA_TO_MBAR_FACTOR;
            
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
        delay(100);
    }

    if (valid_readings == 0) {
        serialPrint("[-] ICP20100 bias calculation failed\n");
        return ERROR;
    }

    /* Calculate bias from filtered average */
    float avg_pressure = pressure_sum / valid_readings;
    sensor_data.pressure_bias = REFERENCE_PRESSURE_MBAR - avg_pressure;
    
    serialPrint("[+] ICP20100 avg: %.2f mBar, bias: %.2f mBar\n", 
                avg_pressure, sensor_data.pressure_bias);
    return OK;
}

static int8_t process_sensor_reading(void) {
    /* Handle settling period */
    if (needs_settling) {
        settling_count++;
        if (settling_count >= SETTLING_SAMPLES) {
            needs_settling = 0;
            serialPrint("[+] ICP20100 FIR filter settled\n");
        }
        return ERROR; /* Skip processing during settling */
    }

    /* Check FIFO and read data */
    uint8_t fifo_fill = 0;
    if (icp20100dev.read(ICP20100_REG_FIFO_FILL, &fifo_fill, 1, icp20100dev.intf_ptr) != ICP20100_OK) {
        return ERROR;
    }

    uint8_t fifo_level = fifo_fill & ICP20100_FIFO_LEVEL_MASK;
    if (fifo_level == 0) return ERROR;

    struct icp20100_data raw_data;
    if (icp20100_get_data(&raw_data, &icp20100dev) != ICP20100_OK) {
        return ERROR;
    }

    /* Apply filtering and bias correction */
    float pressure_mbar = raw_data.pressure * KPA_TO_MBAR_FACTOR;
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

static int8_t icp20100_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr) {
    return i2cMemRead((i2c_t*)ptr, ICP20100_I2C_ADDR, reg, data, (uint16_t)len);
}

static int8_t icp20100_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr) {
    return i2cMemWrite((i2c_t*)ptr, ICP20100_I2C_ADDR, reg, (uint8_t*)data, (uint16_t)len);
}

static void icp20100_delay_us(uint32_t period, void *ptr) {
    (void)ptr;
    delayUs(period);
}

/* ==================== Stub Functions ==================== */

void sensorCalibrateICP20100(void) { /* Not implemented */ }
int8_t sensorIsCalibratedICP20100(void) { return FALSE; }
int8_t sensorAcquireICP20100(sense_t* plist, uint8_t n) { return E_ERROR; }

#endif /* ICP20100_I2C */