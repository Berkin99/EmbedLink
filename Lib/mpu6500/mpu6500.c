/*
 *  mpu6500.c
 *
 *  Created on: Jan 12, 2024
 *  Author: BerkN
 *
 *  TDK IvenSense MPU6500 sensor driver.
 *  Platform independent C API.
 *  Calibration and raw data transformation.
 *  SPI or I2C based communication.
 *  Adaptive to changed settings.
 *
 *  Updates and bug reports :  @ https://github.com/Berkin99/MPU6500
 *
 *  12.01.2024 : Created MPU6500 module driver.
 *  03.10.2024 : Constructor updated, comments added.
 *
 *  References:
 *  [0] PS-MPU-6500A-01-v1.3.pdf (Datasheet)
 *  [1] MPU-6500-Register-Map2.pdf
 *
 */

#include "mpu6500.h"

#define REG_XG_OFFS_TC              0x00
#define REG_YG_OFFS_TC              0x01
#define REG_ZG_OFFS_TC              0x02
#define REG_X_FINE_GAIN             0x03
#define REG_Y_FINE_GAIN             0x04
#define REG_Z_FINE_GAIN             0x05
#define REG_XA_OFFS_H               0x06
#define REG_XA_OFFS_L               0x07
#define REG_YA_OFFS_H               0x08
#define REG_YA_OFFS_L               0x09
#define REG_ZA_OFFS_H               0x0A
#define REG_ZA_OFFS_L               0x0B
#define REG_PRODUCT_ID              0x0C
#define REG_SELF_TEST_X             0x0D
#define REG_SELF_TEST_Y             0x0E
#define REG_SELF_TEST_Z             0x0F
#define REG_SELF_TEST_A             0x10
#define REG_XG_OFFS_USRH            0x13
#define REG_XG_OFFS_USRL            0x14
#define REG_YG_OFFS_USRH            0x15
#define REG_YG_OFFS_USRL            0x16
#define REG_ZG_OFFS_USRH            0x17
#define REG_ZG_OFFS_USRL            0x18
#define REG_SMPLRT_DIV              0x19
#define REG_CONFIG                  0x1A
#define REG_GYRO_CONFIG             0x1B
#define REG_ACCEL_CONFIG            0x1C
#define REG_INT_PIN_CFG             0x37
#define REG_INT_ENABLE              0x38
#define REG_ACCEL_XOUT_H            0x3B
#define REG_ACCEL_XOUT_L            0x3C
#define REG_ACCEL_YOUT_H            0x3D
#define REG_ACCEL_YOUT_L            0x3E
#define REG_ACCEL_ZOUT_H            0x3F
#define REG_ACCEL_ZOUT_L            0x40
#define REG_TEMP_OUT_H              0x41
#define REG_TEMP_OUT_L              0x42
#define REG_GYRO_XOUT_H             0x43
#define REG_GYRO_XOUT_L             0x44
#define REG_GYRO_YOUT_H             0x45
#define REG_GYRO_YOUT_L             0x46
#define REG_GYRO_ZOUT_H             0x47
#define REG_GYRO_ZOUT_L             0x48
#define REG_USER_CTRL               0x6A
#define REG_PWR_MGMT_1              0x6B
#define REG_PWR_MGMT_2              0x6C
#define REG_BANK_SEL                0x6D
#define REG_MEM_START_ADDR          0x6E
#define REG_MEM_R_W                 0x6F
#define REG_DMP_CFG_1               0x70
#define REG_DMP_CFG_2               0x71
#define REG_FIFO_COUNTH             0x72
#define REG_FIFO_COUNTL             0x73
#define REG_FIFO_R_W                0x74
#define REG_WHOAMI                  0x75

#define READ_FLAG                   0x80

MPU6500_Device_t MPU6500_NewDevice(void* intf, MPU6500_Intf_e intf_type, MPU6500_Read_t readf, MPU6500_Write_t writef, MPU6500_Delay_t delayf){
    MPU6500_Device_t dev = {
        .intf = intf,
        .intf_type = intf_type,
        .read = readf,
        .write = writef,
        .delay = delayf
    };
    return dev;
}

int8_t MPU6500_Init(MPU6500_Device_t *dev){
    MPU6500_SetSleepMode(dev, 0);
    MPU6500_GetSettings(dev);
    for(uint8_t i = 0; i < 3; i++){
        dev->calib_data.acc_cal[i]      = 0;
        dev->calib_data.acc_scale[i][0] = 1;
        dev->calib_data.acc_scale[i][1] = 1;
        dev->calib_data.gyr_cal[i]      = 0;
    }
    return MPU6500_Test(dev);
}

int8_t MPU6500_Test(MPU6500_Device_t *dev){
    if(MPU6500_GetDeviceID(dev) == 0x70) return MPU6500_OK;
    return MPU6500_ERROR;
}

void MPU6500_ApplySettings(MPU6500_Device_t *dev, struct MPU6500_Settings settings){
    MPU6500_SetAccelRange(dev,settings.acc_range);
    MPU6500_SetGyroRange(dev,settings.gyr_range);
    MPU6500_SetDLPFBandWidth(dev,settings.dlpf);
    MPU6500_SetSampleRateDivider(dev,settings.srd);
    MPU6500_SetClockSource(dev,settings.clock_source);
}

void MPU6500_GetSettings(MPU6500_Device_t *dev){
    MPU6500_GetAccelRange(dev);
    MPU6500_GetGyroRange(dev);
    MPU6500_GetDLPFBandwidth(dev);
    MPU6500_GetSampleRateDivider(dev);
    MPU6500_GetClockSource(dev);
}

void MPU6500_AccCalibration(MPU6500_Device_t *dev, uint32_t time_ms){
    /* Accelerometer calibration for parallel surface*/
    double acc_cal[3] = {0.0, 0.0, 0.0};
    int16_t acc[3];
    int16_t gyr[3];

    for(uint8_t i = 0; i < 3; i++) dev->calib_data.acc_cal[i] = 0;

    for(uint32_t i = 0; i < time_ms; i++){
        MPU6500_GetDataRaw(dev, acc, gyr);
        acc_cal[0]+= acc[0];
        acc_cal[1]+= acc[1];
        acc_cal[2]+= acc[2];
        dev->delay(1);
    }
    for(uint8_t i = 0; i<3; i++){dev->calib_data.acc_cal[i] = (int16_t)(acc_cal[i] / time_ms);}

    dev->calib_data.acc_cal[2] = 0; /* Do not calibrate Z axis */
}

void MPU6500_GyrCalibration(MPU6500_Device_t *dev, uint32_t time_ms){
    double gyr_cal[3] = {0,0,0};
    int16_t acc[3];
    int16_t gyr[3];

    for(uint8_t i = 0; i < 3; i++) dev->calib_data.gyr_cal[i] = 0;

    for(uint32_t i = 0; i < time_ms; i++){
        MPU6500_GetDataRaw(dev, acc, gyr);
        gyr_cal[0]+= gyr[0];
        gyr_cal[1]+= gyr[1];
        gyr_cal[2]+= gyr[2];
        dev->delay(2);
    }
    for(uint8_t i = 0; i<3; i++){dev->calib_data.gyr_cal[i] = (int16_t)(gyr_cal[i]/time_ms);}
}

int8_t MPU6500_ReadRegister(MPU6500_Device_t *dev, uint8_t reg, uint8_t* pRxBuffer, uint8_t len){
    reg |= READ_FLAG;
    return dev->read(dev->intf, reg, pRxBuffer, len);
}

int8_t MPU6500_WriteRegister(MPU6500_Device_t *dev, uint8_t reg, const uint8_t* pTxBuffer, uint8_t len){
    return dev->write(dev->intf, reg, pTxBuffer, len);
}

uint8_t MPU6500_GetDeviceID(MPU6500_Device_t *dev){
    uint8_t buf = 0;
    MPU6500_ReadRegister(dev, REG_WHOAMI, &buf, 1);
    return buf;
}

void MPU6500_SetSleepMode(MPU6500_Device_t *dev, uint8_t isSleep){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev, REG_PWR_MGMT_1, &reg, 1);
    reg &= 0b10111111;                         /* Clear the setting area */
    reg |= ((isSleep & 0b00000001)<<6);        /* Add setting to config register value */
    MPU6500_WriteRegister(dev, REG_CONFIG, &reg, 1);
}

int8_t MPU6500_GetDataRaw(MPU6500_Device_t *dev, int16_t* acc, int16_t* gyr){
    uint8_t _buffer [14];
    int8_t result = MPU6500_ReadRegister(dev, REG_ACCEL_XOUT_H,_buffer, 14);

    acc[0] = ((((int16_t)_buffer[0])  << 8) | _buffer[1]);
    acc[1] = ((((int16_t)_buffer[2])  << 8) | _buffer[3]);
    acc[2] = ((((int16_t)_buffer[4])  << 8) | _buffer[5]);
    gyr[0] = ((((int16_t)_buffer[8])  << 8) | _buffer[9]);
    gyr[1] = ((((int16_t)_buffer[10]) << 8) | _buffer[11]);
    gyr[2] = ((((int16_t)_buffer[12]) << 8) | _buffer[13]);

    return result;
}

void MPU6500_GetData(MPU6500_Device_t *dev, float* acc, float* gyr){

    int16_t _acc[3];
    int16_t _gyr[3];

    MPU6500_GetDataRaw(dev, _acc, _gyr);

    for (uint8_t i = 0; i < 3; i++){
        acc[i] = (float) (_acc[i] - dev->calib_data.acc_cal[i]) * dev->calib_data.acc_coefficient;

        if(acc[i] > 0) acc[i] *= dev->calib_data.acc_scale[i][0];
        else acc[i] *= dev->calib_data.acc_scale[i][1];

        gyr[i] = (float) (_gyr[i] - dev->calib_data.gyr_cal[i]) * dev->calib_data.gyr_coefficient;
    }
}

void MPU6500_SetAccelRange(MPU6500_Device_t *dev,MPU6500_AccelRange_e range){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev, REG_ACCEL_CONFIG, &reg, 1);
    reg &= 0b11100111;                 /* Clear the setting area */
    reg |= ((uint8_t)range << 3);      /* Add setting to config register value */
    MPU6500_WriteRegister(dev, REG_ACCEL_CONFIG, &reg, 1);
    dev->settings.acc_range = range;
    dev->calib_data.acc_coefficient = ((float)(1<<range)) * 4 / 65536.0f; /* +-2G = 4 */
}

void MPU6500_SetGyroRange(MPU6500_Device_t *dev, MPU6500_GyroRange_e range){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev,REG_GYRO_CONFIG, &reg, 1);
    reg &= 0b11100111;                 /* Clear the setting area */
    reg |= ((uint8_t)range << 3);     // Add setting to config register value
    MPU6500_WriteRegister(dev,REG_GYRO_CONFIG, &reg, 1);
    dev->settings.gyr_range = range;
    dev->calib_data.gyr_coefficient = (float)(1<<range)*500/65536.0f; /* +-250deg/s = 500 */
}

void MPU6500_SetDLPFBandWidth(MPU6500_Device_t *dev, MPU6500_DLPFBandWidth_e dlpf){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev,REG_CONFIG, &reg, 1);
    reg &= 0b11111000;             /* Clear the setting area */
    reg |= (uint8_t)dlpf;
    MPU6500_WriteRegister(dev,REG_CONFIG, &reg, 1);
    dev->settings.dlpf = dlpf;
}

void MPU6500_SetSampleRateDivider(MPU6500_Device_t *dev,MPU6500_SampleRateDivider_e srd){
    uint8_t temp = srd;
    MPU6500_WriteRegister(dev, REG_SMPLRT_DIV,&temp,1);
    dev->settings.srd = srd;
}

void MPU6500_SetClockSource(MPU6500_Device_t *dev, MPU6500_ClockSource_e clock_source){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev,REG_PWR_MGMT_1, &reg, 1);
    reg &= 0b11111000;              /* Clear the area */
    reg |= (uint8_t)clock_source;
    MPU6500_WriteRegister(dev,REG_PWR_MGMT_1, &reg, 1);
    dev->settings.clock_source = clock_source;
}


uint8_t MPU6500_GetAccelRange(MPU6500_Device_t *dev){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev, REG_ACCEL_CONFIG, &reg, 1);
    reg &= 0b00011000;    /* Clear the area */
    reg = reg>>3;
    dev->settings.acc_range = reg;
    return reg;
}

uint8_t MPU6500_GetGyroRange(MPU6500_Device_t *dev){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev ,REG_GYRO_CONFIG, &reg, 1);
    reg &= 0b00011000;    /* Clear the area */
    reg = reg>>3;
    dev->settings.gyr_range = reg;
    return reg;
}

uint8_t MPU6500_GetDLPFBandwidth(MPU6500_Device_t *dev){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev, REG_CONFIG, &reg, 1);
    reg &= 0b00000111; /* Clear the area */
    dev->settings.dlpf = reg;
    return reg;
}

uint8_t MPU6500_GetSampleRateDivider(MPU6500_Device_t *dev){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev, REG_SMPLRT_DIV, &reg, 1);
    dev->settings.srd = reg;
    return reg;

}

uint8_t MPU6500_GetClockSource(MPU6500_Device_t *dev){
    uint8_t reg = 0;
    MPU6500_ReadRegister(dev, REG_PWR_MGMT_1, &reg, 1);
    reg &= 0b00000111;    /* Clear the area */
    dev->settings.clock_source = reg;
    return reg;
}
