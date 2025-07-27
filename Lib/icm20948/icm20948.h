#ifndef ICM20948_H_
#define ICM20948_H_

#include <stdint.h>
#include <stdbool.h>

#define ICM20948_OK       0
#define ICM20948_ERROR    1

typedef enum {
    ICM20948_INTF_SPI,
    ICM20948_INTF_I2C
} ICM20948_Intf_e;

typedef int8_t (*ICM20948_Read_t)(void* intf, uint8_t reg, uint8_t* pRxData, uint16_t len);
typedef int8_t (*ICM20948_Write_t)(void* intf, uint8_t reg, const uint8_t* pTxData, uint16_t len);
typedef void   (*ICM20948_Delay_t)(uint32_t ms);

typedef enum {
    ICM20948_UB0 = 0 << 4,
    ICM20948_UB1 = 1 << 4,
    ICM20948_UB2 = 2 << 4,
    ICM20948_UB3 = 3 << 4
} ICM20948_UserBank_e;

typedef enum {
    ICM20948_GYRO_FS_250DPS = 0,
    ICM20948_GYRO_FS_500DPS,
    ICM20948_GYRO_FS_1000DPS,
    ICM20948_GYRO_FS_2000DPS
} ICM20948_GyroFS_e;

typedef enum {
    ICM20948_ACCEL_FS_2G = 0,
    ICM20948_ACCEL_FS_4G,
    ICM20948_ACCEL_FS_8G,
    ICM20948_ACCEL_FS_16G
} ICM20948_AccelFS_e;

typedef enum {
    AK09916_POWER_DOWN = 0,
    AK09916_SINGLE_MEASURE = 1,
    AK09916_CONT_10HZ = 2,
    AK09916_CONT_20HZ = 4,
    AK09916_CONT_50HZ = 6,
    AK09916_CONT_100HZ = 8
} AK09916_Mode_e;

typedef struct {
    float x;
    float y;
    float z;
} ICM20948_Axes_t;

struct ICM20948_Settings {
    ICM20948_GyroFS_e gyro_fs;
    ICM20948_AccelFS_e accel_fs;
    uint8_t gyro_dlpf;   // low pass filter
    uint8_t accel_dlpf;
    uint8_t gyro_odr;
    uint8_t accel_odr;
};

struct ICM20948_CalibData {
    int16_t gyro_offset[3];
    int16_t accel_offset[3];
    float gyro_scale;
    float accel_scale;
};

typedef struct ICM20948_Device_s {
    void* intf;
    ICM20948_Intf_e intf_type;
    ICM20948_Read_t read;
    ICM20948_Write_t write;
    ICM20948_Delay_t delay;
    struct ICM20948_Settings settings;
    struct ICM20948_CalibData calib;
    // internal state
    uint8_t mag_enabled;
} ICM20948_Device_t;

/* --- Constructor and Core --- */
ICM20948_Device_t ICM20948_NewDevice(void* intf, ICM20948_Intf_e intf_type, ICM20948_Read_t readf, ICM20948_Write_t writef, ICM20948_Delay_t delayf);
int8_t ICM20948_Init(ICM20948_Device_t* dev);
int8_t ICM20948_Test(ICM20948_Device_t* dev);
void   ICM20948_Reset(ICM20948_Device_t* dev);

/* --- Register Access (platform independent) --- */
int8_t ICM20948_ReadRegister(ICM20948_Device_t* dev, ICM20948_UserBank_e bank, uint8_t reg, uint8_t* data, uint8_t len);
int8_t ICM20948_WriteRegister(ICM20948_Device_t* dev, ICM20948_UserBank_e bank, uint8_t reg, const uint8_t* data, uint8_t len);

/* --- Settings & Calibration --- */
void ICM20948_ApplySettings(ICM20948_Device_t* dev, struct ICM20948_Settings* settings);
void ICM20948_GetSettings(ICM20948_Device_t* dev, struct ICM20948_Settings* settings);
void ICM20948_GyroCalibration(ICM20948_Device_t* dev, uint32_t samples);
void ICM20948_AccelCalibration(ICM20948_Device_t* dev, uint32_t samples);

/* --- Gyro & Accel Data --- */
int8_t ICM20948_GetRaw(ICM20948_Device_t* dev, int16_t* accel, int16_t* gyro, int16_t* temp);
void   ICM20948_Get(ICM20948_Device_t* dev, float* accel_g, float* gyro_dps, float* temp_c);

/* --- Power, Low-level, ODR, LPF, etc --- */
void   ICM20948_Sleep(ICM20948_Device_t* dev);
void   ICM20948_Wakeup(ICM20948_Device_t* dev);
void   ICM20948_SetGyroFS(ICM20948_Device_t* dev, ICM20948_GyroFS_e fs);
void   ICM20948_SetAccelFS(ICM20948_Device_t* dev, ICM20948_AccelFS_e fs);
void   ICM20948_SetGyroODR(ICM20948_Device_t* dev, uint8_t odr_div);
void   ICM20948_SetAccelODR(ICM20948_Device_t* dev, uint16_t odr_div);
void   ICM20948_SetGyroLPF(ICM20948_Device_t* dev, uint8_t cfg);
void   ICM20948_SetAccelLPF(ICM20948_Device_t* dev, uint8_t cfg);

/* --- Mag (AK09916) --- */
int8_t ICM20948_MagInit(ICM20948_Device_t* dev);
int8_t ICM20948_MagTest(ICM20948_Device_t* dev);
int8_t ICM20948_MagReadRaw(ICM20948_Device_t* dev, int16_t* mag);
int8_t ICM20948_MagReadUT(ICM20948_Device_t* dev, float* mag_uT);
void   ICM20948_MagSetMode(ICM20948_Device_t* dev, AK09916_Mode_e mode);

#endif // ICM20948_H_
