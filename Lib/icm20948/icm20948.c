#include "icm20948.h"
#include "icm20948_regs.h"

// Select user bank (internal helper)
static int8_t ICM20948_SelectBank(ICM20948_Device_t* dev, ICM20948_UserBank_e bank) {
    uint8_t bank_sel = (uint8_t)bank;
    // Always use REG_BANK_SEL macro for bank selection register
    return dev->write(dev->intf, REG_BANK_SEL, &bank_sel, 1);
}

/* -------- Constructor & Core -------- */
ICM20948_Device_t ICM20948_NewDevice(void* intf, ICM20948_Intf_e intf_type, ICM20948_Read_t readf, ICM20948_Write_t writef, ICM20948_Delay_t delayf) {
    ICM20948_Device_t dev;
    dev.intf      = intf;
    dev.intf_type = intf_type;
    dev.read      = readf;
    dev.write     = writef;
    dev.delay     = delayf;
    dev.mag_enabled = 0;
    for(int i=0; i<3; ++i) {
        dev.calib.gyro_offset[i] = 0;
        dev.calib.accel_offset[i] = 0;
    }
    dev.calib.gyro_scale = 16.4f;
    dev.calib.accel_scale = 2048.0f;
    dev.settings.gyro_fs = ICM20948_GYRO_FS_2000DPS;
    dev.settings.accel_fs = ICM20948_ACCEL_FS_16G;
    dev.settings.gyro_dlpf = 0;
    dev.settings.accel_dlpf = 0;
    dev.settings.gyro_odr = 0;
    dev.settings.accel_odr = 0;
    return dev;
}

int8_t ICM20948_Init(ICM20948_Device_t* dev) {
    if (ICM20948_Test(dev) != ICM20948_OK)
        return ICM20948_ERROR;
    ICM20948_Reset(dev);
    // Wake up the device
    uint8_t val;
    ICM20948_SelectBank(dev, ICM20948_UB0);
    dev->read(dev->intf, B0_PWR_MGMT_1, &val, 1);
    val &= ~0x40; // Clear SLEEP bit
    dev->write(dev->intf, B0_PWR_MGMT_1, &val, 1);
    dev->delay(10);

    // Set default full-scale settings
    ICM20948_SelectBank(dev, ICM20948_UB2);
    dev->read(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    val = (val & ~0x06) | (dev->settings.gyro_fs << 1);
    dev->write(dev->intf, B2_GYRO_CONFIG_1, &val, 1);

    dev->read(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    val = (val & ~0x06) | (dev->settings.accel_fs << 1);
    dev->write(dev->intf, B2_ACCEL_CONFIG, &val, 1);

    return ICM20948_OK;
}

int8_t ICM20948_Test(ICM20948_Device_t* dev) {
    uint8_t id = 0;
    if (ICM20948_SelectBank(dev, ICM20948_UB0) != 0)
        return ICM20948_ERROR;
    if (dev->read(dev->intf, B0_WHO_AM_I, &id, 1) != 0)
        return ICM20948_ERROR;
    return (id == ICM20948_ID) ? ICM20948_OK : ICM20948_ERROR;
}

void ICM20948_Reset(ICM20948_Device_t* dev) {
    ICM20948_SelectBank(dev, ICM20948_UB0);
    uint8_t rst = 0x80; // DEVICE_RESET bit
    dev->write(dev->intf, B0_PWR_MGMT_1, &rst, 1);
    dev->delay(100); // Wait minimum 100ms for hardware reset
}

/* -------- Register Access -------- */
int8_t ICM20948_ReadRegister(ICM20948_Device_t* dev, ICM20948_UserBank_e bank, uint8_t reg, uint8_t* data, uint8_t len) {
    if(ICM20948_SelectBank(dev, bank) != 0) return ICM20948_ERROR;
    return dev->read(dev->intf, reg, data, len);
}

int8_t ICM20948_WriteRegister(ICM20948_Device_t* dev, ICM20948_UserBank_e bank, uint8_t reg, const uint8_t* data, uint8_t len) {
    if(ICM20948_SelectBank(dev, bank) != 0) return ICM20948_ERROR;
    return dev->write(dev->intf, reg, data, len);
}

/* -------- Settings & Calibration -------- */
void ICM20948_ApplySettings(ICM20948_Device_t* dev, struct ICM20948_Settings* settings) {
    uint8_t val;
    dev->settings = *settings;

    ICM20948_SelectBank(dev, ICM20948_UB2);

    // Set Gyro Full Scale
    dev->read(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    val = (val & ~0x06) | (settings->gyro_fs << 1);
    dev->write(dev->intf, B2_GYRO_CONFIG_1, &val, 1);

    // Set Accel Full Scale
    dev->read(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    val = (val & ~0x06) | (settings->accel_fs << 1);
    dev->write(dev->intf, B2_ACCEL_CONFIG, &val, 1);

    // Set Gyro DLPF
    dev->read(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    val = (val & ~0x38) | (settings->gyro_dlpf << 3);
    dev->write(dev->intf, B2_GYRO_CONFIG_1, &val, 1);

    // Set Accel DLPF
    dev->read(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    val = (val & ~0x38) | (settings->accel_dlpf << 3);
    dev->write(dev->intf, B2_ACCEL_CONFIG, &val, 1);

    // Set ODR dividers
    dev->write(dev->intf, B2_GYRO_SMPLRT_DIV, &settings->gyro_odr, 1);
    dev->write(dev->intf, B2_ACCEL_SMPLRT_DIV_1, &settings->accel_odr, 1);
    dev->write(dev->intf, B2_ACCEL_SMPLRT_DIV_2, &settings->accel_odr, 1);

    // Update scale factors
    switch(settings->gyro_fs) {
        case ICM20948_GYRO_FS_250DPS:  dev->calib.gyro_scale = 131.0f; break;
        case ICM20948_GYRO_FS_500DPS:  dev->calib.gyro_scale = 65.5f;  break;
        case ICM20948_GYRO_FS_1000DPS: dev->calib.gyro_scale = 32.8f;  break;
        case ICM20948_GYRO_FS_2000DPS: dev->calib.gyro_scale = 16.4f;  break;
    }
    switch(settings->accel_fs) {
        case ICM20948_ACCEL_FS_2G:  dev->calib.accel_scale = 16384.0f; break;
        case ICM20948_ACCEL_FS_4G:  dev->calib.accel_scale = 8192.0f;  break;
        case ICM20948_ACCEL_FS_8G:  dev->calib.accel_scale = 4096.0f;  break;
        case ICM20948_ACCEL_FS_16G: dev->calib.accel_scale = 2048.0f;  break;
    }
}

void ICM20948_GetSettings(ICM20948_Device_t* dev, struct ICM20948_Settings* settings) {
    uint8_t val;
    ICM20948_SelectBank(dev, ICM20948_UB2);
    dev->read(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    settings->gyro_fs = (val & 0x06) >> 1;
    settings->gyro_dlpf = (val & 0x38) >> 3;
    dev->read(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    settings->accel_fs = (val & 0x06) >> 1;
    settings->accel_dlpf = (val & 0x38) >> 3;
    dev->read(dev->intf, B2_GYRO_SMPLRT_DIV, &val, 1);
    settings->gyro_odr = val;
    dev->read(dev->intf, B2_ACCEL_SMPLRT_DIV_1, &val, 1);
    settings->accel_odr = val;
}

void ICM20948_GyroCalibration(ICM20948_Device_t* dev, uint32_t samples) {
    int64_t bias[3] = {0};
    int16_t accel[3], gyro[3], temp;
    for(uint32_t i=0; i<samples; i++) {
        ICM20948_GetRaw(dev, accel, gyro, &temp);
        bias[0] += gyro[0];
        bias[1] += gyro[1];
        bias[2] += gyro[2];
        dev->delay(2);
    }
    for(int i=0; i<3; ++i)
        dev->calib.gyro_offset[i] = (int16_t)(bias[i] / (int32_t)samples);
}

void ICM20948_AccelCalibration(ICM20948_Device_t* dev, uint32_t samples) {
    int64_t bias[3] = {0};
    int16_t accel[3], gyro[3], temp;
    for(uint32_t i=0; i<samples; i++) {
        ICM20948_GetRaw(dev, accel, gyro, &temp);
        bias[0] += accel[0];
        bias[1] += accel[1];
        bias[2] += accel[2];
        dev->delay(2);
    }
    for(int i=0; i<3; ++i)
        dev->calib.accel_offset[i] = (int16_t)(bias[i] / (int32_t)samples);
}

/* -------- Gyro & Accel Data -------- */
int8_t ICM20948_GetRaw(ICM20948_Device_t* dev, int16_t* accel, int16_t* gyro, int16_t* temp) {
    uint8_t buf[14];
    ICM20948_SelectBank(dev, ICM20948_UB0);
    if(dev->read(dev->intf, B0_ACCEL_XOUT_H, buf, 14) != 0)
        return ICM20948_ERROR;
    // Accel
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    // Gyro
    gyro[0]  = (int16_t)((buf[6] << 8) | buf[7]);
    gyro[1]  = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[2]  = (int16_t)((buf[10]<< 8) | buf[11]);
    // Temp
    if(temp)
        *temp = (int16_t)((buf[12]<<8) | buf[13]);
    return ICM20948_OK;
}

void ICM20948_Get(ICM20948_Device_t* dev, float* accel_g, float* gyro_dps, float* temp_c) {
    int16_t accel[3], gyro[3], temp;
    ICM20948_GetRaw(dev, accel, gyro, &temp);
    for(int i=0; i<3; ++i) {
        accel_g[i] = ((float)(accel[i] - dev->calib.accel_offset[i])) / dev->calib.accel_scale;
        gyro_dps[i] = ((float)(gyro[i] - dev->calib.gyro_offset[i])) / dev->calib.gyro_scale;
    }
    if(temp_c)
        *temp_c = ((float)temp) / 333.87f + 21.0f; // See ICM20948 datasheet
}


// Set the device to sleep mode
void ICM20948_Sleep(ICM20948_Device_t* dev) {
    ICM20948_SelectBank(dev, ICM20948_UB0);
    uint8_t val;
    dev->read(dev->intf, B0_PWR_MGMT_1, &val, 1);
    val |= 0x40; // Set SLEEP bit
    dev->write(dev->intf, B0_PWR_MGMT_1, &val, 1);
}

// Wake up the device from sleep mode
void ICM20948_Wakeup(ICM20948_Device_t* dev) {
    ICM20948_SelectBank(dev, ICM20948_UB0);
    uint8_t val;
    dev->read(dev->intf, B0_PWR_MGMT_1, &val, 1);
    val &= ~0x40; // Clear SLEEP bit
    dev->write(dev->intf, B0_PWR_MGMT_1, &val, 1);
}

// Set gyroscope full scale (FS)
void ICM20948_SetGyroFS(ICM20948_Device_t* dev, ICM20948_GyroFS_e fs) {
    ICM20948_SelectBank(dev, ICM20948_UB2);
    uint8_t val;
    dev->read(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    val = (val & ~0x06) | ((uint8_t)fs << 1);
    dev->write(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    // Update internal scale factor
    switch(fs) {
        case ICM20948_GYRO_FS_250DPS:  dev->calib.gyro_scale = 131.0f; break;
        case ICM20948_GYRO_FS_500DPS:  dev->calib.gyro_scale = 65.5f;  break;
        case ICM20948_GYRO_FS_1000DPS: dev->calib.gyro_scale = 32.8f;  break;
        case ICM20948_GYRO_FS_2000DPS: dev->calib.gyro_scale = 16.4f;  break;
    }
    dev->settings.gyro_fs = fs;
}

// Set accelerometer full scale (FS)
void ICM20948_SetAccelFS(ICM20948_Device_t* dev, ICM20948_AccelFS_e fs) {
    ICM20948_SelectBank(dev, ICM20948_UB2);
    uint8_t val;
    dev->read(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    val = (val & ~0x06) | ((uint8_t)fs << 1);
    dev->write(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    // Update internal scale factor
    switch(fs) {
        case ICM20948_ACCEL_FS_2G:  dev->calib.accel_scale = 16384.0f; break;
        case ICM20948_ACCEL_FS_4G:  dev->calib.accel_scale = 8192.0f;  break;
        case ICM20948_ACCEL_FS_8G:  dev->calib.accel_scale = 4096.0f;  break;
        case ICM20948_ACCEL_FS_16G: dev->calib.accel_scale = 2048.0f;  break;
    }
    dev->settings.accel_fs = fs;
}

// Set gyroscope Output Data Rate (ODR) divider
void ICM20948_SetGyroODR(ICM20948_Device_t* dev, uint8_t odr_div) {
    ICM20948_SelectBank(dev, ICM20948_UB2);
    dev->write(dev->intf, B2_GYRO_SMPLRT_DIV, &odr_div, 1);
    dev->settings.gyro_odr = odr_div;
}

// Set accelerometer Output Data Rate (ODR) divider
void ICM20948_SetAccelODR(ICM20948_Device_t* dev, uint16_t odr_div) {
    ICM20948_SelectBank(dev, ICM20948_UB2);
    uint8_t div_1 = (uint8_t)(odr_div >> 8);
    uint8_t div_2 = (uint8_t)(odr_div & 0x0F);
    dev->write(dev->intf, B2_ACCEL_SMPLRT_DIV_1, &div_1, 1);
    dev->write(dev->intf, B2_ACCEL_SMPLRT_DIV_2, &div_2, 1);
    dev->settings.accel_odr = odr_div;
}

// Set gyroscope Digital Low Pass Filter (DLPF) configuration
void ICM20948_SetGyroLPF(ICM20948_Device_t* dev, uint8_t cfg) {
    ICM20948_SelectBank(dev, ICM20948_UB2);
    uint8_t val;
    dev->read(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    val = (val & ~0x38) | ((cfg & 0x07) << 3);
    dev->write(dev->intf, B2_GYRO_CONFIG_1, &val, 1);
    dev->settings.gyro_dlpf = cfg;
}

// Set accelerometer Digital Low Pass Filter (DLPF) configuration
void ICM20948_SetAccelLPF(ICM20948_Device_t* dev, uint8_t cfg) {
    ICM20948_SelectBank(dev, ICM20948_UB2);
    uint8_t val;
    dev->read(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    val = (val & ~0x38) | ((cfg & 0x07) << 3);
    dev->write(dev->intf, B2_ACCEL_CONFIG, &val, 1);
    dev->settings.accel_dlpf = cfg;
}

// Helper: Write one byte to AK09916
static void ICM20948_WriteAK09916(ICM20948_Device_t* dev, uint8_t reg, uint8_t val) {
    // Set up Slave 0 for write (W=0)
    uint8_t addr = MAG_SLAVE_ADDR;
    ICM20948_SelectBank(dev, ICM20948_UB3);
    dev->write(dev->intf, B3_I2C_SLV0_ADDR, &addr, 1);
    dev->write(dev->intf, B3_I2C_SLV0_REG, &reg, 1);
    dev->write(dev->intf, B3_I2C_SLV0_DO, &val, 1);
    uint8_t ctrl = 0x81; // Enable, 1 byte
    dev->write(dev->intf, B3_I2C_SLV0_CTRL, &ctrl, 1);
    dev->delay(1);
}

// Helper: Read one byte from AK09916
static uint8_t ICM20948_ReadAK09916(ICM20948_Device_t* dev, uint8_t reg) {
    // Set up Slave 0 for read (R=1)
    uint8_t addr = 0x80 | MAG_SLAVE_ADDR;
    ICM20948_SelectBank(dev, ICM20948_UB3);
    dev->write(dev->intf, B3_I2C_SLV0_ADDR, &addr, 1);
    dev->write(dev->intf, B3_I2C_SLV0_REG, &reg, 1);
    uint8_t ctrl = 0x81; // Enable, 1 byte
    dev->write(dev->intf, B3_I2C_SLV0_CTRL, &ctrl, 1);
    dev->delay(1);

    ICM20948_SelectBank(dev, ICM20948_UB0);
    uint8_t val = 0;
    dev->read(dev->intf, B0_EXT_SLV_SENS_DATA_00, &val, 1);
    return val;
}

// Helper: Read multiple bytes from AK09916
static void ICM20948_ReadAK09916Multi(ICM20948_Device_t* dev, uint8_t reg, uint8_t* data, uint8_t len) {
    uint8_t addr = 0x80 | MAG_SLAVE_ADDR;
    ICM20948_SelectBank(dev, ICM20948_UB3);
    dev->write(dev->intf, B3_I2C_SLV0_ADDR, &addr, 1);
    dev->write(dev->intf, B3_I2C_SLV0_REG, &reg, 1);
    uint8_t ctrl = 0x80 | len; // Enable, N bytes
    dev->write(dev->intf, B3_I2C_SLV0_CTRL, &ctrl, 1);
    dev->delay(1);

    ICM20948_SelectBank(dev, ICM20948_UB0);
    dev->read(dev->intf, B0_EXT_SLV_SENS_DATA_00, data, len);
}

/* ---- Magnetometer (AK09916) Functions ---- */

// Initialize AK09916, enable continuous measurement mode
int8_t ICM20948_MagInit(ICM20948_Device_t* dev) {
    // Reset I2C master and enable (mandatory for mag access)
    ICM20948_SelectBank(dev, ICM20948_UB0);
    uint8_t val;
    dev->read(dev->intf, B0_USER_CTRL, &val, 1);
    val |= 0x02; // I2C_MST_RST
    dev->write(dev->intf, B0_USER_CTRL, &val, 1);

    dev->read(dev->intf, B0_USER_CTRL, &val, 1);
    val |= 0x20; // I2C_MST_EN
    dev->write(dev->intf, B0_USER_CTRL, &val, 1);
    dev->delay(2);

    // Set I2C master clock speed
    ICM20948_SelectBank(dev, ICM20948_UB3);
    dev->read(dev->intf, B3_I2C_MST_CTRL, &val, 1);
    val = (val & 0xF0) | 0x07; // 345.6 kHz, typical value
    dev->write(dev->intf, B3_I2C_MST_CTRL, &val, 1);

    // Check if AK09916 is present
    if(ICM20948_MagTest(dev) != ICM20948_OK)
        return ICM20948_ERROR;

    // Soft reset AK09916
    ICM20948_WriteAK09916(dev, MAG_CNTL3, 0x01);
    dev->delay(2);

    // Set default mode: continuous measurement 100Hz
    ICM20948_MagSetMode(dev, AK09916_CONT_100HZ);

    dev->mag_enabled = 1;
    return ICM20948_OK;
}

// Test if AK09916 responds with correct device ID
int8_t ICM20948_MagTest(ICM20948_Device_t* dev) {
    uint8_t id = ICM20948_ReadAK09916(dev, MAG_WIA2);
    return (id == AK09916_ID) ? ICM20948_OK : ICM20948_ERROR;
}

// Set AK09916 operating mode (e.g. power down, continuous, etc)
void ICM20948_MagSetMode(ICM20948_Device_t* dev, AK09916_Mode_e mode) {
    ICM20948_WriteAK09916(dev, MAG_CNTL2, mode);
    dev->delay(1);
}

// Read raw XYZ data from AK09916, return 0 on success
int8_t ICM20948_MagReadRaw(ICM20948_Device_t* dev, int16_t* mag) {
    uint8_t drdy = ICM20948_ReadAK09916(dev, MAG_ST1) & 0x01;
    if(!drdy)
        return ICM20948_ERROR;

    uint8_t buf[6];
    ICM20948_ReadAK09916Multi(dev, MAG_HXL, buf, 6);

    uint8_t hofl = ICM20948_ReadAK09916(dev, MAG_ST2) & 0x08;
    if(hofl)
        return ICM20948_ERROR;

    // Little endian order!
    mag[0] = (int16_t)((buf[1]<<8) | buf[0]);
    mag[1] = (int16_t)((buf[3]<<8) | buf[2]);
    mag[2] = (int16_t)((buf[5]<<8) | buf[4]);

    return ICM20948_OK;
}

// Read XYZ data from AK09916 and convert to microtesla (μT)
int8_t ICM20948_MagReadUT(ICM20948_Device_t* dev, float* mag_uT) {
    int16_t mag_raw[3];
    if(ICM20948_MagReadRaw(dev, mag_raw) != ICM20948_OK)
        return ICM20948_ERROR;
    // Sensitivity = 0.15 μT/LSB for AK09916
    mag_uT[0] = mag_raw[0] * 0.15f;
    mag_uT[1] = mag_raw[1] * 0.15f;
    mag_uT[2] = mag_raw[2] * 0.15f;
    return ICM20948_OK;
}
