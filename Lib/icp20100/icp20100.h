/**
 * @file icp20100.h
 * @brief ICP-20100 High Accuracy Barometric Pressure and Temperature Sensor Driver
 * @version 1.0
 * @date 2025
 * 
 * Driver follows Bosch Sensortec standards for MCU-independent sensor libraries
 * Compatible with ICP-20100 datasheet revision 1.4
 */

#ifndef ICP20100_H_
#define ICP20100_H_

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/**\name        Header Files                                               */
/****************************************************************************/
#include <stdint.h>
#include <stddef.h>

/****************************************************************************/
/**\name        Constant Definitions                                       */
/****************************************************************************/

/** Device identification */
#define ICP20100_CHIP_ID                    UINT8_C(0x63)
#define ICP20100_VERSION_A                  UINT8_C(0x00)
#define ICP20100_VERSION_B                  UINT8_C(0xB2)

/** Interface selection */
#define ICP20100_I2C_ADDR_LOW               UINT8_C(0x63)
#define ICP20100_I2C_ADDR_HIGH              UINT8_C(0x64)
#define ICP20100_SPI_3_WIRE                 UINT8_C(0x01)
#define ICP20100_SPI_4_WIRE                 UINT8_C(0x00)

/** Return codes */
#define ICP20100_OK                         INT8_C(0)
#define ICP20100_E_NULL_PTR                 INT8_C(-1)
#define ICP20100_E_COMM_FAIL                INT8_C(-2)
#define ICP20100_E_DEV_NOT_FOUND            INT8_C(-3)
#define ICP20100_E_INVALID_CONFIG           INT8_C(-4)
#define ICP20100_E_BOOT_FAILED              INT8_C(-5)
#define ICP20100_W_PARTIAL_READ             INT8_C(1)

/** Register addresses */
#define ICP20100_REG_TRIM1_MSB              UINT8_C(0x05)
#define ICP20100_REG_TRIM2_LSB              UINT8_C(0x06)
#define ICP20100_REG_TRIM2_MSB              UINT8_C(0x07)
#define ICP20100_REG_DEVICE_ID              UINT8_C(0x0C)
#define ICP20100_REG_IO_DRIVE_STRENGTH      UINT8_C(0x0D)
#define ICP20100_REG_OTP_CONFIG1            UINT8_C(0xAC)
#define ICP20100_REG_MASTER_LOCK            UINT8_C(0xBE)
#define ICP20100_REG_OTP_STATUS2            UINT8_C(0xBF)
#define ICP20100_REG_MODE_SELECT            UINT8_C(0xC0)
#define ICP20100_REG_INTERRUPT_STATUS       UINT8_C(0xC1)
#define ICP20100_REG_INTERRUPT_MASK         UINT8_C(0xC2)
#define ICP20100_REG_FIFO_CONFIG            UINT8_C(0xC3)
#define ICP20100_REG_FIFO_FILL              UINT8_C(0xC4)
#define ICP20100_REG_SPI_MODE               UINT8_C(0xC5)
#define ICP20100_REG_PRESS_ABS_LSB          UINT8_C(0xC7)
#define ICP20100_REG_PRESS_ABS_MSB          UINT8_C(0xC8)
#define ICP20100_REG_PRESS_DELTA_LSB        UINT8_C(0xC9)
#define ICP20100_REG_PRESS_DELTA_MSB        UINT8_C(0xCA)
#define ICP20100_REG_DEVICE_STATUS          UINT8_C(0xCD)
#define ICP20100_REG_I3C_INFO               UINT8_C(0xCE)
#define ICP20100_REG_VERSION                UINT8_C(0xD3)
#define ICP20100_REG_PRESS_DATA_0           UINT8_C(0xFA)
#define ICP20100_REG_PRESS_DATA_1           UINT8_C(0xFB)
#define ICP20100_REG_PRESS_DATA_2           UINT8_C(0xFC)
#define ICP20100_REG_TEMP_DATA_0            UINT8_C(0xFD)
#define ICP20100_REG_TEMP_DATA_1            UINT8_C(0xFE)
#define ICP20100_REG_TEMP_DATA_2            UINT8_C(0xFF)

/** OTP registers for boot sequence */
#define ICP20100_REG_OTP_MR_LSB             UINT8_C(0xAD)
#define ICP20100_REG_OTP_MR_MSB             UINT8_C(0xAE)
#define ICP20100_REG_OTP_MRA_LSB            UINT8_C(0xAF)
#define ICP20100_REG_OTP_MRA_MSB            UINT8_C(0xB0)
#define ICP20100_REG_OTP_MRB_LSB            UINT8_C(0xB1)
#define ICP20100_REG_OTP_MRB_MSB            UINT8_C(0xB2)
#define ICP20100_REG_OTP_ADDRESS            UINT8_C(0xB5)
#define ICP20100_REG_OTP_COMMAND            UINT8_C(0xB6)
#define ICP20100_REG_OTP_RDATA              UINT8_C(0xB8)
#define ICP20100_REG_OTP_STATUS             UINT8_C(0xB9)
#define ICP20100_REG_OTP_DBG2               UINT8_C(0xBC)

/** Bit masks and positions */
#define ICP20100_BOOT_UP_STATUS_MASK        UINT8_C(0x01)
#define ICP20100_POWER_MODE_MASK            UINT8_C(0x04)
#define ICP20100_MEAS_CONFIG_MASK           UINT8_C(0xE0)
#define ICP20100_FORCED_MEAS_TRIGGER_MASK   UINT8_C(0x10)
#define ICP20100_MEAS_MODE_MASK             UINT8_C(0x08)
#define ICP20100_FIFO_READOUT_MODE_MASK     UINT8_C(0x03)
#define ICP20100_MASTER_LOCK_UNLOCK         UINT8_C(0x1F)
#define ICP20100_OTP_ENABLE_MASK            UINT8_C(0x01)
#define ICP20100_OTP_WRITE_SWITCH_MASK      UINT8_C(0x02)
#define ICP20100_OTP_RESET_MASK             UINT8_C(0x80)
#define ICP20100_OTP_BUSY_MASK              UINT8_C(0x01)
#define ICP20100_FIFO_FLUSH_MASK            UINT8_C(0x80)
#define ICP20100_FIFO_LEVEL_MASK            UINT8_C(0x1F)
#define ICP20100_FIFO_EMPTY_MASK            UINT8_C(0x40)
#define ICP20100_FIFO_FULL_MASK             UINT8_C(0x20)
#define ICP20100_MODE_SYNC_STATUS_MASK      UINT8_C(0x01)
#define ICP20100_PRESS_DATA_MASK            UINT8_C(0x0F)

/** Measurement modes */
#define ICP20100_MODE_0                     UINT8_C(0x00)
#define ICP20100_MODE_1                     UINT8_C(0x01)
#define ICP20100_MODE_2                     UINT8_C(0x02)
#define ICP20100_MODE_3                     UINT8_C(0x03)
#define ICP20100_MODE_4                     UINT8_C(0x04)

/** FIFO readout modes */
#define ICP20100_FIFO_PRESSURE_FIRST        UINT8_C(0x00)
#define ICP20100_FIFO_TEMP_ONLY             UINT8_C(0x01)

/** Measurement modes */
#define ICP20100_MEAS_MODE_FORCED           UINT8_C(0x00)
#define ICP20100_MEAS_MODE_CONTINUOUS       UINT8_C(0x01)

/** Power modes */
#define ICP20100_POWER_MODE_NORMAL          UINT8_C(0x00)
#define ICP20100_POWER_MODE_ACTIVE          UINT8_C(0x01)

/** Drive strength settings for different supply voltages */
#define ICP20100_DRIVE_STRENGTH_1_8V_2MA    UINT8_C(0x00)
#define ICP20100_DRIVE_STRENGTH_1_8V_4MA    UINT8_C(0x01)
#define ICP20100_DRIVE_STRENGTH_1_8V_8MA    UINT8_C(0x02)
#define ICP20100_DRIVE_STRENGTH_1_8V_12MA   UINT8_C(0x03)
#define ICP20100_DRIVE_STRENGTH_1_2V_2MA    UINT8_C(0x04)
#define ICP20100_DRIVE_STRENGTH_1_2V_4MA    UINT8_C(0x05)
#define ICP20100_DRIVE_STRENGTH_1_2V_6MA    UINT8_C(0x06)
#define ICP20100_DRIVE_STRENGTH_1_2V_8MA    UINT8_C(0x07)
#define ICP20100_DRIVE_STRENGTH_MASK        UINT8_C(0x07)

/** Conversion constants - Following software_imp.md formulas */
#define ICP20100_TEMP_SCALE_FACTOR          (65.0f / (1 << 18))
#define ICP20100_TEMP_OFFSET                (25.0f)
#define ICP20100_PRESS_SCALE_FACTOR         (40.0f / (1 << 17))  /* Results in kPa */
#define ICP20100_PRESS_OFFSET               (70.0f)               /* In kPa */

/** Timing constants (in milliseconds/microseconds) */
#define ICP20100_POWER_UP_TIME_MS           UINT8_C(4)
#define ICP20100_OTP_WAIT_TIME_US           UINT8_C(10)
#define ICP20100_OTP_BUSY_TIMEOUT_MS        UINT8_C(100)

/** Interrupt bit masks */
#define ICP20100_INT_FIFO_OVERFLOW_MASK     UINT8_C(0x01)
#define ICP20100_INT_FIFO_UNDERFLOW_MASK    UINT8_C(0x02)
#define ICP20100_INT_FIFO_WMK_HIGH_MASK     UINT8_C(0x04)
#define ICP20100_INT_FIFO_WMK_LOW_MASK      UINT8_C(0x08)
#define ICP20100_INT_PRESS_ABS_MASK         UINT8_C(0x20)
#define ICP20100_INT_PRESS_DELTA_MASK       UINT8_C(0x40)

/** Bit position definitions */
#define ICP20100_MEAS_CONFIG_POS            UINT8_C(5)
#define ICP20100_FORCED_MEAS_TRIGGER_POS    UINT8_C(4)
#define ICP20100_MEAS_MODE_POS              UINT8_C(3)
#define ICP20100_POWER_MODE_POS             UINT8_C(2)
#define ICP20100_FIFO_READOUT_MODE_POS      UINT8_C(0)

/****************************************************************************/
/**\name        Type Definitions                                           */
/****************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef int8_t (*icp20100_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[in]     reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef int8_t (*icp20100_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param period - The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
typedef void (*icp20100_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/*!
 * @brief Interface selection enums
 */
enum icp20100_intf {
    ICP20100_I2C_INTF,   /*!< I2C interface */
    ICP20100_SPI_INTF    /*!< SPI interface */
};

/*!
 * @brief Sensor measurement configuration structure
 */
struct icp20100_config {
    /*! Measurement mode (MODE_0 to MODE_4) */
    uint8_t meas_mode;
    
    /*! Measurement type (forced or continuous) */
    uint8_t meas_type;
    
    /*! Power mode (normal or active) */
    uint8_t power_mode;
    
    /*! FIFO readout mode */
    uint8_t fifo_mode;
    
    /*! Drive strength setting */
    uint8_t drive_strength;
};

/*!
 * @brief Sensor data structure
 */
struct icp20100_data {
    /*! Pressure in kPa (following software_imp.md format) */
    float pressure;
    
    /*! Temperature in degree Celsius */
    float temperature;
    
    /*! Raw pressure data */
    int32_t pressure_raw;
    
    /*! Raw temperature data */
    int32_t temperature_raw;
};

/*!
 * @brief Interrupt configuration structure
 */
struct icp20100_int_config {
    /*! FIFO watermark high interrupt enable */
    uint8_t fifo_wmk_high_en;
    
    /*! FIFO watermark low interrupt enable */
    uint8_t fifo_wmk_low_en;
    
    /*! FIFO overflow interrupt enable */
    uint8_t fifo_overflow_en;
    
    /*! FIFO underflow interrupt enable */
    uint8_t fifo_underflow_en;
    
    /*! Pressure absolute threshold interrupt enable */
    uint8_t press_abs_en;
    
    /*! Pressure delta threshold interrupt enable */
    uint8_t press_delta_en;
    
    /*! FIFO watermark high level */
    uint8_t fifo_wmk_high;
    
    /*! FIFO watermark low level */
    uint8_t fifo_wmk_low;
};

/*!
 * @brief ICP20100 device structure
 */
struct icp20100_dev {
    /*! Chip ID */
    uint8_t chip_id;
    
    /*! Device version */
    uint8_t version;
    
    /*! Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr;
    
    /*! Interface Selection */
    enum icp20100_intf intf;
    
    /*! Read function pointer */
    icp20100_read_fptr_t read;
    
    /*! Write function pointer */
    icp20100_write_fptr_t write;
    
    /*! Delay function pointer */
    icp20100_delay_us_fptr_t delay_us;
    
    /*! Current sensor configuration */
    struct icp20100_config config;
    
    /*! Boot sequence completed flag */
    uint8_t boot_completed;
};

/****************************************************************************/
/**\name        Function Declarations                                      */
/****************************************************************************/

/*!
 * @brief This API reads the chip-id and version of the sensor which is the first step to
 * verify the sensor and also it configures the read mechanism of SPI and I2C
 * interface.
 *
 * @param[in,out] dev : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 * @retval ICP20100_E_DEV_NOT_FOUND -> Device not found
 */
int8_t icp20100_init(struct icp20100_dev *dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of icp20100_dev.
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_soft_reset(struct icp20100_dev *dev);

/*!
 * @brief This API executes the boot sequence required for proper sensor operation
 *
 * @param[in] dev : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 * @retval ICP20100_E_BOOT_FAILED -> Boot sequence failed
 */
int8_t icp20100_boot_sequence(struct icp20100_dev *dev);

/*!
 * @brief This API sets the measurement configuration of the sensor
 *
 * @param[in] config : Structure instance of icp20100_config
 * @param[in] dev    : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 * @retval ICP20100_E_INVALID_CONFIG -> Invalid configuration
 */
int8_t icp20100_set_config(const struct icp20100_config *config, struct icp20100_dev *dev);

/*!
 * @brief This API gets the measurement configuration of the sensor
 *
 * @param[out] config : Structure instance of icp20100_config
 * @param[in]  dev    : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_get_config(struct icp20100_config *config, struct icp20100_dev *dev);

/*!
 * @brief This API reads the pressure and temperature data from the sensor
 *
 * @param[out] data : Structure instance of icp20100_data
 * @param[in]  dev  : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_get_data(struct icp20100_data *data, struct icp20100_dev *dev);

/*!
 * @brief This API triggers a forced measurement
 *
 * @param[in] dev : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_trigger_measurement(struct icp20100_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor
 *
 * @param[in] power_mode : Power mode to be set
 * @param[in] dev        : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_set_power_mode(uint8_t power_mode, struct icp20100_dev *dev);

/*!
 * @brief This API gets the power mode of the sensor
 *
 * @param[out] power_mode : Power mode of the sensor
 * @param[in]  dev        : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_get_power_mode(uint8_t *power_mode, struct icp20100_dev *dev);

/*!
 * @brief This API configures the interrupt settings
 *
 * @param[in] int_config : Structure instance of icp20100_int_config
 * @param[in] dev        : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_set_int_config(const struct icp20100_int_config *int_config, struct icp20100_dev *dev);

/*!
 * @brief This API reads the interrupt status
 *
 * @param[out] int_status : Interrupt status register value
 * @param[in]  dev        : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_get_int_status(uint8_t *int_status, struct icp20100_dev *dev);

/*!
 * @brief This API clears the interrupt status
 *
 * @param[in] int_status : Interrupt status bits to clear
 * @param[in] dev        : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_clear_int_status(uint8_t int_status, struct icp20100_dev *dev);

/*!
 * @brief This API gets the FIFO length
 *
 * @param[out] fifo_length : Number of samples in FIFO
 * @param[in]  dev         : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_get_fifo_length(uint8_t *fifo_length, struct icp20100_dev *dev);

/*!
 * @brief This API reads data from FIFO
 *
 * @param[out] data    : Structure array of icp20100_data
 * @param[in]  length  : Number of samples to read
 * @param[in]  dev     : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_read_fifo(struct icp20100_data *data, uint8_t length, struct icp20100_dev *dev);

/*!
 * @brief This API flushes the FIFO
 *
 * @param[in] dev : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_flush_fifo(struct icp20100_dev *dev);

/*!
 * @brief This API performs FIR filter settling procedure as described in software_imp.md
 * When using modes 0-3 with FIR filter, first 14 samples should be discarded
 *
 * @param[in] dev : Structure instance of icp20100_dev
 *
 * @return Result of API execution status
 * @retval ICP20100_OK -> Success
 * @retval ICP20100_E_NULL_PTR -> Null pointer error
 * @retval ICP20100_E_COMM_FAIL -> Communication failure
 */
int8_t icp20100_fir_filter_settling(struct icp20100_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* ICP20100_H_ */