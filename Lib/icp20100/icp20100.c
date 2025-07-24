/**
 * @file icp20100.c
 * @brief ICP-20100 High Accuracy Barometric Pressure and Temperature Sensor Driver
 * @version 1.0
 * @date 2025
 * 
 * Driver follows Bosch Sensortec standards for MCU-independent sensor libraries
 * Compatible with ICP-20100 datasheet revision 1.4
 */

/****************************************************************************/
/**\name        Header Files                                               */
/****************************************************************************/
#include "icp20100.h"

/****************************************************************************/
/**\name        Local Function Declarations                                */
/****************************************************************************/

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct icp20100_dev *dev);

/*!
 * @brief This internal API reads the chip-id and version of the sensor.
 */
static int8_t read_chip_id(struct icp20100_dev *dev);

/*!
 * @brief This internal API checks if boot sequence is required.
 */
static int8_t check_boot_required(struct icp20100_dev *dev, uint8_t *boot_required);

/*!
 * @brief This internal API executes the OTP read sequence.
 */
static int8_t otp_read_sequence(struct icp20100_dev *dev);

/*!
 * @brief This internal API prepares the device for OTP operations.
 */
static int8_t otp_prepare_device(struct icp20100_dev *dev);

/*!
 * @brief This internal API enables OTP interface.
 */
static int8_t otp_enable_interface(struct icp20100_dev *dev);

/*!
 * @brief This internal API configures OTP redundant read registers.
 */
static int8_t otp_configure_redundant_read(struct icp20100_dev *dev);

/*!
 * @brief This internal API reads calibration data from OTP.
 */
static int8_t otp_read_calibration_data(uint8_t *offset, uint8_t *gain, uint8_t *hfosc, struct icp20100_dev *dev);

/*!
 * @brief This internal API writes calibration data to trim registers.
 */
static int8_t otp_write_trim_registers(uint8_t offset, uint8_t gain, uint8_t hfosc, struct icp20100_dev *dev);

/*!
 * @brief This internal API disables OTP interface and locks registers.
 */
static int8_t otp_finalize_sequence(struct icp20100_dev *dev);

/*!
 * @brief This internal API reads OTP data from specified address.
 */
static int8_t read_otp_data(uint8_t address, uint8_t *data, struct icp20100_dev *dev);

/*!
 * @brief This internal API waits for OTP operation to complete.
 */
static int8_t wait_otp_ready(struct icp20100_dev *dev);

/*!
 * @brief This internal API converts raw pressure data to Pascal.
 */
static float convert_pressure(int32_t raw_pressure);

/*!
 * @brief This internal API converts raw temperature data to Celsius.
 */
static float convert_temperature(int32_t raw_temperature);

/*!
 * @brief This internal API reads raw data from pressure and temperature registers.
 */
static int8_t read_raw_data(int32_t *pressure_raw, int32_t *temperature_raw, struct icp20100_dev *dev);

/*!
 * @brief This internal API waits for mode synchronization.
 */
static int8_t wait_mode_sync(struct icp20100_dev *dev);

/****************************************************************************/
/**\name        Function Definitions                                       */
/****************************************************************************/

/*!
 * @brief This API reads the chip-id and version of the sensor which is the first step to
 * verify the sensor and also it configures the read mechanism of SPI and I2C
 * interface.
 */
int8_t icp20100_init(struct icp20100_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        /* For I2C interface, initialize with dummy transaction to enable interface */
        if (dev->intf == ICP20100_I2C_INTF)
        {
            uint8_t dummy_reg = ICP20100_REG_DEVICE_ID;
            uint8_t dummy_data;
            
            /* Perform dummy read to initialize I2C interface */
            rslt = dev->read(dummy_reg, &dummy_data, 1, dev->intf_ptr);
        }

        if (rslt == ICP20100_OK)
        {
            /* Read and validate chip ID */
            rslt = read_chip_id(dev);
        }

        if (rslt == ICP20100_OK)
        {
            /* Initialize default configuration */
            dev->config.meas_mode = ICP20100_MODE_0;
            dev->config.meas_type = ICP20100_MEAS_MODE_CONTINUOUS;
            dev->config.power_mode = ICP20100_POWER_MODE_NORMAL;
            dev->config.fifo_mode = ICP20100_FIFO_PRESSURE_FIRST;
            dev->config.drive_strength = ICP20100_DRIVE_STRENGTH_1_8V_2MA;
            dev->boot_completed = 0;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API reads OTP data from specified address.
 */
static int8_t read_otp_data(uint8_t address, uint8_t *data, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t command_reg;

    /* Set OTP address */
    rslt = dev->write(ICP20100_REG_OTP_ADDRESS, &address, 1, dev->intf_ptr);

    if (rslt == ICP20100_OK)
    {
        /* Read current command register and set read command */
        rslt = dev->read(ICP20100_REG_OTP_COMMAND, &command_reg, 1, dev->intf_ptr);

        if (rslt == ICP20100_OK)
        {
            command_reg = (command_reg & 0xF0) | 0x01; /* Set read command */
            rslt = dev->write(ICP20100_REG_OTP_COMMAND, &command_reg, 1, dev->intf_ptr);
        }
    }

    if (rslt == ICP20100_OK)
    {
        /* Wait for OTP operation to complete */
        rslt = wait_otp_ready(dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Read data from OTP_RDATA register */
        rslt = dev->read(ICP20100_REG_OTP_RDATA, data, 1, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This internal API waits for OTP operation to complete.
 */
static int8_t wait_otp_ready(struct icp20100_dev *dev)
{
    int8_t rslt = ICP20100_OK;
    uint8_t status;
    uint32_t timeout_count = 0;
    uint32_t max_timeout = ICP20100_OTP_BUSY_TIMEOUT_MS * 100; /* Convert to ~10us intervals */

    do {
        rslt = dev->read(ICP20100_REG_OTP_STATUS, &status, 1, dev->intf_ptr);

        if (rslt != ICP20100_OK)
        {
            break;
        }

        if ((status & ICP20100_OTP_BUSY_MASK) == 0)
        {
            break; /* OTP operation completed */
        }

        dev->delay_us(10, dev->intf_ptr); /* Small delay */
        timeout_count++;

    } while (timeout_count < max_timeout);

    if (timeout_count >= max_timeout)
    {
        rslt = ICP20100_E_COMM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This internal API converts raw pressure data to Pascal.
 */
static float convert_pressure(int32_t raw_pressure)
{
    /* Sign extend 20-bit value to 32-bit */
    if (raw_pressure & 0x80000)
    {
        raw_pressure |= 0xFFF00000;
    }

    /* Apply conversion formula: P = (POUT/2^17)*40kPa + 70kPa */
    return ((float)raw_pressure * ICP20100_PRESS_SCALE_FACTOR) + ICP20100_PRESS_OFFSET;
}

/*!
 * @brief This internal API converts raw temperature data to Celsius.
 */
static float convert_temperature(int32_t raw_temperature)
{
    /* Sign extend 20-bit value to 32-bit */
    if (raw_temperature & 0x80000)
    {
        raw_temperature |= 0xFFF00000;
    }

    /* Apply conversion formula: T = (TOUT/2^18)*65C + 25C */
    return ((float)raw_temperature * ICP20100_TEMP_SCALE_FACTOR) + ICP20100_TEMP_OFFSET;
}

/*!
 * @brief This internal API reads raw data from pressure and temperature registers.
 */
static int8_t read_raw_data(int32_t *pressure_raw, int32_t *temperature_raw, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t data[3];

    /* Read temperature data (3 bytes) */
    rslt = dev->read(ICP20100_REG_TEMP_DATA_0, data, 3, dev->intf_ptr);

    if (rslt == ICP20100_OK)
    {
        /* Combine 3 bytes into 20-bit value */
        *temperature_raw = ((int32_t)data[0] << 16) |
                          ((int32_t)data[1] << 8) |
                          data[2];

        /* Read pressure data (3 bytes) */
        rslt = dev->read(ICP20100_REG_PRESS_DATA_0, data, 3, dev->intf_ptr);

        if (rslt == ICP20100_OK)
        {
            /* Combine 3 bytes into 20-bit value */
            *pressure_raw = ((int32_t)data[0] << 16) |
                           ((int32_t)data[1] << 8) |
                           data[2];
        }
    }

    return rslt;
}

/*!
 * @brief This internal API waits for mode synchronization.
 */
static int8_t wait_mode_sync(struct icp20100_dev *dev)
{
    int8_t rslt = ICP20100_OK;
    uint8_t status;
    uint32_t timeout_count = 0;
    uint32_t max_timeout = 1000; /* 10ms timeout with 10us intervals */

    do {
        rslt = dev->read(ICP20100_REG_DEVICE_STATUS, &status, 1, dev->intf_ptr);

        if (rslt != ICP20100_OK)
        {
            break;
        }

        if (status & ICP20100_MODE_SYNC_STATUS_MASK)
        {
            break; /* Mode synchronization completed */
        }

        dev->delay_us(10, dev->intf_ptr);
        timeout_count++;

    } while (timeout_count < max_timeout);

    if (timeout_count >= max_timeout)
    {
        rslt = ICP20100_E_COMM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t icp20100_soft_reset(struct icp20100_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        /* Note: ICP-20100 doesn't have a software reset register.
         * Reset is achieved through power cycling or boot sequence. */
        dev->boot_completed = 0;
    }

    return rslt;
}

/*!
 * @brief This API executes the boot sequence required for proper sensor operation
 */
int8_t icp20100_boot_sequence(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t boot_required = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        /* Check if boot sequence is required */
        rslt = check_boot_required(dev, &boot_required);
    }

    if ((rslt == ICP20100_OK) && boot_required)
    {
        /* Execute OTP read sequence for calibration data */
        rslt = otp_read_sequence(dev);
        
        if (rslt == ICP20100_OK)
        {
            /* Mark boot sequence as completed */
            uint8_t boot_status = 0x01;
            rslt = dev->write(ICP20100_REG_OTP_STATUS2, &boot_status, 1, dev->intf_ptr);
            
            if (rslt == ICP20100_OK)
            {
                dev->boot_completed = 1;
            }
        }
    }
    else if (rslt == ICP20100_OK)
    {
        /* Boot sequence already completed */
        dev->boot_completed = 1;
    }

    return rslt;
}

/*!
 * @brief This API sets the measurement configuration of the sensor
 */
int8_t icp20100_set_config(const struct icp20100_config *config, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && config == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Validate configuration parameters */
        if (config->meas_mode > ICP20100_MODE_4 ||
            config->meas_type > ICP20100_MEAS_MODE_CONTINUOUS ||
            config->power_mode > ICP20100_POWER_MODE_ACTIVE ||
            config->fifo_mode > ICP20100_FIFO_PRESSURE_ONLY)
        {
            rslt = ICP20100_E_INVALID_CONFIG;
        }
    }

    if (rslt == ICP20100_OK)
    {
        /* Wait for mode synchronization before changing configuration */
        rslt = wait_mode_sync(dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Configure measurement mode and settings */
        reg_data = (config->meas_mode << 5) |
                   (config->meas_type << 3) |
                   (config->power_mode << 2) |
                   config->fifo_mode;
        
        rslt = dev->write(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Set drive strength */
        rslt = dev->write(ICP20100_REG_IO_DRIVE_STRENGTH, &config->drive_strength, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Update device configuration */
        dev->config = *config;
    }

    return rslt;
}

/*!
 * @brief This API gets the measurement configuration of the sensor
 */
int8_t icp20100_get_config(struct icp20100_config *config, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && config == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Read mode select register */
        rslt = dev->read(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            config->meas_mode = (reg_data >> 5) & 0x07;
            config->meas_type = (reg_data >> 3) & 0x01;
            config->power_mode = (reg_data >> 2) & 0x01;
            config->fifo_mode = reg_data & 0x03;
        }
    }

    if (rslt == ICP20100_OK)
    {
        /* Read drive strength register */
        rslt = dev->read(ICP20100_REG_IO_DRIVE_STRENGTH, &reg_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            config->drive_strength = reg_data & 0x07;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the pressure and temperature data from the sensor
 */
int8_t icp20100_get_data(struct icp20100_data *data, struct icp20100_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && data == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Read raw pressure and temperature data */
        rslt = read_raw_data(&data->pressure_raw, &data->temperature_raw, dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Convert raw data to physical units */
        data->pressure = convert_pressure(data->pressure_raw);
        data->temperature = convert_temperature(data->temperature_raw);
    }

    return rslt;
}

/*!
 * @brief This API triggers a forced measurement
 */
int8_t icp20100_trigger_measurement(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        /* Wait for mode synchronization */
        rslt = wait_mode_sync(dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Read current mode select register */
        rslt = dev->read(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Set forced measurement trigger bit */
        reg_data |= (1 << 4);
        rslt = dev->write(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor
 */
int8_t icp20100_set_power_mode(uint8_t power_mode, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        if (power_mode > ICP20100_POWER_MODE_ACTIVE)
        {
            rslt = ICP20100_E_INVALID_CONFIG;
        }
    }

    if (rslt == ICP20100_OK)
    {
        /* Wait for mode synchronization */
        rslt = wait_mode_sync(dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Read current mode select register */
        rslt = dev->read(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Update power mode bits */
        reg_data = (reg_data & ~ICP20100_POWER_MODE_MASK) | (power_mode << 2);
        rslt = dev->write(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Update device configuration */
        dev->config.power_mode = power_mode;
        
        /* Wait for power mode change if switching to active mode */
        if (power_mode == ICP20100_POWER_MODE_ACTIVE)
        {
            dev->delay_us(ICP20100_POWER_UP_TIME_MS * 1000, dev->intf_ptr);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor
 */
int8_t icp20100_get_power_mode(uint8_t *power_mode, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && power_mode == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Read mode select register */
        rslt = dev->read(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Extract power mode bits */
        *power_mode = (reg_data & ICP20100_POWER_MODE_MASK) >> 2;
    }

    return rslt;
}

/*!
 * @brief This API configures the interrupt settings
 */
int8_t icp20100_set_int_config(const struct icp20100_int_config *int_config, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t int_mask = 0;
    uint8_t fifo_config = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && int_config == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Configure interrupt mask register */
        int_mask = (int_config->fifo_overflow_en ? 0 : (1 << 0)) |
                   (int_config->fifo_underflow_en ? 0 : (1 << 1)) |
                   (int_config->fifo_wmk_high_en ? 0 : (1 << 2)) |
                   (int_config->fifo_wmk_low_en ? 0 : (1 << 3)) |
                   (int_config->press_abs_en ? 0 : (1 << 5)) |
                   (int_config->press_delta_en ? 0 : (1 << 6)) |
                   (1 << 7); /* Reserved bit set to 1 */

        rslt = dev->write(ICP20100_REG_INTERRUPT_MASK, &int_mask, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Configure FIFO watermark levels */
        fifo_config = (int_config->fifo_wmk_high << 4) | int_config->fifo_wmk_low;
        rslt = dev->write(ICP20100_REG_FIFO_CONFIG, &fifo_config, 1, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This API reads the interrupt status
 */
int8_t icp20100_get_int_status(uint8_t *int_status, struct icp20100_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && int_status == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Read interrupt status register */
        rslt = dev->read(ICP20100_REG_INTERRUPT_STATUS, int_status, 1, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This API clears the interrupt status
 */
int8_t icp20100_clear_int_status(uint8_t int_status, struct icp20100_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        /* Write 1 to clear interrupt status bits (W1C) */
        rslt = dev->write(ICP20100_REG_INTERRUPT_STATUS, &int_status, 1, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This API gets the FIFO length
 */
int8_t icp20100_get_fifo_length(uint8_t *fifo_length, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && fifo_length == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Read FIFO fill register */
        rslt = dev->read(ICP20100_REG_FIFO_FILL, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Extract FIFO level */
        *fifo_length = reg_data & ICP20100_FIFO_LEVEL_MASK;
    }

    return rslt;
}

/*!
 * @brief This API reads data from FIFO
 */
int8_t icp20100_read_fifo(struct icp20100_data *data, uint8_t length, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t fifo_level;
    uint8_t samples_to_read;
    uint8_t fifo_data[6];
    uint8_t i;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK && data == NULL)
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    if (rslt == ICP20100_OK)
    {
        /* Get current FIFO level */
        rslt = icp20100_get_fifo_length(&fifo_level, dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Determine number of samples to read */
        samples_to_read = (length < fifo_level) ? length : fifo_level;

        /* Read samples from FIFO */
        for (i = 0; i < samples_to_read; i++)
        {
            /* Read 6 bytes (3 for pressure, 3 for temperature) */
            rslt = dev->read(ICP20100_REG_PRESS_DATA_0, fifo_data, 6, dev->intf_ptr);
            
            if (rslt != ICP20100_OK)
            {
                break;
            }

            /* Parse data based on FIFO mode */
            if (dev->config.fifo_mode == ICP20100_FIFO_PRESSURE_FIRST)
            {
                /* Pressure first mode: PRESS[0-2], TEMP[0-2] */
                data[i].pressure_raw = ((int32_t)fifo_data[0] << 16) |
                                       ((int32_t)fifo_data[1] << 8) |
                                       fifo_data[2];
                data[i].temperature_raw = ((int32_t)fifo_data[3] << 16) |
                                          ((int32_t)fifo_data[4] << 8) |
                                          fifo_data[5];
            }
            else if (dev->config.fifo_mode == ICP20100_FIFO_TEMP_FIRST)
            {
                /* Temperature first mode: TEMP[0-2], PRESS[0-2] */
                data[i].temperature_raw = ((int32_t)fifo_data[0] << 16) |
                                          ((int32_t)fifo_data[1] << 8) |
                                          fifo_data[2];
                data[i].pressure_raw = ((int32_t)fifo_data[3] << 16) |
                                       ((int32_t)fifo_data[4] << 8) |
                                       fifo_data[5];
            }
            else if (dev->config.fifo_mode == ICP20100_FIFO_PRESSURE_ONLY)
            {
                /* Pressure only mode: PRESS[0-2] */
                rslt = dev->read(ICP20100_REG_TEMP_DATA_0, fifo_data, 3, dev->intf_ptr);
                if (rslt == ICP20100_OK)
                {
                    data[i].pressure_raw = ((int32_t)fifo_data[0] << 16) |
                                           ((int32_t)fifo_data[1] << 8) |
                                           fifo_data[2];
                    data[i].temperature_raw = 0; /* No temperature data */
                }
            }
            else /* TEMP_ONLY */
            {
                /* Temperature only mode: TEMP[0-2] */
                rslt = dev->read(ICP20100_REG_TEMP_DATA_0, fifo_data, 3, dev->intf_ptr);
                if (rslt == ICP20100_OK)
                {
                    data[i].temperature_raw = ((int32_t)fifo_data[0] << 16) |
                                              ((int32_t)fifo_data[1] << 8) |
                                              fifo_data[2];
                    data[i].pressure_raw = 0; /* No pressure data */
                }
            }

            if (rslt == ICP20100_OK)
            {
                /* Convert raw data to physical units */
                data[i].pressure = convert_pressure(data[i].pressure_raw);
                data[i].temperature = convert_temperature(data[i].temperature_raw);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API flushes the FIFO
 */
int8_t icp20100_flush_fifo(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == ICP20100_OK)
    {
        /* Read current FIFO fill register */
        rslt = dev->read(ICP20100_REG_FIFO_FILL, &reg_data, 1, dev->intf_ptr);
    }

    if (rslt == ICP20100_OK)
    {
        /* Set flush bit */
        reg_data |= ICP20100_FIFO_FLUSH_MASK;
        rslt = dev->write(ICP20100_REG_FIFO_FILL, &reg_data, 1, dev->intf_ptr);
    }

    return rslt;
}

/****************************************************************************/
/**\name        Local Function Definitions                                 */
/****************************************************************************/

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct icp20100_dev *dev)
{
    int8_t rslt = ICP20100_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        rslt = ICP20100_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API reads the chip-id and version of the sensor.
 */
static int8_t read_chip_id(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id;
    uint8_t version;

    /* Read chip ID */
    rslt = dev->read(ICP20100_REG_DEVICE_ID, &chip_id, 1, dev->intf_ptr);

    if (rslt == ICP20100_OK)
    {
        if (chip_id == ICP20100_CHIP_ID)
        {
            dev->chip_id = chip_id;
            
            /* Read version */
            rslt = dev->read(ICP20100_REG_VERSION, &version, 1, dev->intf_ptr);
            
            if (rslt == ICP20100_OK)
            {
                dev->version = version;
            }
        }
        else
        {
            rslt = ICP20100_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API checks if boot sequence is required.
 */
static int8_t check_boot_required(struct icp20100_dev *dev, uint8_t *boot_required)
{
    int8_t rslt;
    uint8_t boot_status;

    *boot_required = 0;

    /* Only version A requires boot sequence */
    if (dev->version == ICP20100_VERSION_A)
    {
        /* Read boot status */
        rslt = dev->read(ICP20100_REG_OTP_STATUS2, &boot_status, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            /* Check if boot sequence was already completed */
            if ((boot_status & ICP20100_BOOT_UP_STATUS_MASK) == 0)
            {
                *boot_required = 1;
            }
        }
    }
    else
    {
        /* Version B doesn't require boot sequence */
        rslt = ICP20100_OK;
    }

    return rslt;
}

/*!
 * @brief This internal API executes the OTP read sequence.
 */
static int8_t otp_read_sequence(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t offset, gain, hfosc;

    /* Prepare device for OTP operations */
    rslt = otp_prepare_device(dev);

    if (rslt == ICP20100_OK)
    {
        /* Enable OTP interface */
        rslt = otp_enable_interface(dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Configure redundant read registers */
        rslt = otp_configure_redundant_read(dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Read calibration data */
        rslt = otp_read_calibration_data(&offset, &gain, &hfosc, dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Write calibration data to trim registers */
        rslt = otp_write_trim_registers(offset, gain, hfosc, dev);
    }

    /* Always attempt to finalize sequence, even if previous steps failed */
    int8_t finalize_rslt = otp_finalize_sequence(dev);
    
    /* Return the first error that occurred, or finalize result if all was OK */
    return (rslt != ICP20100_OK) ? rslt : finalize_rslt;
}

/*!
 * @brief This internal API prepares the device for OTP operations.
 */
static int8_t otp_prepare_device(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Enable power mode */
    reg_data = ICP20100_POWER_MODE_MASK;
    rslt = dev->write(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
    
    if (rslt == ICP20100_OK)
    {
        dev->delay_us(ICP20100_POWER_UP_TIME_MS * 1000, dev->intf_ptr);
        
        /* Unlock main registers */
        reg_data = ICP20100_MASTER_LOCK_UNLOCK;
        rslt = dev->write(ICP20100_REG_MASTER_LOCK, &reg_data, 1, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This internal API enables OTP interface.
 */
static int8_t otp_enable_interface(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Enable OTP and write switch */
    rslt = dev->read(ICP20100_REG_OTP_CONFIG1, &reg_data, 1, dev->intf_ptr);
    
    if (rslt == ICP20100_OK)
    {
        reg_data |= (ICP20100_OTP_ENABLE_MASK | ICP20100_OTP_WRITE_SWITCH_MASK);
        rslt = dev->write(ICP20100_REG_OTP_CONFIG1, &reg_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            dev->delay_us(ICP20100_OTP_WAIT_TIME_US, dev->intf_ptr);
            
            /* Toggle OTP reset */
            rslt = dev->read(ICP20100_REG_OTP_DBG2, &reg_data, 1, dev->intf_ptr);
            
            if (rslt == ICP20100_OK)
            {
                reg_data |= ICP20100_OTP_RESET_MASK;
                rslt = dev->write(ICP20100_REG_OTP_DBG2, &reg_data, 1, dev->intf_ptr);
                
                if (rslt == ICP20100_OK)
                {
                    dev->delay_us(ICP20100_OTP_WAIT_TIME_US, dev->intf_ptr);
                    
                    reg_data &= ~ICP20100_OTP_RESET_MASK;
                    rslt = dev->write(ICP20100_REG_OTP_DBG2, &reg_data, 1, dev->intf_ptr);
                    
                    if (rslt == ICP20100_OK)
                    {
                        dev->delay_us(ICP20100_OTP_WAIT_TIME_US, dev->intf_ptr);
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API configures OTP redundant read registers.
 */
static int8_t otp_configure_redundant_read(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Configure MRA registers */
    reg_data = 0x04;
    rslt = dev->write(ICP20100_REG_OTP_MRA_LSB, &reg_data, 1, dev->intf_ptr);
    
    if (rslt == ICP20100_OK)
    {
        rslt = dev->write(ICP20100_REG_OTP_MRA_MSB, &reg_data, 1, dev->intf_ptr);
    }

    /* Configure MRB registers */
    if (rslt == ICP20100_OK)
    {
        reg_data = 0x21;
        rslt = dev->write(ICP20100_REG_OTP_MRB_LSB, &reg_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            reg_data = 0x20;
            rslt = dev->write(ICP20100_REG_OTP_MRB_MSB, &reg_data, 1, dev->intf_ptr);
        }
    }

    /* Configure MR registers */
    if (rslt == ICP20100_OK)
    {
        reg_data = 0x10;
        rslt = dev->write(ICP20100_REG_OTP_MR_LSB, &reg_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            reg_data = 0x80;
            rslt = dev->write(ICP20100_REG_OTP_MR_MSB, &reg_data, 1, dev->intf_ptr);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API reads calibration data from OTP.
 */
static int8_t otp_read_calibration_data(uint8_t *offset, uint8_t *gain, uint8_t *hfosc, struct icp20100_dev *dev)
{
    int8_t rslt;

    /* Read offset calibration data from address 0xF8 */
    rslt = read_otp_data(0xF8, offset, dev);

    if (rslt == ICP20100_OK)
    {
        /* Read gain calibration data from address 0xF9 */
        rslt = read_otp_data(0xF9, gain, dev);
    }

    if (rslt == ICP20100_OK)
    {
        /* Read HFosc calibration data from address 0xFA */
        rslt = read_otp_data(0xFA, hfosc, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API writes calibration data to trim registers.
 */
static int8_t otp_write_trim_registers(uint8_t offset, uint8_t gain, uint8_t hfosc, struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t trim_data;

    /* Write offset to TRIM1_MSB */
    rslt = dev->read(ICP20100_REG_TRIM1_MSB, &trim_data, 1, dev->intf_ptr);
    
    if (rslt == ICP20100_OK)
    {
        trim_data = (trim_data & 0xC0) | (offset & 0x3F);
        rslt = dev->write(ICP20100_REG_TRIM1_MSB, &trim_data, 1, dev->intf_ptr);
    }

    /* Write gain to TRIM2_MSB */
    if (rslt == ICP20100_OK)
    {
        rslt = dev->read(ICP20100_REG_TRIM2_MSB, &trim_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            trim_data = (trim_data & 0x8F) | ((gain & 0x07) << 4);
            rslt = dev->write(ICP20100_REG_TRIM2_MSB, &trim_data, 1, dev->intf_ptr);
        }
    }

    /* Write HFosc to TRIM2_LSB */
    if (rslt == ICP20100_OK)
    {
        rslt = dev->read(ICP20100_REG_TRIM2_LSB, &trim_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            trim_data = (trim_data & 0x80) | (hfosc & 0x7F);
            rslt = dev->write(ICP20100_REG_TRIM2_LSB, &trim_data, 1, dev->intf_ptr);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API disables OTP interface and locks registers.
 */
static int8_t otp_finalize_sequence(struct icp20100_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Disable OTP and write switch */
    rslt = dev->read(ICP20100_REG_OTP_CONFIG1, &reg_data, 1, dev->intf_ptr);
    
    if (rslt == ICP20100_OK)
    {
        reg_data &= ~(ICP20100_OTP_ENABLE_MASK | ICP20100_OTP_WRITE_SWITCH_MASK);
        rslt = dev->write(ICP20100_REG_OTP_CONFIG1, &reg_data, 1, dev->intf_ptr);
        
        if (rslt == ICP20100_OK)
        {
            dev->delay_us(ICP20100_OTP_WAIT_TIME_US, dev->intf_ptr);
            
            /* Lock main registers */
            reg_data = 0x00;
            rslt = dev->write(ICP20100_REG_MASTER_LOCK, &reg_data, 1, dev->intf_ptr);
            
            if (rslt == ICP20100_OK)
            {
                /* Return to standby mode */
                reg_data = 0x00;
                rslt = dev->write(ICP20100_REG_MODE_SELECT, &reg_data, 1, dev->intf_ptr);
            }
        }
    }

    return rslt;
}
