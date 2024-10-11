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

#ifndef SYSCONFIG_H_
#define SYSCONFIG_H_

#include "system.h"

/// SYSTIME /////////////////////////////////////////////////
#define SYSTIME              htim2

/// SYSDEBUG ////////////////////////////////////////////////
#define SYSDEBUG             uart2
#define SYSLED1              LED3
#define SYSLED2              LED4

/// LED /////////////////////////////////////////////////////
#define LED1				  PC1
#define LED2				  PC2
#define LED3				  PC3
#define LED4				  PC4

/// I2C /////////////////////////////////////////////////////
#define BMP388_I2C            i2c2
#define BNO055_I2C            i2c2
#define E24AA_I2C             i2c2
//#define HMC5883L_I2C          i2c1
//#define MPU6050_I2C           i2c1
//#define MS5611_I2C            i2c1

/// SPI /////////////////////////////////////////////////////
#define BMI088_SPI            spi1
#define BMI088_GYR_CS	      PB2
#define BMI088_ACC_CS		  PB1
#define RF24_SPI              spi1
#define RF24_CE               PE5
#define RF24_CS               PE6
#define RF24_RX_ADDRESS       {0xE7, 0xE7, 0xE7, 0xE3, 0x04}
#define RF24_TX_ADDRESS       {0xE7, 0xE7, 0xE7, 0xE3, 0x05}
//#define MPU6500_SPI
//#define MPU6500_CS

/// UART ////////////////////////////////////////////////////
//#define E32100_UART           uart1
//#define E32100_M0
//#define E32100_M1
//#define E32100_AUX
//#define NEOM8N_UART           huart1
#define ZEDF9P_UART           uart1

/// PWM /////////////////////////////////////////////////////s
#define PWM1_TIMER            htim3
#define PWM2_TIMER			  htim3
#define PWM3_TIMER            htim3
#define PWM4_TIMER            htim3
#define PWM1_CH               4
#define PWM2_CH				  3
#define PWM3_CH				  2
#define PWM4_CH				  1

/// ADC /////////////////////////////////////////////////////
#define ADC1_HANDLE            hadc1

/// MEM /////////////////////////////////////////////////////
#define MEM_DEBUG

/// NCOM ////////////////////////////////////////////////////
#define NC_MODULE             "RF24"
#define NC_NTRPPACKET
#define NC_RX_LED		      LED1
#define NC_TX_LED		      LED2
//#define NC_NTRPMESSAGE
//#define NC_ID                 'X'

/// RTOS ////////////////////////////////////////////////////

#define SYSTEM_TASK_PRI			(2)
#define ESTIMATOR_TASK_PRI      (5)
#define CONTROL_TASK_PRI        (6)

#define SYSTEM_TASK_STACK		(4 * RTOS_MIN_STACK)
#define ESTIMATOR_TASK_STACK    (4 * RTOS_MIN_STACK)
#define CONTROL_TASK_STACK      (4 * RTOS_MIN_STACKvs)

/////////////////////////////////////////////////////////////
#endif /* SYSCONFIG_H_ */
