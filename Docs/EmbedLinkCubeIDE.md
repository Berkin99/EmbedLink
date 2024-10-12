# EmbedLink - SDK 

## 1.  IOC

### 1.1 Pinout & Configuration
#### SystemCore:
* RCC/HSE
* SYS/TimebaseSource/TIM1

#### Analog:
* ADC1/SingleEnded : PSC8 : ADC12bit : NVIC

#### Timers:
* TIM2/ClockSource/Internal
* TIM2/PSC = (TIM BUS SPEED MHZ) - 1 = 1MHZ
* TIM3/TIM4/ClockSource/Internal
* TIM3/TIM4/PSC = (TIM BUS SPEED MHZ / 4) - 1  = 4MHZ = 0.25us 
* TIM3/TIM4/Period = 16000 : (0.25us * 16000 = 4ms Standard ESC)

#### Connectivity : NVIC
* I2C1/I2C2 400KHZ 
* SPI1/SPI3 8bit : 4Mbps 
* UART1 
* UART2 115200
* UART3  
* UART4 
* USB FS

#### Pins:
* GPIO Pinouts

#### Debug:
* SerialWire

#### Software:
* FreeRTOS > CMSIS_V2
* USB Device

### 1.2 Clock Configuration MHz
* HSE  26
* CPU  200
* TIM  200
* SPI  32
* I2C  32
* ADC  32
* UART 32

## 2.  Firmware

* Clone The EmbedLink Repo @Core
* Add The Include Paths
* Properties/C Build/Settings/MCU Settings > enable newlib float
* extern and define on the @main.h
* Call systemLaunch in @main.c
* Start system timer HAL_TIM_BaseStart() in TIM config end @main.c
* Linker List .mem. .nrx.
* Configure the sysconfig.h
