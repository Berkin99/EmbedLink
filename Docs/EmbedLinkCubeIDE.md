# EmbedLink - SDK 

## 1.  IOC

### 1.1 Pinout & Configuration
#### SystemCore:
* RCC/HSE
* SYS/TimebaseSource/TIM1

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

* Clone The EmbedLink Repo @Core : OK
* Add The Include Paths : OK
* Properties/C Build/Settings/MCU Settings > enable newlib float : OK
* extern and define on the @main.h : OK
* Call systemLaunch in @main.c : OK
* Start system timer HAL_TIM_Base_Start(&htim2) in TIM config end @main.c : OK
* Linker List .mem. .nrx.
* Configure the sysconfig.h

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    /* Parameters */
    . = ALIGN(4);
    _nrx_start = .;
    KEEP(*(.nrx))
    KEEP(*(.nrx.*))
    _nrx_stop = .;

    /* Parameters */
    . = ALIGN(4);
    _mem_start = .;
    KEEP(*(.mem))
    KEEP(*(.mem.*))
    _mem_stop = .;

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH
