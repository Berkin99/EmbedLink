# Embedlink

This repo is a subfolder in a embedded project.
For stm32 project : {$ProjectWorkspace}/Core/Src/Embedlink

System codes (Embedlink/Src/*):

system      : Main function for code, works like start, loop.
systime     : Contains millis() and delay() functions.
sysdefs     : Generic definitions for return or input types for all embedlink api.
sysconfig   : Configuration file.

Hal codes (Embedlink/Src/Hal/*):

adc.h adc.c
gpio.h gpio.c
i2c.h i2c.c
spi.h spi.c
uart.h uart.c
usb.h usb.c