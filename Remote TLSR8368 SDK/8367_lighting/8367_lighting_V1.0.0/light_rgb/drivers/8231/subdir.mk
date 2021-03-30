################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/8231/adc.c \
../drivers/8231/aes.c \
../drivers/8231/analog.c \
../drivers/8231/bsp.c \
../drivers/8231/clock.c \
../drivers/8231/emi.c \
../drivers/8231/flash.c \
../drivers/8231/gpio_8231.c \
../drivers/8231/i2c.c \
../drivers/8231/otp.c \
../drivers/8231/qdec.c \
../drivers/8231/spi.c \
../drivers/8231/timer.c \
../drivers/8231/uart.c \
../drivers/8231/watchdog.c 

OBJS += \
./drivers/8231/adc.o \
./drivers/8231/aes.o \
./drivers/8231/analog.o \
./drivers/8231/bsp.o \
./drivers/8231/clock.o \
./drivers/8231/emi.o \
./drivers/8231/flash.o \
./drivers/8231/gpio_8231.o \
./drivers/8231/i2c.o \
./drivers/8231/otp.o \
./drivers/8231/qdec.o \
./drivers/8231/spi.o \
./drivers/8231/timer.o \
./drivers/8231/uart.o \
./drivers/8231/watchdog.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/8231/%.o: ../drivers/8231/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\8367_lighting" -D__PROJECT_LIGHT__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


