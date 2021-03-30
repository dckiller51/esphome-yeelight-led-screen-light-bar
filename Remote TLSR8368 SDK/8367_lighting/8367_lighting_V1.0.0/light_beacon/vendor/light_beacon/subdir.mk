################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/light_beacon/app.c \
../vendor/light_beacon/interrupt.c \
../vendor/light_beacon/led.c \
../vendor/light_beacon/main.c \
../vendor/light_beacon/pairing_op.c \
../vendor/light_beacon/rf_control.c \
../vendor/light_beacon/time_event.c \
../vendor/light_beacon/user_pwm.c 

OBJS += \
./vendor/light_beacon/app.o \
./vendor/light_beacon/interrupt.o \
./vendor/light_beacon/led.o \
./vendor/light_beacon/main.o \
./vendor/light_beacon/pairing_op.o \
./vendor/light_beacon/rf_control.o \
./vendor/light_beacon/time_event.o \
./vendor/light_beacon/user_pwm.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/light_beacon/%.o: ../vendor/light_beacon/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\drivers_blackhawk" -D__PROJECT_LIGHT_BEACON__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


