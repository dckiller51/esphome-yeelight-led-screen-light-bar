################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/remote_beacon/app.c \
../vendor/remote_beacon/interrupt.c \
../vendor/remote_beacon/main.c \
../vendor/remote_beacon/rf_control.c 

OBJS += \
./vendor/remote_beacon/app.o \
./vendor/remote_beacon/interrupt.o \
./vendor/remote_beacon/main.o \
./vendor/remote_beacon/rf_control.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/remote_beacon/%.o: ../vendor/remote_beacon/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\drivers_blackhawk" -D__PROJECT_REMOTE_BEACON__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


