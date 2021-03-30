################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/remote/app.c \
../vendor/remote/interrupt.c \
../vendor/remote/main.c \
../vendor/remote/rf_control.c 

OBJS += \
./vendor/remote/app.o \
./vendor/remote/interrupt.o \
./vendor/remote/main.o \
./vendor/remote/rf_control.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/remote/%.o: ../vendor/remote/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\drivers_blackhawk" -D__PROJECT_LIGHT__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


