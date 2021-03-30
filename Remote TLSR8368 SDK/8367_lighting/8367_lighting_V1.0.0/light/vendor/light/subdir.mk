################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/light/app.c \
../vendor/light/interrupt.c \
../vendor/light/led.c \
../vendor/light/main.c \
../vendor/light/pairing_op.c \
../vendor/light/rf_control.c \
../vendor/light/time_event.c \
../vendor/light/user_pwm.c 

OBJS += \
./vendor/light/app.o \
./vendor/light/interrupt.o \
./vendor/light/led.o \
./vendor/light/main.o \
./vendor/light/pairing_op.o \
./vendor/light/rf_control.o \
./vendor/light/time_event.o \
./vendor/light/user_pwm.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/light/%.o: ../vendor/light/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\drivers_blackhawk" -D__PROJECT_LIGHT__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


