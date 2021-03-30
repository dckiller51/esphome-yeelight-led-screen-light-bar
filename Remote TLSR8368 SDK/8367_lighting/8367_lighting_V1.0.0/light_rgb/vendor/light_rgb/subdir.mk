################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/light_rgb/app.c \
../vendor/light_rgb/interrupt.c \
../vendor/light_rgb/led_control.c \
../vendor/light_rgb/led_rgb.c \
../vendor/light_rgb/led_yl.c \
../vendor/light_rgb/main.c \
../vendor/light_rgb/pairing_op.c \
../vendor/light_rgb/rf_control.c \
../vendor/light_rgb/time_event.c \
../vendor/light_rgb/user_pwm.c 

OBJS += \
./vendor/light_rgb/app.o \
./vendor/light_rgb/interrupt.o \
./vendor/light_rgb/led_control.o \
./vendor/light_rgb/led_rgb.o \
./vendor/light_rgb/led_yl.o \
./vendor/light_rgb/main.o \
./vendor/light_rgb/pairing_op.o \
./vendor/light_rgb/rf_control.o \
./vendor/light_rgb/time_event.o \
./vendor/light_rgb/user_pwm.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/light_rgb/%.o: ../vendor/light_rgb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\8367_lighting" -D__PROJECT_LIGHT__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


