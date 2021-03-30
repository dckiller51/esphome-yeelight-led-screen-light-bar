################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/8231/nordic_set_drv.c \
../drivers/8231/pm.c \
../drivers/8231/rf_drv.c 

OBJS += \
./drivers/8231/nordic_set_drv.o \
./drivers/8231/pm.o \
./drivers/8231/rf_drv.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/8231/%.o: ../drivers/8231/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\workspace2019\custom_SDK\drivers_blackhawk" -D__PROJECT_LIGHT__=1 -DCHIP_TYPE=CHIP_TYPE_8231 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


