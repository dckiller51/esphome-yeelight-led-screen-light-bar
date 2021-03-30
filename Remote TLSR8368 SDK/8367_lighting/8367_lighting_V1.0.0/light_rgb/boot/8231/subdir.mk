################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../boot/8231/cstartup_8231.S 

OBJS += \
./boot/8231/cstartup_8231.o 


# Each subdirectory must supply rules for building sources it contributes
boot/8231/%.o: ../boot/8231/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 CC/Assembler'
	tc32-elf-gcc -DMCU_STARTUP_8231 -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


