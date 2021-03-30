################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include vendor/remote/subdir.mk
-include user_drivers/subdir.mk
-include drivers/8231/subdir.mk
-include boot/8231/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
remote.lst \

FLASH_IMAGE += \
remote.bin \

SIZEDUMMY += \
sizedummy \


# All Target
all: drivers_blackhawk.elf secondary-outputs

# Tool invocations
drivers_blackhawk.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 C Linker'
	tc32-elf-ld --gc-sections -L"E:\workspace2019\custom_SDK\drivers_blackhawk\drivers\8231" -T ../boot.link -o"drivers_blackhawk.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

remote.lst: drivers_blackhawk.elf
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S drivers_blackhawk.elf  >"remote.lst"
	@echo 'Finished building: $@'
	@echo ' '

remote.bin: drivers_blackhawk.elf
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy -O binary drivers_blackhawk.elf  "remote.bin"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: drivers_blackhawk.elf
	@echo 'Invoking: Print Size'
	tc32-elf-size -t drivers_blackhawk.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(FLASH_IMAGE)$(ELFS)$(LST)$(SIZEDUMMY) drivers_blackhawk.elf
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
