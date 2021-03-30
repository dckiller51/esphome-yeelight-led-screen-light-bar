################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include drivers/8231/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
rf_pm.lst \

FLASH_IMAGE += \

SIZEDUMMY += \
sizedummy \


# All Target
all: libdrivers_blackhawk.a secondary-outputs

# Tool invocations
libdrivers_blackhawk.a: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 Archiver'
	tc32-elf-ar -r "libdrivers_blackhawk.a" $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

rf_pm.lst: libdrivers_blackhawk.a
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S libdrivers_blackhawk.a  >"rf_pm.lst"
	@echo 'Finished building: $@'
	@echo ' '

: libdrivers_blackhawk.a
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy -O binary libdrivers_blackhawk.a
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: libdrivers_blackhawk.a
	@echo 'Invoking: Print Size'
	tc32-elf-size -t libdrivers_blackhawk.a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(ARCHIVES)$(FLASH_IMAGE)$(LST)$(SIZEDUMMY) libdrivers_blackhawk.a
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
