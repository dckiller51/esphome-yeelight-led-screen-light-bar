################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include vendor/light_rgb/subdir.mk
-include user_drivers/subdir.mk
-include drivers/8231/subdir.mk
-include boot/8231/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
light_rgb.lst \

FLASH_IMAGE += \
light_rgb.bin \

SIZEDUMMY += \
sizedummy \


# All Target
all: 8367_lighting.elf secondary-outputs

# Tool invocations
8367_lighting.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 C Linker'
	tc32-elf-ld --gc-sections -L"E:\workspace2019\custom_SDK\8367_lighting\drivers\8231" -T ../boot.link -o"8367_lighting.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

light_rgb.lst: 8367_lighting.elf
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S 8367_lighting.elf  >"light_rgb.lst"
	@echo 'Finished building: $@'
	@echo ' '

light_rgb.bin: 8367_lighting.elf
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy -O binary 8367_lighting.elf  "light_rgb.bin"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: 8367_lighting.elf
	@echo 'Invoking: Print Size'
	tc32-elf-size -t 8367_lighting.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(FLASH_IMAGE)$(ELFS)$(LST)$(SIZEDUMMY) 8367_lighting.elf
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
