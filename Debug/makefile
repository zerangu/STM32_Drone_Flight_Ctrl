################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
IO_test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
IO_test.list \

OBJCOPY_BIN += \
IO_test.bin \


# All Target
all: IO_test.elf secondary-outputs

# Tool invocations
IO_test.elf: $(OBJS) $(USER_OBJS) C:\Users\Gecao\STM32CubeIDE\workspace_1.3.0\IO_test\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "IO_test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Gecao\STM32CubeIDE\workspace_1.3.0\IO_test\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="IO_test.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

IO_test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "IO_test.list"
	@echo 'Finished building: $@'
	@echo ' '

IO_test.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "IO_test.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
