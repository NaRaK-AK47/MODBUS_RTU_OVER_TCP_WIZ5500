################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/W5500/W5500/subdir.mk
-include Core/W5500/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := DHCP_MODBUS_TCP_STM32C6T6_WIZ5500
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.elf \

MAP_FILES += \
DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.list \


# All Target
all: main-build

# Main-build Target
main-build: DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.elf secondary-outputs

# Tool invocations
DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.elf DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.map: $(OBJS) $(USER_OBJS) C:\Embedded-C\Personal_course\Stm32_Excercise\DHCP_MODBUS_TCP_STM32C6T6_WIZ5500\STM32F103C6TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Embedded-C\Personal_course\Stm32_Excercise\DHCP_MODBUS_TCP_STM32C6T6_WIZ5500\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.elf DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.list DHCP_MODBUS_TCP_STM32C6T6_WIZ5500.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
