################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/W5500/W5500/w5500.c 

OBJS += \
./Core/W5500/W5500/w5500.o 

C_DEPS += \
./Core/W5500/W5500/w5500.d 


# Each subdirectory must supply rules for building sources it contributes
Core/W5500/W5500/%.o Core/W5500/W5500/%.su Core/W5500/W5500/%.cyclo: ../Core/W5500/W5500/%.c Core/W5500/W5500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103x6 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Embedded-C/Personal_course/Stm32_Excercise/DHCP_MODBUS_TCP_STM32C6T6_WIZ5500/Core/W5500/W5500" -I"C:/Embedded-C/Personal_course/Stm32_Excercise/DHCP_MODBUS_TCP_STM32C6T6_WIZ5500/Core/W5500" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-W5500-2f-W5500

clean-Core-2f-W5500-2f-W5500:
	-$(RM) ./Core/W5500/W5500/w5500.cyclo ./Core/W5500/W5500/w5500.d ./Core/W5500/W5500/w5500.o ./Core/W5500/W5500/w5500.su

.PHONY: clean-Core-2f-W5500-2f-W5500

