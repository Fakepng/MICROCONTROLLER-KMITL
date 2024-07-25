################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/EEPROM_Emul/Core/eeprom_emul.c 

OBJS += \
./Core/EEPROM_Emul/Core/eeprom_emul.o 

C_DEPS += \
./Core/EEPROM_Emul/Core/eeprom_emul.d 


# Each subdirectory must supply rules for building sources it contributes
Core/EEPROM_Emul/Core/%.o Core/EEPROM_Emul/Core/%.su Core/EEPROM_Emul/Core/%.cyclo: ../Core/EEPROM_Emul/Core/%.c Core/EEPROM_Emul/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-EEPROM_Emul-2f-Core

clean-Core-2f-EEPROM_Emul-2f-Core:
	-$(RM) ./Core/EEPROM_Emul/Core/eeprom_emul.cyclo ./Core/EEPROM_Emul/Core/eeprom_emul.d ./Core/EEPROM_Emul/Core/eeprom_emul.o ./Core/EEPROM_Emul/Core/eeprom_emul.su

.PHONY: clean-Core-2f-EEPROM_Emul-2f-Core

