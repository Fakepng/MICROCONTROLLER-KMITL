################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/EEPROM_Emul/Core/eeprom_emul.c 

OBJS += \
./Drivers/EEPROM_Emul/Core/eeprom_emul.o 

C_DEPS += \
./Drivers/EEPROM_Emul/Core/eeprom_emul.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/EEPROM_Emul/Core/%.o Drivers/EEPROM_Emul/Core/%.su Drivers/EEPROM_Emul/Core/%.cyclo: ../Drivers/EEPROM_Emul/Core/%.c Drivers/EEPROM_Emul/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/EEPROM_Emul/Porting/STM32G0 -I../Drivers/EEPROM_Emul/Core -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-EEPROM_Emul-2f-Core

clean-Drivers-2f-EEPROM_Emul-2f-Core:
	-$(RM) ./Drivers/EEPROM_Emul/Core/eeprom_emul.cyclo ./Drivers/EEPROM_Emul/Core/eeprom_emul.d ./Drivers/EEPROM_Emul/Core/eeprom_emul.o ./Drivers/EEPROM_Emul/Core/eeprom_emul.su

.PHONY: clean-Drivers-2f-EEPROM_Emul-2f-Core

