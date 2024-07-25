################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/EEPROM_Emul/Porting/STM32G0/flash_interface.c 

OBJS += \
./Core/EEPROM_Emul/Porting/STM32G0/flash_interface.o 

C_DEPS += \
./Core/EEPROM_Emul/Porting/STM32G0/flash_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/EEPROM_Emul/Porting/STM32G0/%.o Core/EEPROM_Emul/Porting/STM32G0/%.su Core/EEPROM_Emul/Porting/STM32G0/%.cyclo: ../Core/EEPROM_Emul/Porting/STM32G0/%.c Core/EEPROM_Emul/Porting/STM32G0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-EEPROM_Emul-2f-Porting-2f-STM32G0

clean-Core-2f-EEPROM_Emul-2f-Porting-2f-STM32G0:
	-$(RM) ./Core/EEPROM_Emul/Porting/STM32G0/flash_interface.cyclo ./Core/EEPROM_Emul/Porting/STM32G0/flash_interface.d ./Core/EEPROM_Emul/Porting/STM32G0/flash_interface.o ./Core/EEPROM_Emul/Porting/STM32G0/flash_interface.su

.PHONY: clean-Core-2f-EEPROM_Emul-2f-Porting-2f-STM32G0

