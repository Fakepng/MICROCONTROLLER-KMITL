################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.c 

OBJS += \
./Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.o 

C_DEPS += \
./Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/EEPROM_Emul/Porting/STM32G0/%.o Drivers/EEPROM_Emul/Porting/STM32G0/%.su Drivers/EEPROM_Emul/Porting/STM32G0/%.cyclo: ../Drivers/EEPROM_Emul/Porting/STM32G0/%.c Drivers/EEPROM_Emul/Porting/STM32G0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/EEPROM_Emul/Porting/STM32G0 -I../Drivers/EEPROM_Emul/Core -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-EEPROM_Emul-2f-Porting-2f-STM32G0

clean-Drivers-2f-EEPROM_Emul-2f-Porting-2f-STM32G0:
	-$(RM) ./Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.cyclo ./Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.d ./Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.o ./Drivers/EEPROM_Emul/Porting/STM32G0/flash_interface.su

.PHONY: clean-Drivers-2f-EEPROM_Emul-2f-Porting-2f-STM32G0

