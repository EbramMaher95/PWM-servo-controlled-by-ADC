################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.c 

OBJS += \
./Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.o 

C_DEPS += \
./Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Device_Drivers/HD44780_LCD/%.o Drivers/Device_Drivers/HD44780_LCD/%.su Drivers/Device_Drivers/HD44780_LCD/%.cyclo: ../Drivers/Device_Drivers/HD44780_LCD/%.c Drivers/Device_Drivers/HD44780_LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Device_Drivers-2f-HD44780_LCD

clean-Drivers-2f-Device_Drivers-2f-HD44780_LCD:
	-$(RM) ./Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.cyclo ./Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.d ./Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.o ./Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.su

.PHONY: clean-Drivers-2f-Device_Drivers-2f-HD44780_LCD

