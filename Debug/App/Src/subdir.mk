################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/Address.c \
../App/Src/Application.c \
../App/Src/Audio.c \
../App/Src/I2CDevices.c \
../App/Src/Interaction.c \
../App/Src/IrTsop.c \
../App/Src/QRE.c \
../App/Src/VL53l0x.c \
../App/Src/WS2812.c \
../App/Src/Wheel.c 

OBJS += \
./App/Src/Address.o \
./App/Src/Application.o \
./App/Src/Audio.o \
./App/Src/I2CDevices.o \
./App/Src/Interaction.o \
./App/Src/IrTsop.o \
./App/Src/QRE.o \
./App/Src/VL53l0x.o \
./App/Src/WS2812.o \
./App/Src/Wheel.o 

C_DEPS += \
./App/Src/Address.d \
./App/Src/Application.d \
./App/Src/Audio.d \
./App/Src/I2CDevices.d \
./App/Src/Interaction.d \
./App/Src/IrTsop.d \
./App/Src/QRE.d \
./App/Src/VL53l0x.d \
./App/Src/WS2812.d \
./App/Src/Wheel.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/Address.cyclo ./App/Src/Address.d ./App/Src/Address.o ./App/Src/Address.su ./App/Src/Application.cyclo ./App/Src/Application.d ./App/Src/Application.o ./App/Src/Application.su ./App/Src/Audio.cyclo ./App/Src/Audio.d ./App/Src/Audio.o ./App/Src/Audio.su ./App/Src/I2CDevices.cyclo ./App/Src/I2CDevices.d ./App/Src/I2CDevices.o ./App/Src/I2CDevices.su ./App/Src/Interaction.cyclo ./App/Src/Interaction.d ./App/Src/Interaction.o ./App/Src/Interaction.su ./App/Src/IrTsop.cyclo ./App/Src/IrTsop.d ./App/Src/IrTsop.o ./App/Src/IrTsop.su ./App/Src/QRE.cyclo ./App/Src/QRE.d ./App/Src/QRE.o ./App/Src/QRE.su ./App/Src/VL53l0x.cyclo ./App/Src/VL53l0x.d ./App/Src/VL53l0x.o ./App/Src/VL53l0x.su ./App/Src/WS2812.cyclo ./App/Src/WS2812.d ./App/Src/WS2812.o ./App/Src/WS2812.su ./App/Src/Wheel.cyclo ./App/Src/Wheel.d ./App/Src/Wheel.o ./App/Src/Wheel.su

.PHONY: clean-App-2f-Src

