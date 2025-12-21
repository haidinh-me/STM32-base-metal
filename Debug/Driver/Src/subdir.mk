################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/stm32_GPIO.c \
../Driver/Src/stm32_I2C.c \
../Driver/Src/stm32_RCC.c \
../Driver/Src/stm32_SPI.c \
../Driver/Src/stm32_USART.c 

OBJS += \
./Driver/Src/stm32_GPIO.o \
./Driver/Src/stm32_I2C.o \
./Driver/Src/stm32_RCC.o \
./Driver/Src/stm32_SPI.o \
./Driver/Src/stm32_USART.o 

C_DEPS += \
./Driver/Src/stm32_GPIO.d \
./Driver/Src/stm32_I2C.d \
./Driver/Src/stm32_RCC.d \
./Driver/Src/stm32_SPI.d \
./Driver/Src/stm32_USART.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/%.o Driver/Src/%.su Driver/Src/%.cyclo: ../Driver/Src/%.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401CCYx -DSTM32F4 -c -I"F:/Document/STM32/Driver_STM32F4/Inc" -I"F:/Document/STM32/Driver_STM32F4/Driver/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Driver-2f-Src

clean-Driver-2f-Src:
	-$(RM) ./Driver/Src/stm32_GPIO.cyclo ./Driver/Src/stm32_GPIO.d ./Driver/Src/stm32_GPIO.o ./Driver/Src/stm32_GPIO.su ./Driver/Src/stm32_I2C.cyclo ./Driver/Src/stm32_I2C.d ./Driver/Src/stm32_I2C.o ./Driver/Src/stm32_I2C.su ./Driver/Src/stm32_RCC.cyclo ./Driver/Src/stm32_RCC.d ./Driver/Src/stm32_RCC.o ./Driver/Src/stm32_RCC.su ./Driver/Src/stm32_SPI.cyclo ./Driver/Src/stm32_SPI.d ./Driver/Src/stm32_SPI.o ./Driver/Src/stm32_SPI.su ./Driver/Src/stm32_USART.cyclo ./Driver/Src/stm32_USART.d ./Driver/Src/stm32_USART.o ./Driver/Src/stm32_USART.su

.PHONY: clean-Driver-2f-Src

