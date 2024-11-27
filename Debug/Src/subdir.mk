################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/010i2c_master_tx_testing.c 

OBJS += \
./Src/010i2c_master_tx_testing.o 

C_DEPS += \
./Src/010i2c_master_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Embedded system workspace/03 MCU1/My Workspace/stm32f4xx_drivers/bsp" -I"D:/Embedded system workspace/03 MCU1/My Workspace/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/010i2c_master_tx_testing.cyclo ./Src/010i2c_master_tx_testing.d ./Src/010i2c_master_tx_testing.o ./Src/010i2c_master_tx_testing.su

.PHONY: clean-Src

