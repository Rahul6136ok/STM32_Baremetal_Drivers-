################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tollgate/toll.c 

OBJS += \
./tollgate/toll.o 

C_DEPS += \
./tollgate/toll.d 


# Each subdirectory must supply rules for building sources it contributes
tollgate/%.o tollgate/%.su tollgate/%.cyclo: ../tollgate/%.c tollgate/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Embedded system workspace/03 MCU1/My Workspace/stm32f4xx_drivers/bsp" -I"D:/Embedded system workspace/03 MCU1/My Workspace/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-tollgate

clean-tollgate:
	-$(RM) ./tollgate/toll.cyclo ./tollgate/toll.d ./tollgate/toll.o ./tollgate/toll.su

.PHONY: clean-tollgate

