################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L476xx -I"/Users/alexmacbook/SCOPUS2_STM32L476G-DISCO/SCOPUS2_Constant_Power/Inc" -I"/Users/alexmacbook/SCOPUS2_STM32L476G-DISCO/SCOPUS2_Constant_Power/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/Users/alexmacbook/SCOPUS2_STM32L476G-DISCO/SCOPUS2_Constant_Power/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/Users/alexmacbook/SCOPUS2_STM32L476G-DISCO/SCOPUS2_Constant_Power/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/Users/alexmacbook/SCOPUS2_STM32L476G-DISCO/SCOPUS2_Constant_Power/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


