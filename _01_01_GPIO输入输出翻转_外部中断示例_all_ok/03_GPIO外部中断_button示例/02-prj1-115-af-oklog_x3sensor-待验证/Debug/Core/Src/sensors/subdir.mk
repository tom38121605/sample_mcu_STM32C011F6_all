################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensors/icm42605.c \
../Core/Src/sensors/mmc5983.c 

OBJS += \
./Core/Src/sensors/icm42605.o \
./Core/Src/sensors/mmc5983.o 

C_DEPS += \
./Core/Src/sensors/icm42605.d \
./Core/Src/sensors/mmc5983.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su Core/Src/sensors/%.cyclo: ../Core/Src/sensors/%.c Core/Src/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32C011xx -DHSE_VALUE=48000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=48000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/icm42605.cyclo ./Core/Src/sensors/icm42605.d ./Core/Src/sensors/icm42605.o ./Core/Src/sensors/icm42605.su ./Core/Src/sensors/mmc5983.cyclo ./Core/Src/sensors/mmc5983.d ./Core/Src/sensors/mmc5983.o ./Core/Src/sensors/mmc5983.su

.PHONY: clean-Core-2f-Src-2f-sensors

