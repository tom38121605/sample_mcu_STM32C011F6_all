################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/drivefile/stm32c0xx_hal.c \
../Core/drivefile/stm32c0xx_hal_cortex.c \
../Core/drivefile/stm32c0xx_hal_flash.c \
../Core/drivefile/stm32c0xx_hal_flash_ex.c 

OBJS += \
./Core/drivefile/stm32c0xx_hal.o \
./Core/drivefile/stm32c0xx_hal_cortex.o \
./Core/drivefile/stm32c0xx_hal_flash.o \
./Core/drivefile/stm32c0xx_hal_flash_ex.o 

C_DEPS += \
./Core/drivefile/stm32c0xx_hal.d \
./Core/drivefile/stm32c0xx_hal_cortex.d \
./Core/drivefile/stm32c0xx_hal_flash.d \
./Core/drivefile/stm32c0xx_hal_flash_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Core/drivefile/%.o Core/drivefile/%.su Core/drivefile/%.cyclo: ../Core/drivefile/%.c Core/drivefile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_FULL_LL_DRIVER -DHSE_VALUE=48000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=48000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32C011xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I"D:/soma_patient_reference_12615/soma_patient_reference_12615/Core/drivefile/hfile" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-drivefile

clean-Core-2f-drivefile:
	-$(RM) ./Core/drivefile/stm32c0xx_hal.cyclo ./Core/drivefile/stm32c0xx_hal.d ./Core/drivefile/stm32c0xx_hal.o ./Core/drivefile/stm32c0xx_hal.su ./Core/drivefile/stm32c0xx_hal_cortex.cyclo ./Core/drivefile/stm32c0xx_hal_cortex.d ./Core/drivefile/stm32c0xx_hal_cortex.o ./Core/drivefile/stm32c0xx_hal_cortex.su ./Core/drivefile/stm32c0xx_hal_flash.cyclo ./Core/drivefile/stm32c0xx_hal_flash.d ./Core/drivefile/stm32c0xx_hal_flash.o ./Core/drivefile/stm32c0xx_hal_flash.su ./Core/drivefile/stm32c0xx_hal_flash_ex.cyclo ./Core/drivefile/stm32c0xx_hal_flash_ex.d ./Core/drivefile/stm32c0xx_hal_flash_ex.o ./Core/drivefile/stm32c0xx_hal_flash_ex.su

.PHONY: clean-Core-2f-drivefile

