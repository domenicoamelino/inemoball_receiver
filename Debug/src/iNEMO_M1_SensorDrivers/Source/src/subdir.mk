################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/iNEMO_M1_SensorDrivers/Source/src/iNEMO_I2C_Driver.c \
../src/iNEMO_M1_SensorDrivers/Source/src/iNemo_SPI_Driver.c 

OBJS += \
./src/iNEMO_M1_SensorDrivers/Source/src/iNEMO_I2C_Driver.o \
./src/iNEMO_M1_SensorDrivers/Source/src/iNemo_SPI_Driver.o 

C_DEPS += \
./src/iNEMO_M1_SensorDrivers/Source/src/iNEMO_I2C_Driver.d \
./src/iNEMO_M1_SensorDrivers/Source/src/iNemo_SPI_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
src/iNEMO_M1_SensorDrivers/Source/src/%.o: ../src/iNEMO_M1_SensorDrivers/Source/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../libs/misc/include" -I"../include" -I"../libs/CMSIS/include" -I"../libs/StdPeriph/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


