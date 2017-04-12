################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/iNEMO_Button.c \
../src/main.c \
../src/stm32f10x_it.c \
../src/tiny_printf.c \
../src/tm_inemo_nrf24l01.c \
../src/tm_inemo_spi.c 

OBJS += \
./src/iNEMO_Button.o \
./src/main.o \
./src/stm32f10x_it.o \
./src/tiny_printf.o \
./src/tm_inemo_nrf24l01.o \
./src/tm_inemo_spi.o 

C_DEPS += \
./src/iNEMO_Button.d \
./src/main.d \
./src/stm32f10x_it.d \
./src/tiny_printf.d \
./src/tm_inemo_nrf24l01.d \
./src/tm_inemo_spi.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../libs/misc/include" -I"../include" -I"../libs/CMSIS/include" -I"../libs/StdPeriph/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


