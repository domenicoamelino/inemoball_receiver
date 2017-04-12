################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Desc.c \
../src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Endp.c \
../src/STM32F1_VirtualCom/Source/src/STM32F1_VC_General.c \
../src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Istr.c \
../src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Prop.c \
../src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Pwr.c 

OBJS += \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Desc.o \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Endp.o \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_General.o \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Istr.o \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Prop.o \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Pwr.o 

C_DEPS += \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Desc.d \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Endp.d \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_General.d \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Istr.d \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Prop.d \
./src/STM32F1_VirtualCom/Source/src/STM32F1_VC_Pwr.d 


# Each subdirectory must supply rules for building sources it contributes
src/STM32F1_VirtualCom/Source/src/%.o: ../src/STM32F1_VirtualCom/Source/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../libs/misc/include" -I"../include" -I"../libs/CMSIS/include" -I"../libs/StdPeriph/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


