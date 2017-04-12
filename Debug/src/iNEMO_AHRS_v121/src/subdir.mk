################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/iNEMO_AHRS_v121/src/iNEMO_AHRS.c \
../src/iNEMO_AHRS_v121/src/iNEMO_AHRS_MemMan_1.c \
../src/iNEMO_AHRS_v121/src/iNEMO_EKF.c \
../src/iNEMO_AHRS_v121/src/iNEMO_math.c 

OBJS += \
./src/iNEMO_AHRS_v121/src/iNEMO_AHRS.o \
./src/iNEMO_AHRS_v121/src/iNEMO_AHRS_MemMan_1.o \
./src/iNEMO_AHRS_v121/src/iNEMO_EKF.o \
./src/iNEMO_AHRS_v121/src/iNEMO_math.o 

C_DEPS += \
./src/iNEMO_AHRS_v121/src/iNEMO_AHRS.d \
./src/iNEMO_AHRS_v121/src/iNEMO_AHRS_MemMan_1.d \
./src/iNEMO_AHRS_v121/src/iNEMO_EKF.d \
./src/iNEMO_AHRS_v121/src/iNEMO_math.d 


# Each subdirectory must supply rules for building sources it contributes
src/iNEMO_AHRS_v121/src/%.o: ../src/iNEMO_AHRS_v121/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../libs/misc/include" -I"../include" -I"../libs/CMSIS/include" -I"../libs/StdPeriph/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


