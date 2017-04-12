################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_cal.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_dev.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_int.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_pcd.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_core.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_init.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_int.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.c \
../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.c 

OBJS += \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_cal.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_dev.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_int.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_pcd.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_core.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_init.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_int.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.o \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.o 

C_DEPS += \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_cal.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_dev.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_int.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/otgd_fs_pcd.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_core.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_init.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_int.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.d \
./src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.d 


# Each subdirectory must supply rules for building sources it contributes
src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/%.o: ../src/STM32_USB-FS-Device_Lib/Source/Libraries/STM32_USB-FS-Device_Driver/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../libs/misc/include" -I"../include" -I"../libs/CMSIS/include" -I"../libs/StdPeriph/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


