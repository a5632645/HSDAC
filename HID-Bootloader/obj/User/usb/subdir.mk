################################################################################
# MRS Version: 2.1.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/usb/ch32v30x_usbhs_device.c \
../User/usb/usb_desc.c 

C_DEPS += \
./User/usb/ch32v30x_usbhs_device.d \
./User/usb/usb_desc.d 

CPP_SRCS += \
../User/usb/usb_conf.cpp 

CPP_DEPS += \
./User/usb/usb_conf.d 

OBJS += \
./User/usb/ch32v30x_usbhs_device.o \
./User/usb/usb_conf.o \
./User/usb/usb_desc.o 



# Each subdirectory must supply rules for building sources it contributes
User/usb/%.o: ../User/usb/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Kawai/Desktop/HSDAC/HID-Bootloader/Debug" -I"c:/Users/Kawai/Desktop/HSDAC/HID-Bootloader/Core" -I"c:/Users/Kawai/Desktop/HSDAC/HID-Bootloader/User" -I"c:/Users/Kawai/Desktop/HSDAC/HID-Bootloader/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User/usb/%.o: ../User/usb/%.cpp
	@	riscv-none-embed-g++ -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -std=gnu++17 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
