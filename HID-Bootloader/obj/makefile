################################################################################
# MRS Version: 2.1.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/usb/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
HID-Bootloader.hex \

SECONDARY_LIST += \
HID-Bootloader.lst \

SECONDARY_SIZE += \
HID-Bootloader.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: HID-Bootloader.elf secondary-outputs

# Tool invocations
HID-Bootloader.elf: $(OBJS) $(USER_OBJS)
	@	riscv-none-embed-g++ -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "c:/Users/Kawai/Desktop/HSDAC/HID-Bootloader/Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"HID-Bootloader.map" --specs=nano.specs --specs=nosys.specs -o "HID-Bootloader.elf" $(OBJS) $(USER_OBJS) $(LIBS)
HID-Bootloader.hex: HID-Bootloader.elf
	@	riscv-none-embed-objcopy -O ihex "HID-Bootloader.elf" "HID-Bootloader.hex"
HID-Bootloader.lst: HID-Bootloader.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "HID-Bootloader.elf" > "HID-Bootloader.lst"
HID-Bootloader.siz: HID-Bootloader.elf
	riscv-none-embed-size --format=berkeley "HID-Bootloader.elf"

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(S_DEPS)$(S_UPPER_DEPS)$(ASM_DEPS)$(ASM_UPPER_DEPS)$(C_DEPS)$(C_UPPER_DEPS)$(CC_DEPS)$(C++_DEPS)$(CXX_DEPS)$(CPP_DEPS) HID-Bootloader.elf

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets