################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include STM32L476RG-SensorTile_ALLMEMS2/User/subdir.mk
-include STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/subdir.mk
-include Middlewares/STM32_USBD_Library/Core/subdir.mk
-include Middlewares/STM32_USBD_Library/Class/CDC/subdir.mk
-include Middlewares/STM32_BlueNRG/hci_tl_patterns/Basic/subdir.mk
-include Middlewares/STM32_BlueNRG/Utils/subdir.mk
-include Middlewares/STM32_BlueNRG/HCI/Controller/subdir.mk
-include Middlewares/STM32_BlueNRG/HCI/subdir.mk
-include Middlewares/MetaDataManager/subdir.mk
-include Middlewares/FreeRTOS/Portable/subdir.mk
-include Middlewares/FreeRTOS/CMSIS_RTOS/subdir.mk
-include Middlewares/FreeRTOS/subdir.mk
-include Middlewares/FatFs/Options/subdir.mk
-include Middlewares/FatFs/Core/subdir.mk
-include Drivers/STM32L4xx-Nucleo/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/SensorTile/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := STM32L476RG-SensorTile_ALLMEMS2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32L476RG-SensorTile_ALLMEMS2.elf \

MAP_FILES += \
STM32L476RG-SensorTile_ALLMEMS2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32L476RG-SensorTile_ALLMEMS2.list \

OBJCOPY_BIN += \
STM32L476RG-SensorTile_ALLMEMS2.bin \


# All Target
all: main-build

# Main-build Target
main-build: STM32L476RG-SensorTile_ALLMEMS2.elf secondary-outputs

# Tool invocations
STM32L476RG-SensorTile_ALLMEMS2.elf STM32L476RG-SensorTile_ALLMEMS2.map: $(OBJS) $(USER_OBJS) ../STM32L476RGTx_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32L476RG-SensorTile_ALLMEMS2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../STM32L476RGTx_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32L476RG-SensorTile_ALLMEMS2.map" -Wl,--gc-sections -static -L../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32L476RG-SensorTile_ALLMEMS2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32L476RG-SensorTile_ALLMEMS2.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32L476RG-SensorTile_ALLMEMS2.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32L476RG-SensorTile_ALLMEMS2.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM32L476RG-SensorTile_ALLMEMS2.bin STM32L476RG-SensorTile_ALLMEMS2.elf STM32L476RG-SensorTile_ALLMEMS2.list STM32L476RG-SensorTile_ALLMEMS2.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
