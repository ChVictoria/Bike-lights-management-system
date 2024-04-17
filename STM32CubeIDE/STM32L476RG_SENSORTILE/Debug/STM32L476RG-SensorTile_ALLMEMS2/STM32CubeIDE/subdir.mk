################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Projects/STM32L476RG-SensorTile/Applications/ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.s 

C_SRCS += \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Projects/STM32L476RG-SensorTile/Applications/ALLMEMS2/STM32CubeIDE/syscalls.c 

OBJS += \
./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.o \
./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.o 

S_DEPS += \
./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.d 

C_DEPS += \
./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Projects/STM32L476RG-SensorTile/Applications/ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.s STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I../../../Inc -x assembler-with-cpp -MMD -MP -MF"STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@" "$<"
STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Projects/STM32L476RG-SensorTile/Applications/ALLMEMS2/STM32CubeIDE/syscalls.c STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-STM32L476RG-2d-SensorTile_ALLMEMS2-2f-STM32CubeIDE

clean-STM32L476RG-2d-SensorTile_ALLMEMS2-2f-STM32CubeIDE:
	-$(RM) ./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.d ./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/startup_stm32l476xx.o ./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.cyclo ./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.d ./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.o ./STM32L476RG-SensorTile_ALLMEMS2/STM32CubeIDE/syscalls.su

.PHONY: clean-STM32L476RG-2d-SensorTile_ALLMEMS2-2f-STM32CubeIDE

