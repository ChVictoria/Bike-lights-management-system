################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/list.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/FreeRTOS/list.o \
./Middlewares/FreeRTOS/queue.o \
./Middlewares/FreeRTOS/tasks.o \
./Middlewares/FreeRTOS/timers.o 

C_DEPS += \
./Middlewares/FreeRTOS/list.d \
./Middlewares/FreeRTOS/queue.d \
./Middlewares/FreeRTOS/tasks.d \
./Middlewares/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/list.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/list.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FreeRTOS/list.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Middlewares/FreeRTOS/queue.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/queue.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FreeRTOS/queue.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Middlewares/FreeRTOS/tasks.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/tasks.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FreeRTOS/tasks.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Middlewares/FreeRTOS/timers.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/timers.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FreeRTOS/timers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Middlewares-2f-FreeRTOS

clean-Middlewares-2f-FreeRTOS:
	-$(RM) ./Middlewares/FreeRTOS/list.cyclo ./Middlewares/FreeRTOS/list.d ./Middlewares/FreeRTOS/list.o ./Middlewares/FreeRTOS/list.su ./Middlewares/FreeRTOS/queue.cyclo ./Middlewares/FreeRTOS/queue.d ./Middlewares/FreeRTOS/queue.o ./Middlewares/FreeRTOS/queue.su ./Middlewares/FreeRTOS/tasks.cyclo ./Middlewares/FreeRTOS/tasks.d ./Middlewares/FreeRTOS/tasks.o ./Middlewares/FreeRTOS/tasks.su ./Middlewares/FreeRTOS/timers.cyclo ./Middlewares/FreeRTOS/timers.d ./Middlewares/FreeRTOS/timers.o ./Middlewares/FreeRTOS/timers.su

.PHONY: clean-Middlewares-2f-FreeRTOS

