################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/pcm1774/PCM1774.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/pcm1774/PCM1774_reg.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/stc3115/STC3115_Driver.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/hts221/hts221.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/hts221/hts221_reg.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lps22hb/lps22hb.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lps22hb/lps22hb_reg.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm303agr/lsm303agr.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm303agr/lsm303agr_reg.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm6dsm/lsm6dsm.c \
C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm6dsm/lsm6dsm_reg.c 

OBJS += \
./Drivers/BSP/Components/PCM1774.o \
./Drivers/BSP/Components/PCM1774_reg.o \
./Drivers/BSP/Components/STC3115_Driver.o \
./Drivers/BSP/Components/hts221.o \
./Drivers/BSP/Components/hts221_reg.o \
./Drivers/BSP/Components/lps22hb.o \
./Drivers/BSP/Components/lps22hb_reg.o \
./Drivers/BSP/Components/lsm303agr.o \
./Drivers/BSP/Components/lsm303agr_reg.o \
./Drivers/BSP/Components/lsm6dsm.o \
./Drivers/BSP/Components/lsm6dsm_reg.o 

C_DEPS += \
./Drivers/BSP/Components/PCM1774.d \
./Drivers/BSP/Components/PCM1774_reg.d \
./Drivers/BSP/Components/STC3115_Driver.d \
./Drivers/BSP/Components/hts221.d \
./Drivers/BSP/Components/hts221_reg.d \
./Drivers/BSP/Components/lps22hb.d \
./Drivers/BSP/Components/lps22hb_reg.d \
./Drivers/BSP/Components/lsm303agr.d \
./Drivers/BSP/Components/lsm303agr_reg.d \
./Drivers/BSP/Components/lsm6dsm.d \
./Drivers/BSP/Components/lsm6dsm_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/PCM1774.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/pcm1774/PCM1774.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/PCM1774.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/PCM1774_reg.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/pcm1774/PCM1774_reg.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/PCM1774_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/STC3115_Driver.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/stc3115/STC3115_Driver.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/STC3115_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/hts221.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/hts221/hts221.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/hts221.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/hts221_reg.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/hts221/hts221_reg.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/hts221_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/lps22hb.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lps22hb/lps22hb.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/lps22hb.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/lps22hb_reg.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lps22hb/lps22hb_reg.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/lps22hb_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/lsm303agr.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm303agr/lsm303agr.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/lsm303agr.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/lsm303agr_reg.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm303agr/lsm303agr_reg.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/lsm303agr_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/lsm6dsm.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm6dsm/lsm6dsm.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/lsm6dsm.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/Components/lsm6dsm_reg.o: C:/Users/power/OneDrive/Dokumente/6\ sem/Embedded/STM32CubeFunctionPack_ALLMEMS2_V2.1.0/Drivers/BSP/Components/lsm6dsm/lsm6dsm_reg.c Drivers/BSP/Components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE -DALLMEMS2 -DPRIORITY_RTOS -c -I../../../Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../../Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Drivers/BSP/Components/pcm1774 -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/Components/lsm6dsm_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components

clean-Drivers-2f-BSP-2f-Components:
	-$(RM) ./Drivers/BSP/Components/PCM1774.cyclo ./Drivers/BSP/Components/PCM1774.d ./Drivers/BSP/Components/PCM1774.o ./Drivers/BSP/Components/PCM1774.su ./Drivers/BSP/Components/PCM1774_reg.cyclo ./Drivers/BSP/Components/PCM1774_reg.d ./Drivers/BSP/Components/PCM1774_reg.o ./Drivers/BSP/Components/PCM1774_reg.su ./Drivers/BSP/Components/STC3115_Driver.cyclo ./Drivers/BSP/Components/STC3115_Driver.d ./Drivers/BSP/Components/STC3115_Driver.o ./Drivers/BSP/Components/STC3115_Driver.su ./Drivers/BSP/Components/hts221.cyclo ./Drivers/BSP/Components/hts221.d ./Drivers/BSP/Components/hts221.o ./Drivers/BSP/Components/hts221.su ./Drivers/BSP/Components/hts221_reg.cyclo ./Drivers/BSP/Components/hts221_reg.d ./Drivers/BSP/Components/hts221_reg.o ./Drivers/BSP/Components/hts221_reg.su ./Drivers/BSP/Components/lps22hb.cyclo ./Drivers/BSP/Components/lps22hb.d ./Drivers/BSP/Components/lps22hb.o ./Drivers/BSP/Components/lps22hb.su ./Drivers/BSP/Components/lps22hb_reg.cyclo ./Drivers/BSP/Components/lps22hb_reg.d ./Drivers/BSP/Components/lps22hb_reg.o ./Drivers/BSP/Components/lps22hb_reg.su ./Drivers/BSP/Components/lsm303agr.cyclo ./Drivers/BSP/Components/lsm303agr.d ./Drivers/BSP/Components/lsm303agr.o ./Drivers/BSP/Components/lsm303agr.su ./Drivers/BSP/Components/lsm303agr_reg.cyclo ./Drivers/BSP/Components/lsm303agr_reg.d ./Drivers/BSP/Components/lsm303agr_reg.o ./Drivers/BSP/Components/lsm303agr_reg.su ./Drivers/BSP/Components/lsm6dsm.cyclo ./Drivers/BSP/Components/lsm6dsm.d ./Drivers/BSP/Components/lsm6dsm.o ./Drivers/BSP/Components/lsm6dsm.su ./Drivers/BSP/Components/lsm6dsm_reg.cyclo ./Drivers/BSP/Components/lsm6dsm_reg.d ./Drivers/BSP/Components/lsm6dsm_reg.o ./Drivers/BSP/Components/lsm6dsm_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components
