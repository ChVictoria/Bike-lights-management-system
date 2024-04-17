/**
  ******************************************************************************
  * @file    ALLMEMS2_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.1.0
  * @date    15-May-2020
  * @brief   FP-SNS-ALLMEMS2 configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ALLMEMS2_CONFIG_H
#define __ALLMEMS2_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* For enabling SD card recording */
#define ALLMEMS2_ENABLE_SD_CARD_LOGGING

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3


/* IMPORTANT 
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms

if QUAT_UPDATE_MUL_10MS!=3, then SEND_N_QUATERNIONS must be ==1
*/

/* Uncomment the following define for changing the default BLE Advertise Interval
 * This define will reduce the Power Consumption but will increase the board discovery section.
 * It's necessary to use the latest Android/iOS application. */
//#define BLE_CHANGE_ADV_INTERVAL

/*************** Debug Defines ******************/
/* For enabling the printf on UART */
/* Enabling this define for SensorTile..
 * it will introduce a delay of 10Seconds before starting the application
 * for having time to open the Terminal
 * for looking the ALLMEMS2 Initialization phase */
#ifndef ALLMEMS2_ENABLE_SD_CARD_LOGGING
  #define ALLMEMS2_ENABLE_PRINTF
#endif /* ALLMEMS2_ENABLE_SD_CARD_LOGGING */

/* For enabling connection and notification subscriptions debug */
#define ALLMEMS2_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
//#define ALLMEMS2_DEBUG_NOTIFY_TRAMISSION

/* Define the default transmission interval for Microphones dB Values */
#define DEFAULT_AUDIO_LEV_PERIOD 50

/* Define the default transmission interval for enviromental sensors */
#define DEFAULT_ENV_PERIOD 500
   
/* Define the default transmission interval for battery features */
#define DEFAULT_BATTERY_FEATURES_PERIOD 500


/*************** Don't Change the following defines *************/

/* Motion Sensor Instance */
#define ACCELERO_INSTANCE        LSM6DSM_0
#define GYRO_INSTANCE            LSM6DSM_0
#define MAGNETO_INSTANCE         LSM303AGR_MAG_0

/* Environmental Sensor Instance */
#define TEMPERATURE_INSTANCE_1  HTS221_0
#define HUMIDITY_INSTANCE       HTS221_0
#define TEMPERATURE_INSTANCE_2  LPS22HB_0
#define PRESSURE_INSTANCE       LPS22HB_0
     
/* Motion Sensor API */
#define MOTION_SENSOR_Init                    BSP_MOTION_SENSOR_Init
#define MOTION_SENSOR_Enable                  BSP_MOTION_SENSOR_Enable
#define MOTION_SENSOR_Disable                 BSP_MOTION_SENSOR_Disable

#define MOTION_SENSOR_AxesRaw_t               BSP_MOTION_SENSOR_AxesRaw_t
#define MOTION_SENSOR_Axes_t                  BSP_MOTION_SENSOR_Axes_t
#define MOTION_SENSOR_Event_Status_t          BSP_MOTION_SENSOR_Event_Status_t

#define MOTION_SENSOR_INT2_PIN                BSP_MOTION_SENSOR_INT2_PIN

#define MOTION_SENSOR_SetOutputDataRate                 BSP_MOTION_SENSOR_SetOutputDataRate
#define MOTION_SENSOR_Disable_Wake_Up_Detection         BSP_MOTION_SENSOR_Disable_Wake_Up_Detection
#define MOTION_SENSOR_Enable_Free_Fall_Detection        BSP_MOTION_SENSOR_Enable_Free_Fall_Detection
#define MOTION_SENSOR_GetOutputDataRate                 BSP_MOTION_SENSOR_GetOutputDataRate
#define MOTION_SENSOR_Enable_6D_Orientation             BSP_MOTION_SENSOR_Enable_6D_Orientation
#define MOTION_SENSOR_Disable_6D_Orientation            BSP_MOTION_SENSOR_Disable_6D_Orientation
#define MOTION_SENSOR_Get_6D_Orientation_XL             BSP_MOTION_SENSOR_Get_6D_Orientation_XL
#define MOTION_SENSOR_Get_6D_Orientation_XH             BSP_MOTION_SENSOR_Get_6D_Orientation_XH
#define MOTION_SENSOR_Get_6D_Orientation_YL             BSP_MOTION_SENSOR_Get_6D_Orientation_YL
#define MOTION_SENSOR_Get_6D_Orientation_YH             BSP_MOTION_SENSOR_Get_6D_Orientation_YH
#define MOTION_SENSOR_Get_6D_Orientation_ZL             BSP_MOTION_SENSOR_Get_6D_Orientation_ZL
#define MOTION_SENSOR_Get_6D_Orientation_ZH             BSP_MOTION_SENSOR_Get_6D_Orientation_ZH
#define MOTION_SENSOR_Enable_Tilt_Detection             BSP_MOTION_SENSOR_Enable_Tilt_Detection
#define MOTION_SENSOR_Disable_Tilt_Detection            BSP_MOTION_SENSOR_Disable_Tilt_Detection
#define MOTION_SENSOR_Enable_Wake_Up_Detection          BSP_MOTION_SENSOR_Enable_Wake_Up_Detection
#define MOTION_SENSOR_Disable_Free_Fall_Detection       BSP_MOTION_SENSOR_Disable_Free_Fall_Detection
#define MOTION_SENSOR_Enable_Double_Tap_Detection       BSP_MOTION_SENSOR_Enable_Double_Tap_Detection
#define MOTION_SENSOR_Disable_Double_Tap_Detection      BSP_MOTION_SENSOR_Disable_Double_Tap_Detection
#define MOTION_SENSOR_Enable_Single_Tap_Detection       BSP_MOTION_SENSOR_Enable_Single_Tap_Detection
#define MOTION_SENSOR_Disable_Single_Tap_Detection      BSP_MOTION_SENSOR_Disable_Single_Tap_Detection
#define MOTION_SENSOR_Enable_Pedometer                  BSP_MOTION_SENSOR_Enable_Pedometer
#define MOTION_SENSOR_Disable_Pedometer                 BSP_MOTION_SENSOR_Disable_Pedometer
#define MOTION_SENSOR_Reset_Step_Counter                BSP_MOTION_SENSOR_Reset_Step_Counter
#define MOTION_SENSOR_Get_Step_Count                    BSP_MOTION_SENSOR_Get_Step_Count
#define MOTION_SENSOR_SetFullScale                      BSP_MOTION_SENSOR_SetFullScale
#define MOTION_SENSOR_GetSensitivity                    BSP_MOTION_SENSOR_GetSensitivity
#define MOTION_SENSOR_SetFullScale                      BSP_MOTION_SENSOR_SetFullScale
#define MOTION_SENSOR_Get_Event_Status                  BSP_MOTION_SENSOR_Get_Event_Status
#define MOTION_SENSOR_GetAxes                           BSP_MOTION_SENSOR_GetAxes
#define MOTION_SENSOR_GetAxesRaw                        BSP_MOTION_SENSOR_GetAxesRaw

/* Environmental Sensor API */
#define ENV_SENSOR_Init               BSP_ENV_SENSOR_Init
#define ENV_SENSOR_GetValue           BSP_ENV_SENSOR_GetValue
#define ENV_SENSOR_Enable             BSP_ENV_SENSOR_Enable
#define ENV_SENSOR_Disable            BSP_ENV_SENSOR_Disable
#define ENV_SENSOR_Set_One_Shot       BSP_ENV_SENSOR_Set_One_Shot

/* Package Version only numbers 0->9 */
#define ALLMEMS2_VERSION_MAJOR '2'
#define ALLMEMS2_VERSION_MINOR '1'
#define ALLMEMS2_VERSION_PATCH '0'

/* Define the ALLMEMS2 Name MUST be 7 char long */
#define NAME_BLUEMS 'A','M','2','V',ALLMEMS2_VERSION_MAJOR,ALLMEMS2_VERSION_MINOR,ALLMEMS2_VERSION_PATCH

/* Package Name */
#define ALLMEMS2_PACKAGENAME "FP-SNS-ALLMEMS2"
 
/* Code for BlueVoice integration - Start Section */
#define BV_AUDIO_SAMPLING_FREQUENCY     8000
#define BV_AUDIO_VOLUME_VALUE           64
#define PCM_IN_SAMPLES_PER_MS           (((uint16_t)BV_AUDIO_SAMPLING_FREQUENCY)/1000)
#define AUDIO_IN_MS                     (1)       /*!< Number of ms of Audio given as input to the BlueVoice library.*/ 
#define BV_PCM_AUDIO_IN_SAMPLES         (PCM_IN_SAMPLES_PER_MS * AUDIO_IN_MS)
/* Code for BlueVoice integration - End Section */

#ifdef ALLMEMS2_ENABLE_PRINTF
  //#define DEBUG_ACC_EVENT_REG

  #include "usbd_cdc_interface.h"

  extern char TmpBufferToWrite[256];
  extern int32_t TmpBytesToWrite;
    
  #define ALLMEMS2_PRINTF(...) {\
    TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
    CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
  }
#else /* ALLMEMS2_ENABLE_PRINTF */
  #define ALLMEMS2_PRINTF(...)
#endif /* ALLMEMS2_ENABLE_PRINTF */

/* STM32 Unique ID */
#define STM32_UUID ((uint32_t *)0x1FFF7590)

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif /* __ALLMEMS2_CONFIG_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
