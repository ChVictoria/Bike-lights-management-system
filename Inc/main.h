/**
  ******************************************************************************
  * @file    main.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.1.0
  * @date    15-May-2020
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "console.h" 

//#include "osal.h"
//#include "debug.h"
#include "ALLMEMS2_config.h"
#include "TargetFeatures.h"


/* Code for MotionFX integration - Start Section */
#include "MotionFX_Manager.h"
#include "motion_fx.h"
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
#include "MotionAR_Manager.h"
#include "motion_ar.h"
/* Code for MotionAR integration - End Section */
    
/* Code for MotionCP integration - Start Section */
#include "MotionCP_Manager.h"
#include "motion_cp.h"
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
#include "MotionGR_Manager.h"
#include "motion_gr.h"
/* Code for MotionGR integration - End Section */

#include "sensor_service.h"

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

/* Blinking Led functions */
extern void LedBlinkStart(void);
extern void LedBlinkStop(void);
extern void LedBlinkPeriodicStart(void);
extern void LedBlinkPeriodicStop(void);

#ifdef ENABLE_SHUT_DOWN_MODE 
void ShutdownTimeOutStart(void);
void ShutdownTimeOutStop(void);
void ShutdownTimeOutReset(void);
#endif /* ENABLE_SHUT_DOWN_MODE */

extern unsigned char ReCallMagnetoCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveMagnetoCalibrationToMemory(uint16_t dataSize, uint32_t *data);
extern int SendMsgToHost(msgData_t *mailPtr);

#ifdef ALLMEMS2_ENABLE_SD_CARD_LOGGING
  extern void RTC_DataConfig(uint8_t WeekDay, uint8_t Date, uint8_t Month, uint8_t Year);
  extern void RTC_TimeConfig(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
  extern void RTC_AlarmConfig(uint8_t StepHour, uint8_t StepMin, uint8_t StepSec);

  extern void RTC_GetCurrentDateTime(void);
#endif /* ALLMEMS2_ENABLE_SD_CARD_LOGGING */

extern void SystemClock_Config(void);

/* Exported defines and variables  ------------------------------------------------------- */

#ifdef ENABLE_SHUT_DOWN_MODE 
#define SHUT_DOWN_TIME_OUT 20000UL
#endif /* ENABLE_SHUT_DOWN_MODE */

/* Code for MotionFX and MotionGR integration - Start Section */
/* 10kHz/100 For MotionFX@100Hz or MotionGR@100Hz as defaul value */
#define DEFAULT_uhCCR1_Val  100
/* Code for MotionFX and MotionGR integration - End Section */

/* Code for MotionCP integration - Start Section */
/* 10kHz/50 For MotionCP@50Hz as defaul value */
#define DEFAULT_uhCCR2_Val  200
/* Code for MotionCP integration - End Section */

/* Code for MotionAR integration - Start Section */
/* Algorithm frequency [Hz] */
#define ALGO_FREQ_AR      16U

/* Algorithm period [ms] */
#define ALGO_PERIOD_AR    (1000U / ALGO_FREQ_AR) 

/* 10kHz/16 as defaul value for:
  MotionAR@16Hz
 */
#define DEFAULT_uhCCR3_Val      (10000U / ALGO_FREQ_AR)
/* Code for MotionAR integration - End Section */

//10kHz/20  For Acc/Gyro/Mag@20Hz
#define DEFAULT_uhCCR4_Val  500

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;


void Send_msg_BT_terminal(char message[]); //for light management

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
