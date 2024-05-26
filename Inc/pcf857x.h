/*
 * pcf857x.h
 *
 *  Created on: May 20, 2024
 *      Author: power
 */

#ifndef INC_PCF857X_H_
#define INC_PCF857X_H_

#include "stdint.h"
#include "stdbool.h"

//-----------User Defines---------------//
#define STM32_I2C_PORT			hbusi2c3
#define PCF857x_ADDRESS			0x23
//--------------------------------------//

#define PCF857x_I2C_PORT		STM32_I2C_PORT
#define PCF857x_I2C_ADDR        PCF857x_ADDRESS<<1 // 0x23<<1 = 0x46

#include "stm32l4xx_hal.h"


#define	_BIT_SET(reg, bit)		reg |= 1 << bit
#define	_BIT_CLEAR(reg, bit)		reg &= ~(1 << bit)
#define	_BIT_TOGGLE(reg, bit)	reg ^= 1 << bit
#define	_BIT_IS_SET(reg, bit)	(reg & (1 << bit)) != 0
#define	_BIT_IS_CLEAR(reg, bit)	(reg & (1 << bit)) == 0

#define PCF857X_IS_READY() HAL_I2C_IsDeviceReady(&PCF857x_I2C_PORT, PCF857x_I2C_ADDR, 5, 1000)
#define PCF857X_READ(data, num_bytes) HAL_I2C_Master_Receive(&PCF857x_I2C_PORT, PCF857x_I2C_ADDR, data, num_bytes, 1000)
#define PCF857X_WRITE(data, num_bytes) HAL_I2C_Master_Transmit(&PCF857x_I2C_PORT, PCF857x_I2C_ADDR, data, num_bytes, 1000)

typedef enum{
	PCF857x_OK 			= 0x00,
	PCF857x_PIN_ERROR 	= 0x01,
	PCF857x_I2C_ERROR 	= 0x02,
	PCF857x_FUN_ERROR	= 0x03,
	PCF857x_VAL_ERROR	= 0x04
}PCF857x_TypeDef;

extern I2C_HandleTypeDef PCF857x_I2C_PORT;

PCF857x_TypeDef pcf857x_Init(uint16_t value_init);
uint8_t pcf857x_Read8(void);
uint16_t pcf857x_Read16(void);
bool pcf857x_Read(uint8_t pin);

PCF857x_TypeDef pcf857x_Write8(uint8_t value);
PCF857x_TypeDef pcf857x_Write16(uint16_t value);
PCF857x_TypeDef pcf857x_Write(uint8_t pin, bool value);

PCF857x_TypeDef pcf857x_Toggle(uint8_t pin);
PCF857x_TypeDef pcf857x_ToggleAll(void);
PCF857x_TypeDef pcf857x_ShiftRight(uint8_t n);
PCF857x_TypeDef pcf857x_ShiftLeft(uint8_t n);
PCF857x_TypeDef pcf857x_RotateRight(uint8_t n);
PCF857x_TypeDef pcf857x_RotateLeft(uint8_t n);
PCF857x_TypeDef pcf857x_ResetInterruptPin(void);
PCF857x_TypeDef pcf857x_GetLastError(void);



#endif /* INC_PCF857X_H_ */
