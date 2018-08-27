/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SCCB.h
** Descriptions:		SCCB ²Ù×÷º¯Êý¿â 
**
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2011-2-13
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/
#ifndef __SCCB_H
#define __SCCB_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f469xx.h"
#include "stm32f4xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define SCL_H()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)		/* GPIO_SetBits(GPIOB , GPIO_Pin_8)   */
#define SCL_L()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)  /* GPIO_ResetBits(GPIOB , GPIO_Pin_8) */
   
#define SDA_H()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)	  /* GPIO_SetBits(GPIOB , GPIO_Pin_9)   */
#define SDA_L()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)  /* GPIO_ResetBits(GPIOB , GPIO_Pin_9) */

#define SCL_read()      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)   /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_8) */
#define SDA_read()      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)   /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_9) */

#define ADDR_OV7670   0x42

/* Private function prototypes -----------------------------------------------*/
void I2C_Configuration(void);
int I2C_WriteByte( uint16_t WriteAddress , uint8_t SendByte , uint8_t DeviceAddress);
int I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint8_t ReadAddress,  uint8_t DeviceAddress);

#endif 
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
