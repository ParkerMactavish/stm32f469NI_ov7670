/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               OV7670.h
** Descriptions:            None
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2011-2-13
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef __OV7670_H
#define __OV7670_H 

/* Includes ------------------------------------------------------------------*/	   
#include "stm32f469xx.h"
#include "main.h"

/*------------------------------------------------------
  模块引脚名称 | 描述            |     STM32端引脚连接 |
  ------------------------------------------------------
  SCCB_SCL     : SCCB时钟        : PB10    I2C2_SCL
  SCCB_SDA     : SCCB数据        : PB11    I2C2_SDA
  CAM_VSYNC    : 帧同步          : PA0     外部中断0
  CAM_HREF     : FIFO            : PB7     GPIO
  CAM_WEN      : FIFO写允许      : PD3     GPIO
  XCLK         : CMOS传感器主时钟: PA8     MCO输出
  CAM_RRST     : FIFO读地址复位  : PE0     GPIO
  CAM_REN      : FIFO片选        : PD6     GPIO
  CAM_RCLK     : FIFO读时钟      : PE1     GPIO
  FIFO D0~D7   : FIFO数据输出    : PC0~PC7 GPIO
  -----------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define RRST_H()	HAL_GPIO_WritePin(RRST_GPIO_Port, RRST_Pin, GPIO_PIN_SET)  /* GPIO_SetBits(GPIOE , FIFO_RRST_PIN)   */
#define RRST_L()	HAL_GPIO_WritePin(RRST_GPIO_Port, RRST_Pin, GPIO_PIN_RESET)	  /* GPIO_ResetBits(GPIOE , FIFO_RRST_PIN) */

#define RCLK_H()	HAL_GPIO_WritePin(RCK_GPIO_Port, RCK_Pin, GPIO_PIN_SET)	  /* GPIO_SetBits(GPIOE , FIFO_RCLK_PIN)   */
#define RCLK_L()	HAL_GPIO_WritePin(RCK_GPIO_Port, RCK_Pin, GPIO_PIN_RESET)	  /* GPIO_ResetBits(GPIOE , FIFO_RCLK_PIN) */

#define WE_H()		HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET)	  /* GPIO_SetBits(GPIOD , FIFO_WE_PIN)   */
#define WE_L()		HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_RESET)	  /* GPIO_ResetBits(GPIOD , FIFO_WE_PIN) */

#define WRST_H()	HAL_GPIO_WritePin(WRST_GPIO_Port, WRST_Pin, GPIO_PIN_SET)
#define WRST_L()	HAL_GPIO_WritePin(WRST_GPIO_Port, WRST_Pin, GPIO_PIN_RESET)

#define OV7670							   0x73
#define OV7670_REG_NUM                     114

/* Private variables ---------------------------------------------------------*/	
extern uint8_t Vsync;

/* Private function prototypes -----------------------------------------------*/
int Sensor_Init(void);
void FIFO_GPIO_Configuration(void);
void OV7670_NVIC_Configuration(void);
void OV7670_EXTI_Configuration(void);
int  OV7670_ReadReg(uint8_t LCD_Reg,uint16_t LCD_RegValue);
int  OV7670_WriteReg(uint8_t LCD_Reg,uint16_t LCD_RegValue);

#endif
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

