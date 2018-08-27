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
  ģ���������� | ����            |     STM32���������� |
  ------------------------------------------------------
  SCCB_SCL     : SCCBʱ��        : PB10    I2C2_SCL
  SCCB_SDA     : SCCB����        : PB11    I2C2_SDA
  CAM_VSYNC    : ֡ͬ��          : PA0     �ⲿ�ж�0
  CAM_HREF     : FIFO            : PB7     GPIO
  CAM_WEN      : FIFOд����      : PD3     GPIO
  XCLK         : CMOS��������ʱ��: PA8     MCO���
  CAM_RRST     : FIFO����ַ��λ  : PE0     GPIO
  CAM_REN      : FIFOƬѡ        : PD6     GPIO
  CAM_RCLK     : FIFO��ʱ��      : PE1     GPIO
  FIFO D0~D7   : FIFO�������    : PC0~PC7 GPIO
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

