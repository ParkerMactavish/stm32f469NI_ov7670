/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SCCB.c
** Descriptions:		SCCB 操作函数库 
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

/* Includes ------------------------------------------------------------------*/
#include "SCCB.h"


/*******************************************************************************
* Function Name  : I2C_Configuration
* Description    : EEPROM管脚配置
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void I2C_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  /* Configure I2C2 pins: PB10->SCL and PB11->SDA */
	__GPIOB_CLK_ENABLE();
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.Pin =  GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : 延迟时间
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_delay(void)
{	
   uint8_t i = 100; /* 这里可以优化速度 */
   while(i) 
   { 
     i--; 
   } 
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static int I2C_Start(void)
{
	SDA_H();
	SCL_H();
	I2C_delay();
	if(!SDA_read())return DISABLE;	/* SDA线为低电平则总线忙,退出 */
	SDA_L();
	I2C_delay();
	if(SDA_read()) return DISABLE;	/* SDA线为高电平则总线出错,退出 */
	SDA_L();
	I2C_delay();
	return ENABLE;
}

/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_Stop(void)
{
	SCL_L();
	I2C_delay();
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SDA_H();
	I2C_delay();
}

/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_Ack(void)
{	
	SCL_L();
	I2C_delay();
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_NoAck(void)
{	
	SCL_L();
	I2C_delay();
	SDA_H();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : None
* Input          : None
* Output         : None
* Return         : 返回为:=1有ACK,=0无ACK
* Attention		 : None
*******************************************************************************/
static int I2C_WaitAck(void) 	
{
	SCL_L();
	I2C_delay();
	SDA_H();			
	I2C_delay();
	SCL_H();
	I2C_delay();
	if(SDA_read())
	{
      SCL_L();
      return DISABLE;
	}
	SCL_L();
	return ENABLE;
}

 /*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : 数据从高位到低位
* Input          : - SendByte: 发送的数据
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_SendByte(uint8_t SendByte) 
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L();
        I2C_delay();
      if(SendByte&0x80)
        SDA_H();  
      else 
        SDA_L();   
        SendByte<<=1;
        I2C_delay();
		SCL_H();
        I2C_delay();
    }
    SCL_L();
}


/*******************************************************************************
* Function Name  : I2C_ReceiveByte
* Description    : 数据从高位到低位
* Input          : None
* Output         : None
* Return         : I2C总线返回的数据
* Attention		 : None
*******************************************************************************/
static int I2C_ReceiveByte(void)  
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H();				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L();
      I2C_delay();
	  SCL_H();
      I2C_delay();	
      if(SDA_read())
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L();
    return ReceiveByte;
}

/*******************************************************************************
* Function Name  : I2C_WriteByte
* Description    : 写一字节数据
* Input          : - WriteAddress: 待写入地址
*           	   - SendByte: 待写入数据
*                  - DeviceAddress: 器件类型
* Output         : None
* Return         : 返回为:=1成功写入,=0失败
* Attention		 : None
*******************************************************************************/           
int I2C_WriteByte( uint16_t WriteAddress , uint8_t SendByte , uint8_t DeviceAddress)
{		
    if(!I2C_Start())
	{
	    return DISABLE;
	}
    I2C_SendByte( DeviceAddress );                    /* 器件地址 */
    if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return DISABLE;
	}
    I2C_SendByte((uint8_t)(WriteAddress & 0x00FF));   /* 设置低起始地址 */      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
	/* 注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)	*/
    /* Systick_Delay_1ms(10); */
    return ENABLE;
}									 

/*******************************************************************************
* Function Name  : I2C_ReadByte
* Description    : 读取一串数据
* Input          : - pBuffer: 存放读出数据
*           	   - length: 待读出长度
*                  - ReadAddress: 待读出地址
*                  - DeviceAddress: 器件类型
* Output         : None
* Return         : 返回为:=1成功读入,=0失败
* Attention		 : None
*******************************************************************************/          
int I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint8_t ReadAddress,  uint8_t DeviceAddress)
{	
    if(!I2C_Start())
	{
	    return DISABLE;
	}
    I2C_SendByte( DeviceAddress );         /* 器件地址 */
    if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return DISABLE;
	}
    I2C_SendByte( ReadAddress );           /* 设置低起始地址 */      
    I2C_WaitAck();	
    I2C_Stop(); 
	
    if(!I2C_Start())
	{
		return DISABLE;
	}
    I2C_SendByte( DeviceAddress + 1 );               /* 器件地址 */ 
    if(!I2C_WaitAck())
	{
		I2C_Stop(); 
		return DISABLE;
	}
    while(length)
    {
      *pBuffer = I2C_ReceiveByte();
      if(length == 1)
	  {
		  I2C_NoAck();
	  }
      else
	  {
		I2C_Ack(); 
	  }
      pBuffer++;
      length--;
    }
    I2C_Stop();
    return ENABLE;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
