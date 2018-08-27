/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SCCB.c
** Descriptions:		SCCB ���������� 
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
* Description    : EEPROM�ܽ�����
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
* Description    : �ӳ�ʱ��
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_delay(void)
{	
   uint8_t i = 100; /* ��������Ż��ٶ� */
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
	if(!SDA_read())return DISABLE;	/* SDA��Ϊ�͵�ƽ������æ,�˳� */
	SDA_L();
	I2C_delay();
	if(SDA_read()) return DISABLE;	/* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
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
* Return         : ����Ϊ:=1��ACK,=0��ACK
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
* Description    : ���ݴӸ�λ����λ
* Input          : - SendByte: ���͵�����
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
* Description    : ���ݴӸ�λ����λ
* Input          : None
* Output         : None
* Return         : I2C���߷��ص�����
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
* Description    : дһ�ֽ�����
* Input          : - WriteAddress: ��д���ַ
*           	   - SendByte: ��д������
*                  - DeviceAddress: ��������
* Output         : None
* Return         : ����Ϊ:=1�ɹ�д��,=0ʧ��
* Attention		 : None
*******************************************************************************/           
int I2C_WriteByte( uint16_t WriteAddress , uint8_t SendByte , uint8_t DeviceAddress)
{		
    if(!I2C_Start())
	{
	    return DISABLE;
	}
    I2C_SendByte( DeviceAddress );                    /* ������ַ */
    if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return DISABLE;
	}
    I2C_SendByte((uint8_t)(WriteAddress & 0x00FF));   /* ���õ���ʼ��ַ */      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
	/* ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)	*/
    /* Systick_Delay_1ms(10); */
    return ENABLE;
}									 

/*******************************************************************************
* Function Name  : I2C_ReadByte
* Description    : ��ȡһ������
* Input          : - pBuffer: ��Ŷ�������
*           	   - length: ����������
*                  - ReadAddress: ��������ַ
*                  - DeviceAddress: ��������
* Output         : None
* Return         : ����Ϊ:=1�ɹ�����,=0ʧ��
* Attention		 : None
*******************************************************************************/          
int I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint8_t ReadAddress,  uint8_t DeviceAddress)
{	
    if(!I2C_Start())
	{
	    return DISABLE;
	}
    I2C_SendByte( DeviceAddress );         /* ������ַ */
    if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return DISABLE;
	}
    I2C_SendByte( ReadAddress );           /* ���õ���ʼ��ַ */      
    I2C_WaitAck();	
    I2C_Stop(); 
	
    if(!I2C_Start())
	{
		return DISABLE;
	}
    I2C_SendByte( DeviceAddress + 1 );               /* ������ַ */ 
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
