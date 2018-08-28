
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32469i_discovery.h"
#include "stm32469i_discovery_lcd.h"
#include "SCCB.h"
#include "OV7670.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart3;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t count=0;
uint32_t vsyncCount=(uint32_t)'0';
uint32_t vsyncStatus=0;
uint32_t photoTakeTimeStart=0;
uint32_t photoTakeTimeEnd=0;
uint32_t photoTakeTimeElapse=0;
char photoTakeTimeElapseHex[6];
uint8_t Data[8];
/**
  * @brief  Retargets the C library printf function to the UART.
	* @param  None
	* @retval None
  */
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	uint8_t Container[2] = {0x44, 0xAA};
	char ContainerInHex[2];
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  /* USER CODE BEGIN 2 */
	I2C_WriteByte(0x12, 0x80, ADDR_OV7670);
	HAL_Delay(50);
//	I2C_WriteByte(0x12, 0x14, ADDR_OV7670);
	HAL_Delay(50);
	//Sensor_Init();
	
	
	I2C_ReadByte(Container, 1, 0x12, ADDR_OV7670);	
	printf("0x12:%X\n\r", Container[0]);
	
	for(uint8_t i=0; i<202; i++)
	{
		I2C_ReadByte(Container, 1, i, ADDR_OV7670);	
		printf("%X:%X\n\r",i, Container[0]);
		HAL_Delay(10);
	}
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	
	//WE_H();
	
	WRST_H();
	RRST_H();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		I2C_ReadByte(Container, 1, 0x11, ADDR_OV7670);
//		HAL_Delay(100);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "It's Working", 12, 0XFFFF);
//		ContainerInHex[0]=Container[0]/16+'0';
//		ContainerInHex[1]=Container[0]%16+'0';
//		HAL_UART_Transmit(&huart3, (uint8_t*)ContainerInHex, 2, 0xFFFF);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 4, 0xFFFF);
//		HAL_UART_Transmit(&huart3, (uint8_t*)&vsyncCount, 1, 0xFFFF);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 4, 0xFFFF);
//		for(int i=5; i>=0; i--)
//		{
//			photoTakeTimeElapseHex[i]=photoTakeTimeElapse%16+'0';
//			photoTakeTimeElapse/=16;
//		}
//		if(vsyncStatus==1) HAL_UART_Transmit(&huart3, (uint8_t*)"Interrupted\r\n", 15, 0xFFFF);
		
//		HAL_UART_Transmit(&huart3, (uint8_t*)photoTakeTimeElapseHex, 6, 0xFFFF);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 4, 0xFFFF);
		
		if( vsyncStatus == 2 )
		{  
//			HAL_UART_Transmit(&huart3, (uint8_t*)"It can turn into 2\n\r", 22, 0xFFFF);
			//WE_L();
			RRST_L(); 
			RCLK_L();
			
			for(int i=0; i<100	; i++);
			
			RCLK_H();
			RRST_H();
			RCLK_L();
			for(int i=0; i<100; i++);
			
			RCLK_H();
			for(int i=0; i<100; i++);
       
//			for( count = 0; count < 76800; count++ )
//			{
			for(int yAxis=0; yAxis<240; yAxis++)
			{
				for(int xAxis=0; xAxis<320; xAxis++)
				{
					RCLK_L();
					
					uint8_t tmpData = 0x00;
					uint32_t redVal=0x00, greenVal=0x00, blueVal=0x00;
					uint8_t tmpDataHex[2];
					uint32_t mixed=0xFF000000;
					
					redVal |=	 ((Data3_GPIO_Port->IDR & Data3_Pin)>>2)
										|((Data4_GPIO_Port->IDR & Data4_Pin)>>5)
										|((Data5_GPIO_Port->IDR & Data5_Pin)>>9)
										|((Data3_GPIO_Port->IDR & Data3_Pin)<<1)
										|((Data4_GPIO_Port->IDR & Data4_Pin)>>2)
										|((Data5_GPIO_Port->IDR & Data5_Pin)>>6)
										|((Data6_GPIO_Port->IDR & Data6_Pin)>>4)
										|((Data7_GPIO_Port->IDR & Data7_Pin));
					
					greenVal |=	 (Data0_GPIO_Port->IDR & Data0_Pin>>8)
											|((Data1_GPIO_Port->IDR & Data1_Pin)<<5)
											|((Data2_GPIO_Port->IDR & Data2_Pin)>>5);
					
	//				tmpData |= (Data0_GPIO_Port->IDR & Data0_Pin>>13)
	//									|((Data1_GPIO_Port->IDR & Data1_Pin))
	//									|((Data2_GPIO_Port->IDR & Data2_Pin)>>10)
	//									|((Data3_GPIO_Port->IDR & Data3_Pin)<<1)
	//									|((Data4_GPIO_Port->IDR & Data4_Pin)>>2)
	//									|((Data5_GPIO_Port->IDR & Data5_Pin)>>6)
	//									|((Data6_GPIO_Port->IDR & Data6_Pin)>>4)
	//									|((Data7_GPIO_Port->IDR & Data7_Pin));	  /* ( GPIO_ReadInputData(GPIOC) << 8 ) & 0xff00; */
					
	//				printf("1:");
	//				if(Data0_GPIO_Port->IDR&Data0_Pin) printf("1"); else printf("0");				
	//				if(Data1_GPIO_Port->IDR&Data1_Pin) printf("1"); else printf("0");
	//				if(Data2_GPIO_Port->IDR&Data2_Pin) printf("1"); else printf("0");
	//				if(Data3_GPIO_Port->IDR&Data3_Pin) printf("1"); else printf("0");
	//				if(Data4_GPIO_Port->IDR&Data4_Pin) printf("1"); else printf("0");
	//				if(Data5_GPIO_Port->IDR&Data5_Pin) printf("1"); else printf("0");
	//				if(Data6_GPIO_Port->IDR&Data6_Pin) printf("1"); else printf("0");
	//				if(Data7_GPIO_Port->IDR&Data7_Pin) printf("1"); else printf("0");
	//				printf("\r\n");
					RCLK_H();
					
					
	//				tmpDataHex[0]=tmpData/16+'0';
	//				tmpDataHex[1]=tmpData%16+'0';
					//HAL_UART_Transmit(&huart3, tmpDataHex, 2, 0xFFFF);
					for(int i=0; i<100; i++);
					
					RCLK_L();
	//				tmpData=0x00;
	//				tmpData |= (Data0_GPIO_Port->IDR & Data0_Pin>>13)
	//									|((Data1_GPIO_Port->IDR & Data1_Pin))
	//									|((Data2_GPIO_Port->IDR & Data2_Pin)>>10)
	//									|((Data3_GPIO_Port->IDR & Data3_Pin)<<1)
	//									|((Data4_GPIO_Port->IDR & Data4_Pin)>>2)
	//									|((Data5_GPIO_Port->IDR & Data5_Pin)>>6)
	//									|((Data6_GPIO_Port->IDR & Data6_Pin)>>4)
	//									|((Data7_GPIO_Port->IDR & Data7_Pin));	  /* ( GPIO_ReadInputData(GPIOC) ) & 0x00ff; */
					greenVal |=  ((Data6_GPIO_Port->IDR & Data6_Pin)>>10)
											|((Data7_GPIO_Port->IDR & Data7_Pin)>>6)
											|((Data5_GPIO_Port->IDR & Data5_Pin)>>9)
											|((Data6_GPIO_Port->IDR & Data6_Pin)>>7)
											|((Data7_GPIO_Port->IDR & Data7_Pin)>>3);	
					
					blueVal |=   ((Data0_GPIO_Port->IDR & Data0_Pin)>>13)
											|((Data1_GPIO_Port->IDR & Data1_Pin))
											|((Data2_GPIO_Port->IDR & Data2_Pin)>>10)
											|((Data0_GPIO_Port->IDR & Data0_Pin)>>10)
											|((Data1_GPIO_Port->IDR & Data1_Pin)<<3)
											|((Data2_GPIO_Port->IDR & Data2_Pin)>>7)
											|((Data3_GPIO_Port->IDR & Data3_Pin)<<4)
											|((Data4_GPIO_Port->IDR & Data4_Pin)<<1);
											
					mixed |= 0xff000000|redVal<<16 | greenVal<<8 | blueVal;
					BSP_LCD_DrawPixel(xAxis, yAxis, mixed);
					RCLK_H();
				}
			}
				
//				tmpDataHex[0]=tmpData/16+'0';
//				tmpDataHex[1]=tmpData%16+'0';
				//HAL_UART_Transmit(&huart3, tmpDataHex, 2, 0xFFFF);

			//}
			//HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 4, 0xFFFF);
			//WE_L();
			RCLK_L();
			WRST_L();
			RRST_L();
			for(int i=0; i<100; i++);
			WRST_H();
			RRST_H();
			
			vsyncStatus = 0;
    }
				
		if(count>500)
		{
			count=0;
			BSP_LED_Toggle(LED1);
		}
	
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DSIHOST init function */
static void MX_DSIHOST_DSI_Init(void)
{

  DSI_PLLInitTypeDef PLLInit;
  DSI_HOST_TimeoutTypeDef HostTimeouts;
  DSI_PHY_TimerTypeDef PhyTimings;
  DSI_VidCfgTypeDef VidCfg;

  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 62;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  VidCfg.VirtualChannelID = 0;
  VidCfg.ColorCoding = DSI_RGB565;
  VidCfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  VidCfg.Mode = DSI_VID_MODE_NB_PULSES;
  VidCfg.PacketSize = 1;
  VidCfg.NumberOfChunks = 640;
  VidCfg.NullPacketSize = 0;
  VidCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  VidCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  VidCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  VidCfg.HorizontalSyncActive = 10;
  VidCfg.HorizontalBackPorch = 9;
  VidCfg.HorizontalLine = 854;
  VidCfg.VerticalSyncActive = 4;
  VidCfg.VerticalBackPorch = 2;
  VidCfg.VerticalFrontPorch = 2;
  VidCfg.VerticalActive = 480;
  VidCfg.LPCommandEnable = DSI_LP_COMMAND_DISABLE;
  VidCfg.LPLargestPacketSize = 0;
  VidCfg.LPVACTLargestPacketSize = 0;
  VidCfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_DISABLE;
  VidCfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_DISABLE;
  VidCfg.LPVerticalActiveEnable = DSI_LP_VACT_DISABLE;
  VidCfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_DISABLE;
  VidCfg.LPVerticalBackPorchEnable = DSI_LP_VBP_DISABLE;
  VidCfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_DISABLE;
  VidCfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;
  if (HAL_DSI_ConfigVideoMode(&hdsi, &VidCfg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins
     PE4   ------> SAI1_FS_A
     PG14   ------> USART6_TX
     PB3   ------> I2S3_CK
     PC12   ------> SDIO_CK
     PE5   ------> SAI1_SCK_A
     PB6   ------> QUADSPI_BK1_NCS
     PD6   ------> SAI1_SD_A
     PC11   ------> SDIO_D3
     PC10   ------> SDIO_D2
     PA12   ------> USB_OTG_FS_DP
     PA11   ------> USB_OTG_FS_DM
     PG9   ------> USART6_RX
     PD2   ------> SDIO_CMD
     PA10   ------> USB_OTG_FS_ID
     PA9   ------> USB_OTG_FS_VBUS
     PC9   ------> SDIO_D1
     PC8   ------> SDIO_D0
     PH4   ------> I2C2_SCL
     PH5   ------> I2C2_SDA
     PG7   ------> SAI1_MCLK_A
     PF7   ------> QUADSPI_BK1_IO2
     PF6   ------> QUADSPI_BK1_IO3
     PF10   ------> QUADSPI_CLK
     PF9   ------> QUADSPI_BK1_IO1
     PF8   ------> QUADSPI_BK1_IO0
     PD13   ------> S_TIM4_CH2
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPKR_HP_Pin|AUDIO_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WR_Pin|RCK_Pin|WRST_Pin|RRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin|SIOC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIOD_Pin|LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SAI1_FSA_Pin SAI1_SCKA_Pin */
  GPIO_InitStruct.Pin = SAI1_FSA_Pin|SAI1_SCKA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPKR_HP_Pin AUDIO_RST_Pin */
  GPIO_InitStruct.Pin = SPKR_HP_Pin|AUDIO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_USART6_TX_Pin USART6_RX_Pin */
  GPIO_InitStruct.Pin = ARDUINO_USART6_TX_Pin|USART6_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_CK_Pin */
  GPIO_InitStruct.Pin = I2S3_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : uSD_CLK_Pin uSD_D3_Pin uSD_D2_Pin uSD_D1_Pin 
                           uSD_D0_Pin */
  GPIO_InitStruct.Pin = uSD_CLK_Pin|uSD_D3_Pin|uSD_D2_Pin|uSD_D1_Pin 
                          |uSD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Data0_Pin Data5_Pin Data2_Pin Data6_Pin 
                           uSD_Detect_Pin */
  GPIO_InitStruct.Pin = Data0_Pin|Data5_Pin|Data2_Pin|Data6_Pin 
                          |uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS1_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS1_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_BK1_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(QSPI_BK1_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_DATA_Pin */
  GPIO_InitStruct.Pin = MIC_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(MIC_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS1_P_Pin USB_FS1_N_Pin USB_FS1_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS1_P_Pin|USB_FS1_N_Pin|USB_FS1_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_CMD_Pin */
  GPIO_InitStruct.Pin = uSD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(uSD_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS1_Pin */
  GPIO_InitStruct.Pin = VBUS_FS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI1_MCLKA_Pin */
  GPIO_InitStruct.Pin = SAI1_MCLKA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(SAI1_MCLKA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO2_Pin QSPI_BK1_IO3_Pin QSPI_CLK_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO2_Pin|QSPI_BK1_IO3_Pin|QSPI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO1_Pin QSPI_BK1_IO0_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO1_Pin|QSPI_BK1_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : WR_Pin RCK_Pin WRST_Pin RRST_Pin */
  GPIO_InitStruct.Pin = WR_Pin|RCK_Pin|WRST_Pin|RRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS1_PowerSwitchOn_Pin EXT_RESET_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CK_Pin */
  GPIO_InitStruct.Pin = MIC_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(MIC_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Data1_Pin Data3_Pin Data4_Pin Data7_Pin */
  GPIO_InitStruct.Pin = Data1_Pin|Data3_Pin|Data4_Pin|Data7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SIOD_Pin */
  GPIO_InitStruct.Pin = SIOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SIOD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_Pin */
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SIOC_Pin */
  GPIO_InitStruct.Pin = SIOC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SIOC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==VSYNC_Pin)
	{
		if(vsyncStatus==0)
		{
			//WE_L();
			WE_H();
			photoTakeTimeStart=HAL_GetTick();
			vsyncStatus=1;
		}
		else if(vsyncStatus==1)
		{
			//WE_H();
			WE_L();
			photoTakeTimeEnd=HAL_GetTick();
			photoTakeTimeElapse=photoTakeTimeEnd-photoTakeTimeStart;
			vsyncStatus=2;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
