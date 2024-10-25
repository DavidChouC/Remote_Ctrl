/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Bsp_Usart.h"
#include "u8g2.h"
#include "KunUI.h"
#include "Bsp_Key.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "nrf24L01.h"
#include "Rocker.h"
/* USER CODE END Includes */
#pragma pack (4)
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void draw(u8g2_t *u8g2);
void Kun_Basketball(u8g2_t *u8g2);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Buff[4]={0};   //摇杆电位器值
float pitch,roll,yaw; 		  //欧拉角
short aacx,aacy,aacz;				//加速度传感器原始数据
short gyrox,gyroy,gyroz;		//陀螺仪原始数据

uint8_t KeyNum_Data[8];   //按键数据
float Mpu6050_Data[22]; 	//陀螺仪数据
uint8_t L_Rocker_Data[11];//左摇杆数据
uint8_t R_Rocker_Data[11];//右摇杆数据
short Speed_Data[8];		  //小车速度数据
uint8_t Mode_Data[3];			//控制模式数据
uint8_t Send_Data[33];		//遥控器下发数据

uint8_t Keynum = 0;			//按键值		
short Speed = 0;				//电机初始速度
uint8_t Mode = 0;       //控制模式 0：摇杆 1：陀螺仪 2：按键
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
typedef void (*app)(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  *@breif      定时器回调函数
	*@param      htim:定时器句柄
  *@retval     none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t i = 0;
	static uint8_t Now_KeyNum = 0;
	static uint8_t Last_KeyNum = 0;
	if(htim->Instance == TIM1)//1ms定时
	{
		i++;i%=20;
		if(i == 0)//20ms
		{
			Last_KeyNum = Now_KeyNum;
			Now_KeyNum = Key_Read();
			if(Now_KeyNum == Last_KeyNum)
			{
				Keynum = Now_KeyNum;
			}
		}

	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    
	HAL_UART_Receive_IT(&huart1,(uint8_t *)RxBuffer,1);//开启串口1接收中断
	
	HAL_TIM_Base_Start_IT(&htim1);										 //开始定时器1中断(1ms)
	
	HAL_TIM_Base_Start(&htim3);												 //定时器触发ADC
	HAL_ADCEx_Calibration_Start(&hadc1);							 //校准ADC
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Buff,4);  //开启DMA
	
	MPU_Init();			  //MPU6050初始化
  mpu_dmp_init();		//dmp初始化

	u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8g2_gpio_and_delay_stm32);
	u8g2_InitDisplay(&u8g2); 		 // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_ClearBuffer(&u8g2);
	
	draw(&u8g2);		//画U8G2库LOGO
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);//设置U8G2显示字体

	while(NRF24L01_Check())//开机自检2.4G模块
	{
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawUTF8(&u8g2,0,32,"NRF24L01 NOT EXIST");
		u8g2_SendBuffer(&u8g2);
		HAL_Delay(500);
	}
	NRF24L01_TX_Mode();//设置为发送模式

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Keynum == 9)
		{
			Speed += 1;
			if(Speed > 10) Speed = 0;
		}
		else if(Keynum == 10)
		{
			Speed -= 1;
			if(Speed < 0) Speed = 10;
		}
		if(Read_L_MTS() == UP && Read_R_MTS() == UP)//摇杆模式
		{
			Mode = 0;
			sprintf((char *)Mode_Data,":%d",Mode);
			
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawUTF8(&u8g2,26,8,"Mode:Rocker");
			
			sprintf((char *)Speed_Data,"Speed:%d",Speed);
			u8g2_DrawUTF8(&u8g2,0,20,(char *)Speed_Data);
			
			sprintf((char *)L_Rocker_Data,"X:%d  Y:%d",L_ROCKER_X/100,L_ROCKER_Y/100);
			u8g2_DrawUTF8(&u8g2,0,32,(char *)L_Rocker_Data);
			
			sprintf((char *)R_Rocker_Data,"X:%d  Y:%d",R_ROCKER_X/100,R_ROCKER_Y/100);
			u8g2_DrawUTF8(&u8g2,0,44,(char *)R_Rocker_Data);
			
			//把摇杆按键值和摇杆电位器数据拼接成一个字符串下发给下位机
			sprintf((char *)Send_Data,"%s%s%s%s",Mode_Data,(char *)Speed_Data,L_Rocker_Data,R_Rocker_Data);
			u8g2_SendBuffer(&u8g2);

		}
		else if(Read_L_MTS() == DOWN && Read_R_MTS() == DOWN)//陀螺仪模式
		{
			Mode = 1;
			sprintf((char *)Mode_Data,":%d",Mode);
			
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawUTF8(&u8g2,20,8,"Mode:Mpu6050");
			
			sprintf((char *)Speed_Data,"S:%d%%",Speed);
			u8g2_DrawUTF8(&u8g2,0,20,(char *)Speed_Data);
			
			while(mpu_dmp_get_data(&pitch, &roll, &yaw));	//必须要用while等待，才能读取成功
			MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		//得到陀螺仪数据
			
			printf("%.1f,%.1f,%.1f \r\n",roll,pitch,yaw);
			
			sprintf((char *)Mpu6050_Data,"X:%.1f    Y:%.1f    Z:%.1f",roll,pitch,yaw);
			strcpy((char *)Send_Data,(char *)Mode_Data);
			strcat((char *)Send_Data,(char *)Mpu6050_Data);

			u8g2_DrawUTF8(&u8g2,0,32,(char *)Mpu6050_Data);
	
			u8g2_SendBuffer(&u8g2);
		}
		else if(Read_L_MTS() == UP && Read_R_MTS() == DOWN)//按键模式
		{
			Mode = 2;
			sprintf((char *)Mode_Data,":%d",Mode);
			
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawUTF8(&u8g2,28,8,"Mode:Key");
			
			sprintf((char *)Speed_Data,"S:%d%%",Speed);
			u8g2_DrawUTF8(&u8g2,0,20,(char *)Speed_Data);
			
			sprintf((char *)Mode_Data,":%d",Mode);
			sprintf((char *)KeyNum_Data,"Num:%d",Key_Read());
			
			strcpy((char *)Send_Data,(char *)Mode_Data);
			strcat((char *)Send_Data,(char *)KeyNum_Data);
			
			u8g2_DrawUTF8(&u8g2,0,32,(char *)KeyNum_Data);
			u8g2_SendBuffer(&u8g2);
		}
		else
		{
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawUTF8(&u8g2,16,8,"Mode Select Error");
			u8g2_SendBuffer(&u8g2);
		}
    if(NRF24L01_TxPacket(Send_Data)==TX_OK)
    {
			memset(Send_Data,0,sizeof(Send_Data));
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/**
  *@breif      IO口和延时初始化
  *@param      none
  *@retval     none
  */
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	switch(msg)
		{
			//Function which implements a delay, arg_int contains the amount of ms
			case U8X8_MSG_DELAY_MILLI:
			{
				HAL_Delay(arg_int);
				break;				
			}
			//Function which delays 10us
			case U8X8_MSG_DELAY_10MICRO:
			{
				for(uint16_t n=0;n<320;n++)
				{
					__NOP();
				}
				break;				
			}
			//Function which delays 100ns
			case U8X8_MSG_DELAY_100NANO:
			{
				__NOP();
				break;
			}
			default:
				return 0; //A message was received which is not implemented, return 0 to indicate an error
	}
	return 1; // command processed successfully.
}

/**
  *@breif      硬件I2C
  *@param      none
  *@retval     none
  */
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;      
      while( arg_int > 0 )
      {
				buffer[buf_idx++] = *data;
				data++;
				arg_int--;
      }      
      break;
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
			HAL_I2C_Master_Transmit(&hi2c1,u8x8_GetI2CAddress(u8x8),buffer,buf_idx,1000);
      break;
    default:
      return 0;
  }
  return 1;
}

/**
  *@breif      画U8g2库的logo
  *@param      none
  *@retval     none
  */
void draw(u8g2_t *u8g2)
{
		u8g2_ClearBuffer(u8g2); 
    u8g2_SetFontMode(u8g2, 1); 
    u8g2_SetFontDirection(u8g2, 0); 
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
	
	u8g2_SendBuffer(u8g2);
	HAL_Delay(1000);
}


void Kun_Basketball(u8g2_t *u8g2)
{
		u8g2_ClearBuffer(u8g2);
		u8g2_DrawXBMP(u8g2,0,0,128,59,Kun_1);
		u8g2_SendBuffer(u8g2);HAL_Delay(50);
		
		u8g2_ClearBuffer(u8g2);
		u8g2_DrawXBMP(u8g2,0,0,128,59,Kun_2);
		u8g2_SendBuffer(u8g2);HAL_Delay(50);

		u8g2_ClearBuffer(u8g2);
		u8g2_DrawXBMP(u8g2,0,0,128,59,Kun_3);
		u8g2_SendBuffer(u8g2);HAL_Delay(50);
		
		u8g2_ClearBuffer(u8g2);
		u8g2_DrawXBMP(u8g2,0,0,128,59,Kun_4);
		u8g2_SendBuffer(u8g2);HAL_Delay(50);
		
		u8g2_ClearBuffer(u8g2);
		u8g2_DrawXBMP(u8g2,0,0,128,59,Kun_5);
		u8g2_SendBuffer(u8g2);HAL_Delay(50);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
