/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "stm32_print.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t checkInCircle(uint16_t pos_1_x ,uint16_t pos_1_y ,uint16_t pos_2_x ,uint16_t pos_2_y ,uint16_t radius);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char str[50];
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];
uint32_t tick;
uint32_t last = 0, AM2320_last = 0;
float h=30.0, t=40.0;
uint8_t step = 0;
HAL_StatusTypeDef status;

uint16_t r = 0,g = 0,b = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t CRC16_2(uint8_t*, uint8_t);
void readAM2320(void);
uint16_t convert_565(void);
void clearUART(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	ILI9341_Init();//initial driver setup to drive ili9341
	HAL_TIM_Base_Start_IT(&htim1);
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	
	clearUART();
	Serial.println("Start Timer");
	Serial.println("Start I2C");

	cmdBuffer[0] = 0x03;
	cmdBuffer[1] = 0x00;
	cmdBuffer[2] = 0x04;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		uint16_t colourr = RED;
		uint16_t xr = 160;
		uint16_t yr = 40;
		uint16_t radiusr = 15;
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		ILI9341_Draw_Filled_Circle(xr, yr, radiusr*2, convert_565());
		
		uint16_t small_circle_x = 50;
		uint16_t small_circle_y = 100;
		uint16_t small_circle = 10;
		
		ILI9341_Draw_Filled_Circle(small_circle_x, small_circle_y, small_circle*2, RED);
		
		ILI9341_Draw_Filled_Circle(small_circle_x, small_circle_y+50, small_circle*2, GREEN);
		
		ILI9341_Draw_Filled_Circle(small_circle_x, small_circle_y+100, small_circle*2, BLUE);
		


		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
			if(TP_Touchpad_Pressed())
        {
					
					uint16_t x_pos = 0;
					uint16_t y_pos = 0;
					
				
				
					HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_SET);
					
          uint16_t position_array[2];					
					
					if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
					{
						x_pos = position_array[0];
						y_pos = position_array[1];			
					ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);						
						if( y_pos > 30 & y_pos < 70 && x_pos < 163 && x_pos >120){
							r+=25;
						}
						
						if( y_pos > 30 & y_pos < 70 && x_pos < 110 && x_pos > 70){
							g+=25;
						}
						
						if( y_pos > 30 & y_pos < 70 && x_pos < 60 && x_pos > 20){
							b+=25;
						}
						
						if(r>250){
							r = 0;
							ILI9341_Fill_Screen(WHITE);
						}
						if(g>250){
							g = 0;
							ILI9341_Fill_Screen(WHITE);
						}
						
						if(b>250){
							b = 0;
							ILI9341_Fill_Screen(WHITE);
						}
					
						ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
					
        }
				}
			
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		
	//Frame rate 
	 if(tick - last > 300){
	
		char dht_temp_uff[30];								
		sprintf(dht_temp_uff, "%.1f C", t);						
		ILI9341_Draw_Text(dht_temp_uff, 20, 30, BLACK, 2, WHITE);	
		
		char dht_humi_uff[30];								
		sprintf(dht_humi_uff, "%.1f %%RH", h);						
		ILI9341_Draw_Text(dht_humi_uff, 210, 30, BLACK, 2, WHITE);	
		
		char red_percent_buff[30];		
		sprintf(red_percent_buff, "%.0f %%", ((float)r/250)*100);						
		ILI9341_Draw_Text(red_percent_buff, 260, 90, BLACK, 2, WHITE);	
		
		char green_percent_buff[30];								
		sprintf(green_percent_buff, "%.0f %%", ((float)g/250)*100);						
		ILI9341_Draw_Text(green_percent_buff, 260, 140, BLACK, 2, WHITE);	
		
		char blue_percent_buff[30];								
		sprintf(blue_percent_buff, "%.0f %%", ((float)b/250)*100);						
		ILI9341_Draw_Text(blue_percent_buff, 260, 190, BLACK, 2, WHITE);	
		
		uint16_t start_red_x = 90;
		uint16_t start_red_y = 90;
		uint16_t end_red_y = 120;
		uint16_t box_size  = 15;
		
	
		for(int i = 0 ; i < 10 ;i++){
			// 15 px / box
			float red_percent = ((float)r/250)*100 ;
			float green_percent = ((float)g/250)*100 ;
			float blue_percent = ((float)b/250)*100 ;
			if(i * 10 < red_percent){
				ILI9341_Draw_Filled_Rectangle_Coord(start_red_x,start_red_y,start_red_x+box_size,end_red_y,RED);
			}else{
				ILI9341_Draw_Filled_Rectangle_Coord(start_red_x,start_red_y,start_red_x+box_size,end_red_y,0xFD34);
			}
			
			if(i * 10 < green_percent){
				ILI9341_Draw_Filled_Rectangle_Coord(start_red_x,start_red_y+50,start_red_x+box_size,end_red_y+50,GREEN);
			}else{
				ILI9341_Draw_Filled_Rectangle_Coord(start_red_x,start_red_y+50,start_red_x+box_size,end_red_y+50,0xA7F0);
			}
			
			if(i * 10 < blue_percent){
				ILI9341_Draw_Filled_Rectangle_Coord(start_red_x,start_red_y+100,start_red_x+box_size,end_red_y+100,BLUE);
			}else{
				ILI9341_Draw_Filled_Rectangle_Coord(start_red_x,start_red_y+100,start_red_x+box_size,end_red_y+100,0x865F);
			}
			
			start_red_x +=15;
			
		}
		
	 	last = tick;
	 }
	
	 
	// HAL_Delay(0);
			readAM2320();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CRC16_2(uint8_t* ptr, uint8_t length){
	uint16_t crc = 0xFFFF;
	uint8_t s = 0x00;
	while(length--){
		crc^=*ptr++;
		for(s = 0; s < 8; s++){
			if((crc & 0x01) != 0){
				crc >>= 1;
				crc ^= 0xA001;
			} else crc >>= 1;
		}
	}
	return crc;
}
void readAM2320(){
	if(tick - AM2320_last > 300){
		AM2320_last = tick;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		
		HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200); //wake
		HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200); //read command
		HAL_Delay(1);
		HAL_I2C_Master_Receive(&hi2c1, 0x5c<<1, dataBuffer, 8, 200);
		
		uint16_t Rcrc = dataBuffer[7] << 8;
		Rcrc += dataBuffer[6];
		if(Rcrc == CRC16_2(dataBuffer, 6)){
			uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
			t = temperature / 10.0;
			t = (((dataBuffer[4] & 0x80) >> 7) == 1) ? (t * (-1)) : t;
			
			uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
			h = humidity / 10.0;
		}
	}
}
uint16_t convert_565(){
    uint8_t red   = r;
    uint8_t green = g;
    uint8_t blue  = b;

    uint16_t b = (blue >> 3) & 0x1f;
    uint16_t g = ((green >> 2) & 0x3f) << 5;
    uint16_t r = ((red >> 3) & 0x1f) << 11;

    return (uint16_t) (r | g | b);
}

uint16_t checkInCircle(uint16_t pos_1_x ,uint16_t pos_1_y ,uint16_t pos_2_x ,uint16_t pos_2_y ,uint16_t radius){
	uint32_t distance_x = pos_1_x - pos_1_x;
	uint32_t distance_y = pos_1_y - pos_2_y;
	float distance = sqrt(distance_x * distance_x + distance_y * distance_y);
	if(distance < radius){
		return 1 ;
	}else
	{
		return 0;
	}
}
void clearUART(){
	Serial.printc(27);
	Serial.print("[2J");
	Serial.printc(27);
	Serial.print("[H");
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
