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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB 61
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if LAB==11
uint8_t num=0;
#elif LAB == 22
#elif LAB == 23
uint32_t ledState ;

#elif LAB == 24
uint32_t ledTemp;
uint32_t initLed = 1;
uint16_t bitNum[8];
uint16_t shift = 0;

#elif LAB == 33
char input;
char display[] = "Input =>";
char newLine[] ="\r\n";
char quit[] = "QUIT";
#elif LAB == 34
char description[]="Display Blinking LED PRESS (r, g) \r\nDisplay Group Members PRESS m \r\nQuit PRESS q\r\n";
char question[]="\tInput =>";
char answer;
char quit[] = "QUIT";
char unknown[] = "Unknown Command \r\n";
char member[]="61010707 \r\nPasawee Laearun \r\n";
int isDisplay=0;
char newLine[] ="\r\n";

#elif LAB == 41
#elif LAB ==01
	uint8_t n;
#elif LAB ==02
char display[] = "07P";
#elif LAB == 54
volatile uint32_t adc_val = 0;
#elif LAB == 55
uint32_t adc_val[8000];
char newLine[] ="\r\n";
//char display[] = "07P";
#endif

#if LAB ==61
uint32_t count;
char newLine[] ="\r\n";
char display[] = "Hellor";
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void bitConvert(uint32_t);
void displayHEX(uint32_t);
void LedControl(GPIO_PinState LED3,GPIO_PinState LED2,GPIO_PinState LED1,GPIO_PinState LED0);
#if LAB ==55
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
 {
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET );
 }

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *AdcHandle)
 {
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
 }
#endif

#if LAB == 61
void displayNumber(uint32_t number){
		char display[8000];
		sprintf(display, "%d ", number);	
		while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
		HAL_UART_Transmit(&huart3, (uint8_t*) display, strlen(display),1000);
		
}
#endif

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	#if LAB == 24
  GPIOG->ODR = 0xFFFFFFFF;
	GPIOF->ODR = 0xFFFFFFFF;
	GPIOE->ODR = 0xFFFFFFFF;
	#endif	
	#if LAB == 34
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
	#endif	
	#if LAB == 54
	HAL_ADC_Start(&hadc1);
	#endif	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	#if LAB ==55
	 HAL_ADC_Start_DMA(&hadc1,adc_val,8000);
	#endif 
	#if LAB == 61
	HAL_TIM_Base_Start_IT(&htim1);
	#endif
 while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
		HAL_UART_Transmit(&huart3, (uint8_t*) newLine, strlen(newLine),1000);


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if LAB == 24
void bitConvert(uint32_t ledTemp){
		for(int index = 0 ; index <= 7 ; index++){
			bitNum[index] = (ledTemp >> index) & 0x01;
		}
		GPIOG->ODR = ~(bitNum[7] <<9 | bitNum[6] << 14);
		GPIOF->ODR =~(bitNum[5] <<15 | bitNum[3] <<14 | bitNum[0] <<13);
		GPIOE->ODR = ~(bitNum[4]<<13 | bitNum[2] << 11 | bitNum[1] <<9); 
}

#elif LAB == 41
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13){
		HAL_UART_Transmit(&huart3,(uint8_t *)"---",3,100);
		HAL_Delay(200);
		for(int i = 0 ; i<20;i++){
			HAL_UART_Transmit(&huart3,(uint8_t *)"B",1,100);
			HAL_Delay(200);
		}
	}
	
	if(GPIO_Pin == GPIO_PIN_0){
		HAL_UART_Transmit(&huart3,(uint8_t *)"---",3,100);
		HAL_Delay(200);
		for(int i = 0 ; i<20;i++){
			HAL_UART_Transmit(&huart3,(uint8_t *)"E",1,100);
			HAL_Delay(200);
		}
	}
}
#elif LAB == 54
void displayHEX(uint32_t input){
		char display[80];
		sprintf(display, "ADC_CH10 0x%08x Vin = %f v raw value = %d \r\n ", input, (input * 3.3)/4095, input);	
		
		while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(&huart3, (uint8_t*) display, strlen(display),1000);
		
		if(input > 0 && input < 819){
		 // LEVEL 1
			LedControl(GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET);
		}
		else if(input > 819 && input < 1638){
		 // LEVEL 2
			LedControl(GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET);
		}	else if(input > 1638 && input < 2457){
		 // LEVEL 2
			LedControl(GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_RESET);
		}
		else if(input > 2457 && input < 3276){
		 // LEVEL 2
			LedControl(GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
		}
		else if(input > 3276 && input < 4095){
		 // LEVEL 2
			LedControl(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
		}
		else{
		}
}

void LedControl(GPIO_PinState LED3,GPIO_PinState LED2,GPIO_PinState LED1,GPIO_PinState LED0){
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,LED0);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,LED1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,LED2);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,LED3);
}
#elif LAB == 55


#endif
#if LAB ==61

#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
