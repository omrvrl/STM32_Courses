/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void RCC_Config(void);
void Flash_Unlock(void);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Flash_Read(uint32_t address);
void RCC_Config(void);
void Flash_Unlock(void);
void Flash_Erase(void);
void Flash_Write(uint32_t address, uint16_t data);
void Flash_Locker(void);
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
  /* USER CODE BEGIN 2 */

  Flash_Unlock();
  Flash_Erase();
  Flash_Write(0x08002C00, 0xCC);
  Flash_Locker();
  Flash_Unlock();
  data = Flash_Read(0x08002C00);
  Flash_Locker();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void RCC_Config(void){

	RCC->CR |= 0x03;                                    // hsi and hsiready on
	while(!(RCC->CR & (1<<1)));                         //wait hsi ready
	RCC->CR |= (1 << 19);								// CSS ON
	RCC->CFGR &= ~(1 << 16);                            // HSI SOURCE HSI
	RCC->CFGR = 0x0140000;								// pll x7
	RCC->CFGR |= (1<<1);								//PLL AS SYSTEM CLOCK
	RCC->CFGR |= (1 << 14);                             // ADC PRESCALER /4
	RCC->CIR  &= ~(0x0840000);							//flag reset(css and hsi)

}


void Flash_Unlock(void){

	while((FLASH->SR & 0x00010000) != 0);

	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

void Flash_Erase(void){
	while((FLASH->SR & 0x00010000) != 0);			// wait no memory operation

	FLASH->CR |= 1 << 1;							// erase select

	FLASH->AR |= (0x08002C00);						// page 11 selected

	FLASH->CR |= (1 << 6);							// start bit enable

	while((FLASH->SR & 0x00010000) != 0);			// wait no memory operation
}


void Flash_Write(uint32_t address, uint16_t data){

	while((FLASH->SR & 0x00010000) != 0);			// wait no memory operation

	FLASH->CR |= (1 << 0);							// programming chosen

	*(__IO  uint32_t*) address = data;				// assign data to the address

	while((FLASH->SR & 0x00010000) != 0);			// wait no memory operation,

}

uint16_t Flash_Read(uint32_t address){
	uint16_t  my_flashData;

	my_flashData = *(uint32_t*) address;

	while((FLASH->SR & 0x00010000) != 0);			// wait no memory operation

	return my_flashData;

}

void Flash_Locker(void){

	while((FLASH->SR & 0x00010000) != 0);			// wait no memory operation,

	FLASH->CR |= (1 << 7);							// LOCK bit set
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
