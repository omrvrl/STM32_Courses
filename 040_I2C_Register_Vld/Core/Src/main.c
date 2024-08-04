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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int i = 0;
uint8_t m_address = 0x4E;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void RCC_Config(void);
void GPIO_Config(void);
void I2C_Config(void);
uint8_t I2C_Write(uint8_t , uint8_t );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RCC_Config(void){

	RCC->CFGR |= (0x0110002);							// PLL X6 & SW SOURCE = PLL & PLL ENTRY = PREDIV1
	RCC->CFGR2 |= (0x01);								// PREDIV /2
	RCC->CR |= 0x1090000; 								// HSE & PLL & CSS ON
	wait(RCC->CR & (0x2020000) == HAL_OK);				// WAIT HSE & PLL READY

	RCC->CIR |= (0x0980000); 							// HSE PLL CSS FLAG RESET

}
void GPIO_Config(void){

	RCC->APB2ENR |= (0x0D << 0);  						// AF GPIOA GPIOB CLK ENABLE

	GPIOB->CRH |= (0xDD < 0); 							// B10 & B11 AF_OD - 10MHZ

	GPIOB->CRL |= (8 << 0);								// B1 PUSH BUTTON IN_PP_PD

}

void I2C_Config(void){

	RCC->APB1ENR |= (1 << 22);						// I2C2 CLK ENABLE

	I2C2->CR2 |= (8 << 0);							// FREQ = 8 MHZ
	I2C2->CCR |= (0x028);							// 100 KHZ
	I2C2->TRISE |= (0x9);							//
	I2C2->CR1 |= (1 << 0);							// PERIPH ENABLE
}

uint8_t I2C_Write(uint8_t addres, uint8_t data){

	I2C2->CR1 |= (1 << 8 );						// SEND START BIT
	while(!(I2C2->CR1 & (1 << 0)));				// WAIT START BIT SENT
	I2C2->DR |= 0x4E;							// SLAVE ADDRES
	while(!(I2C2->SR1 & (0x002)));				// WAIT SEND ADDRES
	while(!(I2C2->SR2 & (0x001)));				// WAIT MSL ( MASTER SLAVE MODE)
//	I2C2->DR |= addres;
	while(!(I2C2->SR1 & (0x0070)));				// WAIT DATA REGISTER EMPTY ( TxE )
	while(!(I2C2->SR1 & (1 << 2))); 			// WAIT BYTE TRANSFER FINISHED ( BTF )
	I2C2->CR1 |= (1 << 9); 						// SEND STOP BIT
}

void delay(uint8_t time){
	while(time--);
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
  /* USER CODE BEGIN 2 */
  void RCC_Config(void);
  void GPIO_Config(void);
  void I2C_Config(void);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(GPIOB->IDR == GPIO_PIN_SET){
		  while(GPIOB->IDR & GPIO_PIN_SET);
		  i++;
		  delay(2100000);
	  }

	  switch (i){

	  case 0:
		  I2C_Write(m_address, 0x00);
		  break;
	  case 1:
		  I2C_Write(m_address, 0x01);
		  break;
	  case 2:
		  I2C_Write(m_address, 0x02);
		  break;
	  case 3:
		  I2C_Write(m_address, 0x04);
		  break;
	  case 4:
		  I2C_Write(m_address, 0x08);
		  break;
	  case 5:
		  I2C_Write(m_address, 0x10);
		  break;
	  case 6:
		  I2C_Write(m_address, 0x20);
		  break;
	  case 7:
		  I2C_Write(m_address, 0x40);
		  break;
	  case 8:
		  I2C_Write(m_address, 0x80);
		  break;

	  default:
		  i = 0;
		  break;
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
