/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention                    <<<--------------------------------------------!! KARŞILAŞTIĞIM SORUNLAR ÖNEMLİ !! -----------------
  *
  *	1- PERİHPHERAL ENABLE BİTİ HER YAZMA İŞLEMİNDEN ÖNCE SET EDİLMELİ
  *	2- GPIO PİNLERİ MODLARINA DİKKAT ET SET ETTİĞİN BİTLER DIŞINDA RESET YAPMAN GEREKEBİLİR ( 10 YAZILMASI GEREKİYOR FAKAT GPIOA-> CRL |= (1 << 1) YETERLİ DEĞİL, &= ~(1<<0) LAZIM
  * 3- I2C2->DR REGISTERLARINA SET İŞLEMİNİ DİREKT = İLE YAP, |= İLE DEĞİL. ( I2C2->DR = 23 ✓ , I2C2->DR |= 23 ✕ )
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
#define m_address 0x4E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
uint8_t i;
uint16_t j = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void RCC_Config(void);
void Gpio_Config(void);
void I2C_Config(void);
void I2C_Write(uint8_t address, uint8_t data);
//void I2C_Read(uint8_t address, uint8_t data);
void delay(uint32_t time);
void Scan_Slave_Address(void);



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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  RCC_Config();
  Gpio_Config();
  I2C_Config();
  Scan_Slave_Address();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(GPIOA->IDR & (0x1) ){
		  while(GPIOA->IDR & GPIO_PIN_SET);
		  i++;
		  delay(2100000);
	  }

	  switch (i){

	  case 0:
		  I2C_Write(m_address,0X00);
		  break;
	  case 1:
		  I2C_Write(m_address,0X01);
		  break;
	  case 2:
		  I2C_Write(m_address,0X02);
		  break;
	  case 3:
		  I2C_Write(m_address,0x04);
		  break;
	  case 4:
		  I2C_Write(m_address,0x08);
		  break;
	  case 5:
		  I2C_Write(m_address,0x10);
		  break;
	  case 6:
		  I2C_Write(m_address,0x20);
		  break;
	  case 7:
		  I2C_Write(m_address,0x40);
		  break;
	  case 8:
		  I2C_Write(m_address,0x80);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Scan_Slave_Address(void){

	for(j=0; j <= 255 ; j++){
		if(HAL_I2C_IsDeviceReady(&hi2c2, j, 1, 100) == HAL_OK)
			break;
	}
}


void delay(uint32_t time){
	while(time--);
}

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

void Gpio_Config(void){

	RCC->APB2ENR |= (0x0D << 0);  						// AF GPIOA GPIOB CLK ENABLE

	GPIOA->CRL |= (1<<3);								// A0 input
	GPIOA->CRL &= ~(1<<2);

	GPIOB->CRH |= (0xDD00); 							// B10 & B11 AF_OD - 10MHZ

}

void I2C_Config(void){

	RCC->APB1ENR |= (1 << 22); 							// I2C2 ENABLE

	I2C2->CR2 |= 0;
	I2C2->TRISE |= 0x009;
	I2C2->CR2 |= 0b01000;								// FREQ= 8 MHZ
	I2C2->CCR |= (0x8028);								// 100 khz scl freq
	I2C2->CR1 |= (1 << 0);								// peripheral enable

}

void I2C_Write(uint8_t address, uint8_t wRegister){
	I2C2->CR1 |= (1 << 0);								// PE ENABLE
	I2C2->CR1 |= (1 << 8);								// send START bit
	while(!(I2C2->SR1 & 0x0001));						// wait for start
	I2C2->DR = address;									// SLAVE ADDRESS
	while(!(I2C2->SR1 & 0x002));						// wait till address send
	while(!(I2C2->SR2 & 0x001));						// wait msl ( master or slave) cihaz master moda girdi mi
	while(!(I2C2->SR1 & 1 << 7));						// wait txe empty
	I2C2->DR = wRegister;
	while(!(I2C2->SR1 & 1 << 7));						// wait txe empty
	while(!(I2C2->SR1 & 0x04));							// wait btf empty byte transfer finished
	I2C2->CR1 = (1<<9);									// SEND STOP BIT
}

//void I2C_Read(void){
//
//
//
//}



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
