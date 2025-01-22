/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RTC_ADDR 0x68 << 1

// Ðinh nghia các chân GPIO cho LED 7 thanh
#define SEG_A GPIO_PIN_0
#define SEG_B GPIO_PIN_1
#define SEG_C GPIO_PIN_2
#define SEG_D GPIO_PIN_3
#define SEG_E GPIO_PIN_4
#define SEG_F GPIO_PIN_5
#define SEG_G GPIO_PIN_6

#define SEG_PORT GPIOA

#define LED1_PIN GPIO_PIN_8
#define LED2_PIN GPIO_PIN_9
#define LED_PORT GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Bang ma hoa so tu 0-9 (a,b,c,d,e,f,g)
const uint8_t SEGMENT_NUMBERS[10] = {
    0xC0, // 0: a,b,c,d,e,f
    0xF9, // 1: b,c
    0xA4, // 2: a,b,d,e,g
    0xB0, // 3: a,b,c,d,g
    0x99, // 4: b,c,f,g
    0x92, // 5: a,c,d,f,g
    0x82, // 6: a,c,d,e,f,g
    0xF8, // 7: a,b,c
    0x80, // 8: a,b,c,d,e,f,g
    0x90  // 9: a,b,c,d,f,g
};

// Hàm hien thi so trên LED 7 thanh
void displayNumber(uint8_t number, GPIO_TypeDef *port, uint16_t pin) {
    // Chan LED (bat Common Anode)
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN | LED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_RESET);

    // Hien thi so
    uint8_t segments = SEGMENT_NUMBERS[number];
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(SEG_PORT, (SEG_A << i), (segments & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}


//Tao mot struct de luu cac gia tri thoi gian
typedef struct{
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} DateTime;

uint8_t Decimal2BCD(uint8_t num){
	return (num/10)<<4 | (num%10);
}

uint8_t BCD2Decimal(uint8_t num){
	return (num>>4)*10 + (num&0x0F);
}

void RTC_WriteTime(DateTime *dt){
	uint8_t buf[8];
	buf[0] = 0;
	buf[1] = Decimal2BCD(dt->second);
	buf[2] = Decimal2BCD(dt->minute);
	buf[3] = Decimal2BCD(dt->hour);
	buf[4] = Decimal2BCD(dt->day);
	buf[5] = Decimal2BCD(dt->date);
	buf[6] = Decimal2BCD(dt->month);
	buf[7] = Decimal2BCD(dt->year);
	
	HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR, buf, 8, 100);
}


void RTC_ReadTime(DateTime *dt){
	uint8_t buff[7];
	uint8_t addr_reg = 0;
	HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR, &addr_reg, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR, buff, 7, 100);
	dt->second = BCD2Decimal(buff[0]);
	dt->minute = BCD2Decimal(buff[1]);
	dt->hour = BCD2Decimal(buff[2]);
	dt->day = BCD2Decimal(buff[3]);
	dt->date = BCD2Decimal(buff[4]);
	dt->month = BCD2Decimal(buff[5]);
	dt->year = BCD2Decimal(buff[6]);
	
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	

	DateTime datetime;
	/*
	datetime.second = 20;
	datetime.minute = 15;
	datetime.hour = 23;
	datetime.day = 5;
	datetime.date = 26;
	datetime.month = 11;
	datetime.year = 24;
	RTC_WriteTime(&datetime);
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		RTC_ReadTime(&datetime);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Hien thi so trên LED thu nhat
        displayNumber(datetime.minute / 10, LED_PORT, LED1_PIN);
        HAL_Delay(5);

        // Hien thi so tren LED thu hai
        displayNumber(datetime.minute % 10, LED_PORT, LED2_PIN);
        HAL_Delay(5);
		 
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
