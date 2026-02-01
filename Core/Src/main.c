/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "epaper.h"
#include "bmp.h"
#include "./BME280/bme280.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t image_bw[EPD_W_BUFF_SIZE * EPD_H];
uint8_t text[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float temperature;
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char hum_string[50];
char temp_string[50];
char press_string[50];

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  dev.dev_id = BME280_I2C_ADDR_PRIM;
   dev.intf = BME280_I2C_INTF;
   dev.read = user_i2c_read;
   dev.write = user_i2c_write;
   dev.delay_ms = user_delay_ms;

   rslt = bme280_init(&dev);

   /* BME280 settings */
   dev.settings.osr_h = BME280_OVERSAMPLING_1X;
   dev.settings.osr_p = BME280_OVERSAMPLING_16X;
   dev.settings.osr_t = BME280_OVERSAMPLING_2X;
   dev.settings.filter = BME280_FILTER_COEFF_16;
   rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

  epd_init();

      epd_paint_newimage(image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);

    epd_paint_selectimage(image_bw);

    epd_paint_clear(EPD_COLOR_WHITE);
  	// epd_paint_showPicture((EPD_H - 200) / 2,(EPD_W - 64) / 2,200,64,gImage_5,EPD_COLOR_WHITE);

      epd_displayBW(image_bw);
  	// epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

      // HAL_Delay(2500);

  	epd_init_partial();

      epd_paint_selectimage(image_bw);
      epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showString(10, 0, (uint8_t *)&"1.54 Inch", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
  	epd_paint_showString(10, 23, (uint8_t *)&"Epaper Module", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
  	epd_paint_showString(10, 48, (uint8_t *)&"Designed By WeAct Studio", EPD_FONT_SIZE12x6, EPD_COLOR_BLACK);
  	epd_paint_showString(10, 60, (uint8_t *)&"with 200 x 200 resolution", EPD_FONT_SIZE12x6, EPD_COLOR_BLACK);
  	// epd_paint_showPicture((EPD_H - 200) / 2,130,200,64,gImage_5,EPD_COLOR_WHITE);

      epd_paint_showString(10,100,(uint8_t *)&"STM32F103C8T6 Example",EPD_FONT_SIZE16x8,EPD_COLOR_BLACK);

  	// sprintf((char *)&text, ">> Partial Mode");
  	// epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

  	epd_displayBW_partial(image_bw);

  	HAL_Delay(1000);

  	// for (uint32_t i = 123; i < 8 * 123; i += 123)
  	// {
  	// 	sprintf((char *)&text, ">> Num=%d     ", i);
  	// 	epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

  	// 	epd_displayBW_partial(image_bw);

  	// 	HAL_Delay(100);
  	// }

  	// sprintf((char *)&text, ">> Hello World.");
  	// epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
  	// epd_displayBW_partial(image_bw);

  	// HAL_Delay(1000);

  	// epd_update();

    // epd_update();
      // epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     /* Forced mode setting, switched to SLEEP mode after measurement */
	    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	    dev.delay_ms(40);
	    /*Get Data */
	    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	    if(rslt == BME280_OK)
	    {
	      temperature = comp_data.temperature / 100.0;
	      humidity = comp_data.humidity / 1024.0;
	      pressure = comp_data.pressure / 10000.0 * 0.75;

	      /*Display Data */
	      memset(hum_string, 0, sizeof(hum_string));
	      memset(temp_string, 0, sizeof(temp_string));
	      memset(press_string, 0, sizeof(press_string));

	      sprintf(hum_string, "Humidity %03.1f %%", humidity);
	      sprintf(temp_string, "Temperature %03.1f C", temperature);
	      sprintf(press_string, "Pressure %03.1f mm", pressure);

	      //char* text = "Loop\n";
	      //HAL_UART_Transmit(&huart1, (uint8_t*)hum_string, strlen(hum_string), 1000);
	      //HAL_UART_Transmit(&huart1, (uint8_t*)temp_string, strlen(temp_string), 1000);
	      //HAL_UART_Transmit(&huart1, (uint8_t*)press_string, strlen(press_string), 1000);
	      // printf(hum_string);
	      // printf(temp_string);
	      // printf(press_string);

        epd_paint_showString(10, 120, hum_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
        epd_paint_showString(10, 140, temp_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
        epd_paint_showString(10, 160, press_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

        
	    }
	    else
	    {
	    	printf("BME280 ERROR!\r\n");
        epd_paint_showString(10, 180, "BME280 ERROR!", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
	    }

      epd_displayBW_partial(image_bw);
      // epd_update();

	    HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  hi2c1.Init.Timing = 0x00000103;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|DC_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_Pin DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

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
