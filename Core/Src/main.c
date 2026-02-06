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
#define EINK_PARTIAL 0

// Настройки делителя напряжения
#define R1_POWER 10000 // Ом (верхний резистор)
#define R2_POWER 10000 // Ом (нижний резистор)

// Настройки делителя напряжения
#define R1_BATTERY 10000 // Ом (верхний резистор)
#define R2_BATTERY 10000 // Ом (нижний резистор)

// Диапазон напряжений для преобразования в проценты
#define BATTERY_VOLTAGE_MIN 3.0f // Минимальное напряжение в диапазоне (В)
#define BATTERY_VOLTAGE_MAX 4.1f // Максимальное напряжение в диапазоне (В)

// Диапазон напряжений для преобразования в проценты
#define POWER_VOLTAGE_MIN 3.5f // Минимальное напряжение в диапазоне (В)
#define POWER_VOLTAGE_MAX 5.5f // Максимальное напряжение в диапазоне (В)

// Диапазон напряжений для преобразования в проценты
#define RTC_VOLTAGE_MIN 2.0f // Минимальное напряжение в диапазоне (В)
#define RTC_VOLTAGE_MAX 3.0f // Максимальное напряжение в диапазоне (В)

#define ADC_MAX_VALUE 4095.0f
#define VOLTAGE_VREF 3.3f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t image_bw[EPD_W_BUFF_SIZE * EPD_H];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

typedef struct
{
  float temperature;
  float humidity;
  float pressure;
  int8_t connectionOk;
} bmeData_t;

typedef struct
{
  uint32_t batteryAdc;
  uint32_t rtcAdc;
  uint32_t powerAdc;
} adcData_t;

typedef struct
{
  float batteryVoltage;
  float rtcVoltage;
  float powerVoltage;
} voltageData_t;

typedef struct
{
  float batteryPercent;
  float rtcPercent;
  float powerPercent;
} percentData_t;

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK)
    return -1;
  if (HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK)
    return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len + 1);
  buf[0] = reg_addr;
  memcpy(buf + 1, data, len);

  if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t *)buf, len + 1, HAL_MAX_DELAY) != HAL_OK)
    return -1;

  free(buf);
  return 0;
}

int8_t Bme280Init()
{
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

  return rslt;
}

void EnterStopMode(void)
{
  // Отключаем SysTick, чтобы он не мешал
  HAL_SuspendTick();

  // Чистим флаги пробуждения
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  // Входим в STOP
  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  // ==== МК ПРОСНУЛСЯ ЗДЕСЬ ====

  // Восстанавливаем тактирование
  SystemClock_Config();

  // Возвращаем SysTick
  HAL_ResumeTick();

#if EINK_PARTIAL == 1
  epd_init_partial();
#else
  epd_init();
#endif
  epd_paint_selectimage(image_bw);
  epd_paint_clear(EPD_COLOR_WHITE);

#if EINK_PARTIAL == 1
  epd_displayBW_partial(image_bw);
#endif
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
}

adcData_t ReadADC(void)
{
  adcData_t adcData = {};
  HAL_ADC_Start(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adcData.powerAdc = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adcData.rtcAdc = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adcData.batteryAdc = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);

  return adcData;
}

RTC_DateTypeDef GetDate()
{
  RTC_DateTypeDef date;
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  return date;
}

RTC_TimeTypeDef GetTime()
{
  RTC_TimeTypeDef time;
  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  return time;
}

void DrawDateTime(RTC_DateTypeDef date, RTC_TimeTypeDef time)
{
  char time_string[50];
  char date_string[50];
  char week_string[50];

  sprintf(time_string, "Time: %02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);
  epd_paint_showString(1, 1, (uint8_t *)time_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

  sprintf(date_string, "Date: %02d/%02d/%02d", date.Date, date.Month, date.Year);
  epd_paint_showString(1, 20, (uint8_t *)date_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

  sprintf(week_string, "Day: %02d", date.WeekDay);
  epd_paint_showString(1, 40, (uint8_t *)week_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
}

void DrawBme280Data(bmeData_t bmeData)
{
  if (1 == bmeData.connectionOk)
  {
    char temp_string[50];
    char hum_string[50];
    char press_string[50];

    sprintf(temp_string, "Temperature %03.1f C", bmeData.temperature);
    sprintf(hum_string, "Humidity %03.1f %%", bmeData.humidity);
    sprintf(press_string, "Pressure %03.1f mm", bmeData.pressure);

    epd_paint_showString(1, 140, temp_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(1, 160, hum_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(1, 180, press_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  }
  else
  {
    epd_paint_showString(10, 180, "BME280: ERROR!", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  }
}

void SatelliteInit(float version)
{
  epd_init();
  epd_paint_newimage(image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
  epd_paint_selectimage(image_bw);
  epd_paint_clear(EPD_COLOR_WHITE);
  epd_displayBW(image_bw);

  epd_init_partial();
  epd_paint_selectimage(image_bw);
  epd_paint_clear(EPD_COLOR_WHITE);

  char version_string[50];
  memset(version_string, 0, sizeof(version_string));
  sprintf(version_string, "OpenSatellite v%0.1f", version);
  epd_paint_showString(1, 0, (uint8_t *)&version_string, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  epd_displayBW_partial(image_bw);

  int8_t bmeInitResult = Bme280Init();
  if (bmeInitResult == 0)
  {
    epd_paint_showString(1, 20, (uint8_t *)&"BME280: OK", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  }
  else
  {
    epd_paint_showString(1, 20, (uint8_t *)&"BME280: ERROR", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  }
  epd_displayBW_partial(image_bw);

  epd_paint_showString(1, 140, (uint8_t *)&"Designed by", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  epd_paint_showString(1, 160, (uint8_t *)&"Vadim 'syjoosy' Nikolaev [LCT]", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  epd_displayBW_partial(image_bw);

  epd_paint_showString(1, 180, (uint8_t *)&"Starting satellite!", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  epd_displayBW_partial(image_bw);

  epd_paint_clear(EPD_COLOR_WHITE);
  epd_displayBW_partial(image_bw);
}

bmeData_t ReadBme280()
{
  bmeData_t bmeData = {};
  /* Forced mode setting, switched to SLEEP mode after measurement */
  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
  dev.delay_ms(40);
  /*Get Data */
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
  if (rslt == BME280_OK)
  {
    bmeData.temperature = comp_data.temperature / 100.0;
    bmeData.humidity = comp_data.humidity / 1024.0;
    bmeData.pressure = comp_data.pressure / 10000.0 * 0.75;
    bmeData.connectionOk = 1;
    bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);
  }
  else
  {
    bmeData.connectionOk = 0;
  }

  return bmeData;
}

float ConvertToVoltage(uint32_t adcValue, uint32_t resistor1, uint32_t resistor2)
{
  // Напряжение на входе АЦП (после делителя)
  // V_adc = (adc_value / 4095) * V_ref
  float adcVoltage = (float)(adcValue / ADC_MAX_VALUE * VOLTAGE_VREF); // 3.3 В — опорное напряжение АЦП

  float voltage;
  if (0 == resistor1 || 0 == resistor2)
  {
    voltage = adcVoltage;
  }
  else
  {
    // Восстанавливаем исходное напряжение до делителя:
    // V_in = V_adc * (1 + R1/R2)
    float voltage = adcVoltage * (1.0f + resistor1 / resistor2);
  }

  return voltage;
}

// Функция: Чтение АЦП и преобразование в вольты (с учётом делителя)
voltageData_t GetVoltageData(adcData_t adcData)
{
  voltageData_t voltageData = {};

  voltageData.batteryVoltage = ConvertToVoltage(adcData.batteryAdc, R1_BATTERY, R2_BATTERY);
  voltageData.powerVoltage = ConvertToVoltage(adcData.powerAdc, R1_POWER, R1_POWER);
  voltageData.rtcVoltage = ConvertToVoltage(adcData.rtcAdc, 0, 0);

  return voltageData;
}

uint8_t ConvertToPercent(float voltage, float voltageMin, float voltageMax)
{
  // Ограничиваем напряжение диапазоном
  if (voltage < voltageMin)
  {
    voltage = voltageMin;
  }

  if (voltage > voltageMax)
  {
    voltage = voltageMax;
  }

  // Преобразуем в проценты: (v - min) / (max - min) * 100
  uint8_t percent = (voltage - voltageMin) / (voltageMax - voltageMin) * 100.0f;

  // Ограничиваем от 0 до 100
  if (percent < 0)
  {
    percent = 0;
  }

  if (percent > 100)
  {
    percent = 100;
  }

  return percent;
}

// Функция: Преобразование напряжения в проценты в заданном диапазоне
percentData_t GetPercentData(voltageData_t voltageData)
{
  percentData_t percentData = {};

  percentData.batteryPercent = ConvertToPercent(voltageData.batteryVoltage, BATTERY_VOLTAGE_MIN, BATTERY_VOLTAGE_MAX);
  percentData.powerPercent = ConvertToPercent(voltageData.powerVoltage, POWER_VOLTAGE_MIN, POWER_VOLTAGE_MAX);
  percentData.rtcPercent = ConvertToPercent(voltageData.rtcVoltage, RTC_VOLTAGE_MIN, RTC_VOLTAGE_MAX);

  return percentData;
}

void DrawPowerData(percentData_t percentData)
{
  char powerPercent[50];
  char batteryPercent[50];
  char rtcPercent[50];

  sprintf(powerPercent, "Power %i %%", percentData.powerPercent);
  sprintf(batteryPercent, "Battery %i %%", percentData.batteryPercent);
  sprintf(rtcPercent, "RTC %i %%", percentData.rtcPercent);

  epd_paint_showString(1, 80, powerPercent, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  epd_paint_showString(1, 100, batteryPercent, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
  epd_paint_showString(1, 120, rtcPercent, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
}

void SendDataToDisplay()
{
#if EINK_PARTIAL == 1
  epd_displayBW_partial(image_bw);
#else
  epd_displayBW(image_bw);
#endif

  epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
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
  float version = 0.1f;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  SatelliteInit(version);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    RTC_TimeTypeDef time = GetTime();
    RTC_DateTypeDef date = GetDate();
    DrawDateTime(date, time);

    adcData_t adcData = ReadADC();
    voltageData_t voltageData = GetVoltageData(adcData);
    percentData_t percentData = GetPercentData(voltageData);
    DrawPowerData(percentData);

    bmeData_t bmeData = ReadBme280();
    DrawBme280Data(bmeData);

    SendDataToDisplay();
    EnterStopMode();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    if (1 == bmeData.connectionOk)

      Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
   */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
   */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 29, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
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
  HAL_GPIO_WritePin(GPIOA, RST_Pin | DC_Pin | CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_Pin DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = RST_Pin | DC_Pin | CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_LED_Pin */
  GPIO_InitStruct.Pin = POWER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_LED_GPIO_Port, &GPIO_InitStruct);

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

#ifdef USE_FULL_ASSERT
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
