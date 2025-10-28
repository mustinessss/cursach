/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define MasSize 416
#define ADCIn MasSize + 200

const uint32_t Bark[MasSize] = {
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397,
    1736, 2075, 2400, 2700, 2964, 3179, 3340, 3439, 3472, 3439, 3340, 3179, 2964, 2700, 2400, 2075,
    1736, 1397, 1072, 772, 508, 293, 132, 33, 0, 33, 132, 293, 508, 772, 1072, 1397
};
int32_t buff[ADCIn];
uint16_t num = 0;
volatile int32_t corr = 0;
volatile uint16_t distance = 0;
uint8_t speakerNum = 1;
unsigned char a, buf[3];
unsigned char micdebug[4];
volatile int32_t debug1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
void autocorr(int32_t *mas1, const uint32_t *mas2);
int32_t calculateAverage(int32_t *mas1, int32_t length);
void switchSpeaker(void);
void processMeasurement(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Включаем первый динамик
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(1000);

  // Запускаем передачу сигнала
  HAL_TIM_Base_Start(&htim2);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Bark, MasSize, DAC_ALIGN_12B_R);

  // Запускаем прием данных с прерываниями
  HAL_ADC_Start_IT(&hadc1);

  // Тестовое сообщение
  HAL_UART_Transmit(&huart2, (uint8_t*)"System Started\n", 15, 100);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // debug1 = 0;
      if(num == ADCIn)
      {
          // Вычисление постоянной составляющей и взаимной корреляции
          int32_t mean = calculateAverage(buff, ADCIn);
          for(uint16_t c = 0; c < ADCIn; c++)
          {
              buff[c] = buff[c] - mean;
          }
          autocorr(buff, Bark);

          // Конец сегмента
          num = 0;

          // Пересылка расстояния по UART
          for(a = 0; a < 3; a++)
              buf[a] = '0';

          uint16_t temp_distance = distance;
          while(temp_distance >= 100)
          {
              buf[0]++;
              temp_distance = temp_distance - 100;
          }
          while(temp_distance >= 10)
          {
              buf[1]++;
              temp_distance = temp_distance - 10;
          }
          buf[2] += temp_distance;

          if (speakerNum == 0)
          {
              HAL_UART_Transmit(&huart2, (uint8_t*)"s\n", 2, 1000);
          }
          HAL_UART_Transmit(&huart2, (uint8_t*)buf, 3, 1000);
          HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
          HAL_Delay(30);

          // Переключаем светодиод для индикации следующего замера
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

          // Выбираем следующий динамик
          speakerNum++;
          if (speakerNum > 2)
          {
              speakerNum = 0;
          }

          // Включаем необходимый динамик
          switch(speakerNum)
          {
              case 0:
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
                  break;
              case 1:
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                  break;
              case 2:
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
                  break;
              default:
                  break;
          }

          // Повторно запускаем DMA и ADC
          HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Bark, MasSize, DAC_ALIGN_12B_R);
          HAL_ADC_Start_IT(&hadc1);
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // Можно добавить небольшую паузу для снижения нагрузки
      HAL_Delay(1);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 561;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 561;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Функция автокорреляции
void autocorr(int32_t *mas1, const uint32_t *mas2)
{
    volatile int32_t maxcorr = 0;
    volatile int32_t ans = 0;
    volatile int32_t sdvig = 0;

    for(uint16_t t = 0; t < ADCIn; t++)
    {
        volatile int32_t vnutr = 0;
        for(volatile uint16_t i = 0; i < MasSize; i++)
        {
            sdvig = i + t;
            if(sdvig < ADCIn)
            {
                vnutr = vnutr + (mas1[sdvig]) * ((int16_t)mas2[i] - 1736);
            }
        }
        corr = vnutr;
        if(corr > maxcorr)
        {
            maxcorr = corr;
            ans = t;
        }
    }
    distance = (ans * 33) / 16; // Коэффициент может потребовать корректировки
}

// Функция вычисления среднего значения
int32_t calculateAverage(int32_t *mas1, int32_t length)
{
    int64_t sum = 0; // Используем int64_t для избежания переполнения
    for (int i = 0; i < length; i++)
    {
        sum += mas1[i];
    }
    return (int32_t)(sum / length);
}

// Функция переключения динамиков
void switchSpeaker(void)
{
    // Выключаем все динамики
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    // Переключаем на следующий динамик
    speakerNum++;
    if (speakerNum > 2)
    {
        speakerNum = 0;
    }

    // Включаем выбранный динамик
    switch(speakerNum)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            break;
        default:
            break;
    }

    // Индикация переключения
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

// Функция обработки измерения
void processMeasurement(void)
{
    // Вычисляем постоянную составляющую
    int32_t mean = calculateAverage(buff, ADCIn);

    // Убираем постоянную составляющую
    for(uint16_t c = 0; c < ADCIn; c++)
    {
        buff[c] = buff[c] - mean;
    }

    // Вычисляем корреляцию
    autocorr(buff, Bark);

    // Отправляем расстояние по UART
    for(a = 0; a < 3; a++)
        buf[a] = '0';

    uint16_t temp_distance = distance;
    while(temp_distance >= 100)
    {
        buf[0]++;
        temp_distance = temp_distance - 100;
    }
    while(temp_distance >= 10)
    {
        buf[1]++;
        temp_distance = temp_distance - 10;
    }
    buf[2] += temp_distance;

    // Отправляем маркер начала для первого динамика
    if (speakerNum == 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"s\n", 2, 1000);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, 3, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);

    // Небольшая пауза перед следующим измерением
    HAL_Delay(30);
}

// Обработчик прерывания ADC (завершение преобразования)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // Заполняем массив значениями с микрофона
        if(num < ADCIn)
        {
            buff[num] = HAL_ADC_GetValue(&hadc1);
            num++;
        }
        else
        {
            // Буфер заполнен - выключаем динамики и останавливаем ADC
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
            HAL_ADC_Stop_IT(&hadc1);
        }
    }
}

// Обработчик Half Transfer (опционально)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Можно использовать для обработки первой половины данных
    // пока заполняется вторая половина
}

// Обработчик прерывания таймера (если будете использовать)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        // Код для периодических измерений по таймеру
    }
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
