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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueueData */
osMessageQueueId_t myQueueDataHandle;
const osMessageQueueAttr_t myQueueData_attributes = {
  .name = "myQueueData"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void Callback01(void *argument);

/* USER CODE BEGIN PFP */


void joysticTransmition(int k, uint8_t* txBuf)
{
	uint8_t rxBuf[1];
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	for( int i = 0; i < k; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, txBuf+i, rxBuf, 1, 200);
		for (int i = 0; i<100;i++);
	}
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void joysticTrRs(int k, uint8_t* txBuf, uint8_t* rxBuf)
{
	//uint8_t rxBuf[1];
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	for( int i = 0; i < k; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, txBuf+i, rxBuf+i, 1, 200);
		for (int i = 0; i<100;i++);
	}
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

#define FLASH_TYPEPROGRAMDATA_BYTE         (0x00U)  // ???????? 1 ????
#define FLASH_TYPEPROGRAMDATA_HALFWORD     (0x01U)  // ???????? 2 ?????
#define FLASH_TYPEPROGRAMDATA_WORD         (0x02U)  // ???????? 4 ?????
#define FLASH_TYPEPROGRAMDATA_FASTBYTE     (0x04U)  // ?????? ???????? 1 ????
#define FLASH_TYPEPROGRAMDATA_FASTHALFWORD (0x08U)  // ?????? ???????? 2 ?????
#define FLASH_TYPEPROGRAMDATA_FASTWORD     (0x10U)  // ?????? ???????? 4 ?????
#define FLASH_TYPEERASEDATA_BYTE         (0x00U)  // ??????? ????
#define FLASH_TYPEERASEDATA_HALFWORD     (0x01U)  // ??????? 2 ?????
#define FLASH_TYPEERASEDATA_WORD         (0x02U)  // ??????? 4 ?????


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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* creation of myQueueData */
  myQueueDataHandle = osMessageQueueNew (16, sizeof(uint16_t), &myQueueData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

////encoders filter data
uint16_t adc_val_f[2] = {0,0};
unsigned int LPIN0, LPIN1;
unsigned long LPACC0, LPACC1;
const int K = 70;
uint8_t flg = 0;

///////////////////////////////////////////////////////////////
////encoders data
uint16_t adc_val[2] = {0,0};

uint16_t adc_val_max0 = 3900;
uint16_t adc_val_min0 = 2600;//2480;

uint16_t adc_val_max1 = 3000;
uint16_t adc_val_min1 = 2000;

//motors & servo config
uint16_t M0Speed = 299;
uint16_t M1Speed = 299;
uint16_t M0MaxSpeed = 2999;
uint16_t M1MaxSpeed = 2999;
uint16_t M0MinSpeed = 299;
uint16_t M1MinSpeed = 299;
///////////////////////////////////////////////////////////////


//servos data
uint16_t S1 = 50;
uint16_t S2 = 50;
uint16_t S3 = 50;
uint8_t dS = 1;

unsigned short int timer = 0;
unsigned short int prev_timer = 0;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* USER CODE BEGIN StartDefaultTask */
	/* USER CODE BEGIN StartDefaultTask */

	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 2);
	
	////Servo start pos
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Servo_1 Timer Start
	TIM3->CCR1 = S1;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Servo_2 Timer Start
	TIM3->CCR2 = S2;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Servo_3 Timer Start
	TIM3->CCR3 = S3;
	
  uint8_t t = 0;
	
//	uint8_t tx3Buf[41]={0x01, 0x43, 0x00, 0x01, 0x00,
//										  0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00,
//										  0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF,
//										  0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00,
//										  0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A}; 
	
///////////////////////////////////////////////////////
///	configuration into analog mode	
	uint8_t tx4Buf[5] = {0x01, 0x43, 0x00, 0x01, 0x00};	// FF 41 5A FF FF
	uint8_t tx5Buf[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};	//FF	F3	5A	00	00	00	00	00	00
	uint8_t tx6Buf[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF}; //FF	F3	5A	00	01	FF	FF	FF	FF
	uint8_t tx7Buf[9] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};	//FF	F3	5A	00	00	00	00	00	5A
	uint8_t tx8Buf[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};	//FF	F3	5A	00	00	00	00	00	00										
											
///////////////////////////////////////////////////////
	uint8_t bufSize = 21;

	uint8_t txAnalogBuf[21] = {0x01, 0x42, 0x00, 0x00, 0x00,
														 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rxBuf[21];
	uint8_t txVibroBuf[21] = {0x01, 0x42, 0x00, 0xFF, 0xFF,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	uint8_t txBuf[5] = {0x01, 0x42, 0x00, 0xFF, 0xFF};

////Motor speed	
	TIM2->ARR = M0Speed;
	TIM4->ARR = M1Speed;


	
//  /* Infinite loop */
///	configuration into analog mode										
	joysticTransmition( 5, tx4Buf);
	osDelay(20);
	joysticTransmition( 9, tx5Buf);
	osDelay(20);
	joysticTransmition( 9, tx6Buf);
	osDelay(20);
	joysticTransmition( 9, tx7Buf);
	osDelay(20);
	joysticTransmition( 9, tx8Buf);
	osDelay(20);
	float rotate_time = 10;
	

  for(;;)
  {
		///config into analog mode
		joysticTrRs( 21, txAnalogBuf, rxBuf);

		// START_MOOVING motors

			////x button
			if((rxBuf[4] & 0x40 ) == 0){		
					if( (prev_timer > timer) || ( timer - prev_timer > 50))
					{
						if (dS < 9){
							dS = dS + 2;
						}
						else{dS = 1;}
						prev_timer = timer;
					}
				}
				else{
				}
		
//////////////////////////////////////////////////////////////////////////////////		
			
				
				if(((rxBuf[4] & 0x04) == 0) || ((rxBuf[4] & 0x08) == 0)){ //R1||L1 buttons
					if((rxBuf[4] & 0x08) == 0){ //R1 button
						if((rxBuf[6] != 0x7F) && (rxBuf[6] <= 0xFF)){   //if stick is used
							if(rxBuf[6] < 0x7F){ //if stick up
								if(adc_val_f[1] > adc_val_min1){
									HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
									HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
								}
								else{
									//vibration
									joysticTrRs( 21, txAnalogBuf, rxBuf);
									joysticTrRs( 21, txVibroBuf, rxBuf);
									joysticTrRs( 21, txVibroBuf, rxBuf);
									HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
								}
							}
							else{//if stick down
							// ex-X button
								if(adc_val_f[1] < adc_val_max1){
									HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
									HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
								}
								else{
									//vibration
									joysticTrRs( 21, txAnalogBuf, rxBuf);
									joysticTrRs( 21, txVibroBuf, rxBuf);
									joysticTrRs( 21, txVibroBuf, rxBuf);
									HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
								}	
							}
						}
						else{ 
								HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); ////needed?????
							}
					}
					else{ //putting data into the queue
						flg = 1;
						osMessageQueuePut(myQueue01Handle, &rxBuf[7], 0, 100); // delay ???
					}
				}
				else{ //buttons R1 ||R2 unpressed			
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); ////needed?????
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); ////needed?????
				}				
//////////////////////////////////////////////////////////////////////////////////					
			
		///////////////Servo1
		if(((rxBuf[3] & 0x80) == 0) || ((rxBuf[3] & 0x20) == 0)){
			if ((rxBuf[3] & 0x80) == 0){ // LEFT button
				S1 = S1 - dS;
			}
			else{ // RIGHT button
				S1 = S1 + dS;
			}
			if ( S1 > 250){S1 = 250;}
			if ( S1 < 49){S1 = 50;}
			TIM3->CCR1 = S1;
			osDelay(50);
		}
		else{
			// NO UP||DOWN buttons
		}
		
		///////////////Servo2
		if(((rxBuf[3] & 0x10) == 0) || ((rxBuf[3] & 0x40) == 0)){
			if ((rxBuf[3] & 0x10) == 0){ // DOWN button
				S2 = S2 - dS;
			}
			else{ // UP button
				S2 = S2 + dS;
			}
			if ( S2 > 250){S2 = 250;}
			if ( S2 < 49){S2 = 50;}
			TIM3->CCR2 = S2;
			osDelay(50);
		}
		else{
			// NO UP||DOWN buttons
		}
		
		///////////////Servo3
		if(((rxBuf[4] & 0x01) == 0) || ((rxBuf[4] & 0x02) == 0)){
			if ((rxBuf[4] & 0x01) == 0){ // L2 button
				S3 = S3 - dS;
			}
			else{ // R2 button
				S3 = S3 + dS;
			}
			if ( S3 > 250){S3 = 250;}
			if ( S3 < 49){S3 = 50;}
			TIM3->CCR3 = S3;
			osDelay(50);
		}
		else{
			// NO R2||L2 buttons
		}
				
		///	configuration into analog mode every x seconds
		timer++;
		if(timer >= 300)
		{
			///	configuration into analog mode										
			joysticTransmition( 5, tx4Buf);
			osDelay(20);
			joysticTransmition( 9, tx5Buf);
			osDelay(20);
			joysticTransmition( 9, tx6Buf);
			osDelay(20);
			joysticTransmition( 9, tx7Buf);
			osDelay(20);
			joysticTransmition( 9, tx8Buf);
			osDelay(20);
			timer = 0;
		}
		
		////encode data filter
		LPIN0 = (double)(adc_val[0]);
		LPIN1 = (double)(adc_val[1]);
		adc_val_f[0] = (int)( (unsigned long)LPACC0/K);
		adc_val_f[1] = (int)( (unsigned long)LPACC1/K);
		LPACC0 = LPACC0 + LPIN0 - adc_val_f[0];
		LPACC1 = LPACC1 + LPIN1 - adc_val_f[1];
		
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t data[5] = {0, 0, 0, 0, 0};
	uint8_t txAnalogBuf[21] = {0x01, 0x42, 0x00, 0x00, 0x00,
														 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rxBuf[21];
	uint8_t txVibroBuf[21] = {0x01, 0x42, 0x00, 0xFF, 0xFF,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	uint8_t txBuf[5] = {0x01, 0x42, 0x00, 0xFF, 0xFF};

  /* Infinite loop */
  for(;;)
	{				
		if( flg == 1){
			osMessageQueueGet(myQueue01Handle, data, 0, 100);
			if((data[0] != 0x80) && (data[0] <= 0xFF)){   //if stick is used
				flg = 0;
				if(data[0] > 0x80){ //if stick is on right
					if(adc_val_f[0] > adc_val_min0){
						HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
					}
					else{
						//vibration
						joysticTrRs( 21, txAnalogBuf, rxBuf);
						joysticTrRs( 21, txVibroBuf, rxBuf);
						joysticTrRs( 21, txVibroBuf, rxBuf);
						HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
					}
				}
				else{//if stick is on left
				// ex-X button
					if(adc_val_f[0] < adc_val_max0){
						HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
					}
					else{
						//vibration
						joysticTrRs( 21, txAnalogBuf, rxBuf);
						joysticTrRs( 21, txVibroBuf, rxBuf);
						joysticTrRs( 21, txVibroBuf, rxBuf);
						HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
					}
				}
			}
			else{ 
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			}
			
		}
  }
  /* USER CODE END StartTask02 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
