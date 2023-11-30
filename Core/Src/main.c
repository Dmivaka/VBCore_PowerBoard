/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "my_helpers.h"
#include "communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  USER,
  WARNING,
  ALARM
} buzzer_mutex_priorities ; 

typedef struct 
{
  float raw;
  float LPF;
  float HPF;
  uint8_t charged;
  uint8_t attached;
  
  uint32_t PG_pin;
  GPIO_TypeDef * PG_port;
} input_src_stat;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define auto_prime_selection
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint64_t TIM7_ITs = 0; // counter of microseconds timesource ITs

#define ADC_buf_size 6
uint32_t ADC1_buf[ADC_buf_size] = {0};

input_src_stat VIN1;
input_src_stat VIN2 = { .PG_pin = SUPP_2_PG_Pin, .PG_port = SUPP_2_PG_GPIO_Port};
input_src_stat VIN3 = { .PG_pin = SUPP_3_PG_Pin, .PG_port = SUPP_3_PG_GPIO_Port};

input_src_stat CHRG;

input_src_stat *prime_VIN;
uint8_t prime = 0; // selected power source variable. RW

input_src_stat *prime_VOUT = NULL;

/********** User accessible variables **********/
const float uvlo_level = 30.0f; // battery discharged voltage level, Volts
const float uvlo_hyst = 1.5f; // battery discharged hysteresis, Volts

const float src_charged_level = 40.0f; // battery charged voltage level, Volts

const uint32_t bus_start_timeout = 30000u; // timeout for bus output reaching PowerGood status, microseconds
const uint8_t emergency_start_threshold = 3u; // number of attempts to start the bus before an emergency shutdown

uint8_t default_prime = 0; // variable to store the prefered prime power source. 0 for input "2", 1 for "3"

uint8_t emergency_stat = 0; // flag indicating the emergency button is pressed. RO
uint8_t pc_enable = 1; // PC power bus control. RW
uint8_t bus_enable = 1; // main power bus control. RW

uint8_t pc_stat = 1; // PC power bus status. RO
uint8_t bus_stat = 1; // main power bus status. RO
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize s=none
uint64_t micros()
#elif defined ( __GNUC__ ) /* GNU Compiler */
uint64_t __attribute__((optimize("O0"))) micros()
#endif
{ 
  return (uint64_t)(__HAL_TIM_GET_COUNTER(&htim7) + 50000u * TIM7_ITs);
}


#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize s=none
void micros_delay( uint64_t delay )
#elif defined ( __GNUC__ ) /* GNU Compiler */
void __attribute__((optimize("O0"))) micros_delay( uint64_t delay )
#endif
{
  uint64_t timestamp = micros();
  while( micros() < timestamp + delay );
}
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_FDCAN1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(300); // prevents MOSFETs torture if board is forced to restart frequently
  
  HAL_TIM_Base_Start_IT(&htim7); // enable microseconds timesource
  
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // apply factory ADC calibration settings
  HAL_ADC_Start_DMA(&hadc1, ADC1_buf, ADC_buf_size); // enable ADC and bind DMA to the output buffer
  
  // FDCAN configuration
  FDCAN_FilterTypeDef sFilterConfig;  
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x1FFFFFFF;
  
  // Configure CAN frames filtering 
  if( HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK ){ Error_Handler(); }  
  
  // Configure global filter: Filter all remote frames with STD and EXT ID. Reject non matching frames with STD ID and EXT ID
  if( HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK ){ Error_Handler(); }
  
  // Activate Rx FIFO 0 new message notification
  //if( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK ){ Error_Handler(); }        
  
  // Configure and enable Tx Delay Compensation, required for BRS mode.
  if( HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK){ Error_Handler(); }
  if( HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK){ Error_Handler(); }
  
  if( HAL_FDCAN_ConfigRxFifoOverwrite( &hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK){ Error_Handler(); }
  
    // Start FDCAN periphery
  if( HAL_FDCAN_Start(&hfdcan1) != HAL_OK ){ Error_Handler(); }

  HAL_TIM_Base_Start_IT(&htim6); // processes ADC values in TIM6 interrupt
  
  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim16); // processes power control logic in TIM16 interrupt
  HAL_Delay(10);
  
  uavcan_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uavcan_spin();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 24;
  hfdcan1.Init.NominalTimeSeg1 = 55;
  hfdcan1.Init.NominalTimeSeg2 = 24;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
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
  sConfigOC.Pulse = 9;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 159;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 159;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 159;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(S3_red_GPIO_Port, S3_red_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S2_red_GPIO_Port, S2_red_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S1_red_GPIO_Port, S1_red_Pin);

  /**/
  LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(PC7_GPIO_Port, PC7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(PC8_GPIO_Port, PC8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(PC9_GPIO_Port, PC9_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S3_grn_GPIO_Port, S3_grn_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S2_grn_GPIO_Port, S2_grn_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S1_grn_GPIO_Port, S1_grn_Pin);

  /**/
  GPIO_InitStruct.Pin = S3_red_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S3_red_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S2_red_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S2_red_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S1_red_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S1_red_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OE_CTL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OE_CTL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DEMUX_OE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DEMUX_OE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SW3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DEMUX_S0_CTL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DEMUX_S0_CTL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PC7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PC7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PC8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PC8_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PC9_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PC9_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUS_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUS_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SUPP_2_PG_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SUPP_2_PG_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SUPP_3_PG_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SUPP_3_PG_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUS_CTl_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUS_CTl_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUS_PG_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BUS_PG_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S3_grn_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S3_grn_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S2_grn_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S2_grn_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S1_grn_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S1_grn_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t check_battery_input( input_src_stat * VIN )
{
  if( VIN->HPF < -5.0f )
  {
    micros_delay( 1000 );
    if( ADC1_buf[1] < 2060 ) // exclude transients. the source is disconnected only if current ~0A
    {
      VIN->attached = 0;
      VIN->LPF = 0.0f;
    }
  }
  else if( VIN->HPF > 5.0f )
  {
    VIN->attached = 1;
    VIN->LPF = VIN->raw;
  }

  if( VIN->LPF < uvlo_level )
  {
    VIN->charged = 0;
  }
  else if( VIN->LPF > uvlo_level + uvlo_hyst )
  {
    VIN->charged = 1;
  }
    
  return 0;
}

void check_buttons(void)
{
  if( !LL_GPIO_IsInputPinSet( SW2_GPIO_Port, SW2_Pin) )
  {
    if( !LL_GPIO_IsInputPinSet( SW3_GPIO_Port, SW3_Pin) )
    {
      // user is noodle
    }
    else
    {
      prime = 0;
    }
  }
  else
  {
    if( !LL_GPIO_IsInputPinSet( SW3_GPIO_Port, SW3_Pin) )
    {
      prime = 1;
    }
    else
    {
      // do nothing
    }
  }  
}


uint8_t buzzer_mutex = 0;
uint64_t buzzer_pulse_stamp = 0;
uint64_t buzzer_period_stamp = 0;

void power_control(void)
{
  static uint8_t start_fail_cnt = 0; // counter of failed start-up sequences
  
  // emergency shutdown if output did not start too many times
  if( start_fail_cnt >= emergency_start_threshold )
  {
    LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
    LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus    
    
    Error_Handler();
  }
  
  check_battery_input( &VIN2 );
  check_battery_input( &VIN3 );
  
  // Check buttons and select prime
  check_buttons();
  
#ifdef auto_prime_selection
  if( VIN2.charged && VIN3.charged )
  {
    prime = default_prime;
  }
#endif
  
  if( CHRG.raw > 3.0f ) 
  {
    // we're in charging mode
    LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus    

    if( prime_VOUT == NULL )
    {
      LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // disable input channels
      
      micros_delay( 50000 );

      if( ( VIN2.raw > 10.0f ) && ( VIN2.raw < src_charged_level ) )
      {
        prime_VOUT = &VIN2;
        
        LL_GPIO_SetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // set channels output control
        
        micros_delay( 50000 );
      }
      else if( ( VIN3.raw > 10.0f ) && ( VIN3.raw < src_charged_level ) )
      {
        prime_VOUT = &VIN3;

        LL_GPIO_ResetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // set channels output control

        micros_delay( 50000 );
      }
      else
      {
        return ;
      }
    }
    
    if( ADC1_buf[1] < 1890 )
    {
      // charging overcurrent event
      Error_Handler();
    }
    
    if( ADC1_buf[1] > 2030 )
    {
      // battery takes no current = battery is charged
      prime_VOUT = NULL;
    }
  }
  else
  {
    prime_VOUT = NULL;
    
    // select suitable power source based on availability and user request
    if( prime == 0 )
    {
      if( VIN2.charged )
      {
        prime_VIN = &VIN2;
      }
      else if( VIN3.charged )
      {
        prime = 1; // switch ptime channel to VIN3
        prime_VIN = &VIN3;
      }
      else
      {
        prime_VIN = NULL;
      }
    }
    else
    {
      if( VIN3.charged )
      {
        prime_VIN = &VIN3;
      }
      else if( VIN2.charged )
      {
        prime = 0; // switch ptime channel to VIN2
        prime_VIN = &VIN2;
      }
      else
      {
        prime_VIN = NULL;
      }    
    }

    // switch power rails
    if( prime_VIN != NULL )
    {
      uint64_t timestamp = 0;
      
      if( pc_enable )
      {
        // select desired channel on multiplexer
        if( !prime )
        {
          LL_GPIO_SetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        }
        else
        {
          LL_GPIO_ResetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        }
        
        LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // set channels output control
        
        timestamp = micros();
        while( LL_GPIO_IsInputPinSet( prime_VIN->PG_port, prime_VIN->PG_pin) ) // wait until PG pin goes low ( PG = OK )
        {
          if( micros() > timestamp + 10000 ) // the bus didnt reach PG status in 10 ms = abort operation
          {
            LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
            LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus
            
            micros_delay( 1000000 ); // wait for 1s before trying to enable bus again to prevent MOSFET damage
            
            start_fail_cnt++;

            return ;
          }
        }
      }
      else
      {
        LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
        LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus
        return ;
      }
      
      if( bus_enable )
      {
        if( !LL_GPIO_IsOutputPinSet(BUS_CTl_GPIO_Port, BUS_CTl_Pin) )
        {
          LL_GPIO_SetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // enable power bus
          micros_delay(2);
        }
        
        if( !LL_GPIO_IsInputPinSet( BUS_EN_GPIO_Port, BUS_EN_Pin) && LL_GPIO_IsOutputPinSet(BUS_CTl_GPIO_Port, BUS_CTl_Pin) )
        {
          emergency_stat = 1;
          LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus
        }
        else
        {
          emergency_stat = 0;
          
          timestamp = micros();
          while( LL_GPIO_IsInputPinSet( BUS_PG_GPIO_Port, BUS_PG_Pin) ) // wait until PG pin goes low ( PG = OK )
          {
            if( micros() > timestamp + bus_start_timeout ) // the bus didnt reach PG status in allotted time = abort operation
            {
              LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
              LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus
              
              micros_delay( 1000000 ); // wait for 1s before trying to enable bus again to prevent MOSFET damage
              
              start_fail_cnt++;
              
              return ;
            }
          }
        }   
      }
      else
      {
        LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus
        return ;
      }
    }
    else
    {
      // all dead, disable output
      
      LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
      
      if( LL_GPIO_IsOutputPinSet(BUS_CTl_GPIO_Port, BUS_CTl_Pin) )
      {
        LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus     
        micros_delay(200000);
      }    
      
      if( buzzer_mutex < ALARM )
      {
        buzzer_mutex = ALARM;
        buzzer_pulse_stamp = micros() + 1000000u;
        buzzer_period_stamp = micros() + 2000000u;
      }    
    } 
  }
}

void indication(void)
{
  if( prime_VIN->LPF < uvlo_level + uvlo_hyst )
  {
    if( buzzer_mutex < WARNING )
    {
      buzzer_mutex = WARNING;
      buzzer_pulse_stamp = micros() + 300000u;
      buzzer_period_stamp = micros() + 1800000u;
    }
  }
  
  if( buzzer_mutex )
  {
    if( micros() < buzzer_pulse_stamp )
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    }
    else if( micros() < buzzer_period_stamp )
    {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    }
    else
    {
      buzzer_mutex = 0;
    }
  }

  if( VIN2.attached ) { LL_GPIO_SetOutputPin(S2_red_GPIO_Port, S2_red_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S2_red_GPIO_Port, S2_red_Pin); }
  
  if( VIN2.charged && pc_stat && LL_GPIO_IsOutputPinSet(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin) )  { LL_GPIO_SetOutputPin(S2_grn_GPIO_Port, S2_grn_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S2_grn_GPIO_Port, S2_grn_Pin); }    
  
  if( VIN3.attached ) { LL_GPIO_SetOutputPin(S3_red_GPIO_Port, S3_red_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S3_red_GPIO_Port, S3_red_Pin); }
  
  if( VIN3.charged && pc_stat && !LL_GPIO_IsOutputPinSet(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin) )  { LL_GPIO_SetOutputPin(S3_grn_GPIO_Port, S3_grn_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S3_grn_GPIO_Port, S3_grn_Pin); }     
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM6 )
  {
    VIN2.raw = 16.0f*3.3f*(float)ADC1_buf[3] / ( 4096.0f );
    VIN2.LPF = VIN2.LPF - (0.005f * (VIN2.LPF - VIN2.raw));
    VIN2.HPF = VIN2.raw - VIN2.LPF;
    
    VIN3.raw = 16.0f*3.3f*(float)ADC1_buf[4] / ( 4096.0f );
    VIN3.LPF = VIN3.LPF - (0.005f * (VIN3.LPF - VIN3.raw));
    VIN3.HPF = VIN3.raw - VIN3.LPF;    
    
    CHRG.raw = 16.0f*3.3f*(float)ADC1_buf[0] / ( 4096.0f );
    CHRG.LPF = CHRG.LPF - (0.005f * (CHRG.LPF - CHRG.raw));
    CHRG.HPF = CHRG.raw - CHRG.LPF;    
    
    return ;
  }
  else if( htim->Instance == TIM7 )
  {
    TIM7_ITs++;
  }
  else if( htim->Instance == TIM16 )
  {
    power_control();
    
    pc_stat = !LL_GPIO_IsOutputPinSet(OE_CTL_GPIO_Port, OE_CTL_Pin);
    bus_stat = LL_GPIO_IsOutputPinSet(BUS_CTl_GPIO_Port, BUS_CTl_Pin);
    
    indication();
  }
}

uint8_t FDCAN_PushFrame( uint32_t id, uint8_t data_size, uint8_t *data )
{
  FDCAN_TxHeaderTypeDef TxHeader;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0)
  {
    // Add message to Tx FIFO 
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = LengthCoder( data_size );
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
    {
      Error_Handler();
    }
    
    return 0;
  }
  
  return 1;
}

uint8_t FDCAN_ProcessFifo(void)
{
  while( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0 )
  {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];

    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){ Error_Handler(); }
    
    FDCAN_RxProcessing( RxHeader.Identifier, LengthDecoder(RxHeader.DataLength), RxData);
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
  LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
  LL_GPIO_ResetOutputPin(BUS_CTl_GPIO_Port, BUS_CTl_Pin); // disable power bus     
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
