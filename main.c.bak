/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef h;
CAN_RxHeaderTypeDef RxHeader;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POWER_LED_ERROR HAL_GPIO_WritePin (GPIOB, GPIO_PIN_7,0);HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6,1);
#define POWER_LED_OK HAL_GPIO_WritePin (GPIOB, GPIO_PIN_7,1);HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6,0);
#define POWER_LED_OFF HAL_GPIO_WritePin (GPIOB, GPIO_PIN_7,0);HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6,0);
#define POWER_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
#define POWER_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
#define CAN_ON HAL_GPIO_WritePin (GPIOC, GPIO_PIN_2, 0);
#define CAN_OFF HAL_GPIO_WritePin (GPIOC, GPIO_PIN_2, 1);
#define CAN_LED_OK HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5,1); HAL_GPIO_WritePin (GPIOD, GPIO_PIN_2,0);
#define CAN_LED_ERROR HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5,0); HAL_GPIO_WritePin (GPIOD, GPIO_PIN_2,1);
#define CAN_LED_OFF HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5,0); HAL_GPIO_WritePin (GPIOD, GPIO_PIN_2,0); 




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for smrtCAN */
osThreadId_t smrtCANHandle;
uint32_t smrtCANBuffer[ 128 ];
osStaticThreadDef_t smrtCANControlBlock;
const osThreadAttr_t smrtCAN_attributes = {
  .name = "smrtCAN",
  .cb_mem = &smrtCANControlBlock,
  .cb_size = sizeof(smrtCANControlBlock),
  .stack_mem = &smrtCANBuffer[0],
  .stack_size = sizeof(smrtCANBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ScanADC */
osThreadId_t ScanADCHandle;
const osThreadAttr_t ScanADC_attributes = {
  .name = "ScanADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DeviceManagment */
osThreadId_t DeviceManagmentHandle;
const osThreadAttr_t DeviceManagment_attributes = {
  .name = "DeviceManagment",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
void EmergencySparkLED (uint8_t NumberOfFlashes);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallBack(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallBack(CAN_HandleTypeDef *hcan);
void CanGetMessage(uint8_t *RxData);
void CanSendMessage (void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void *argument);
void TaskCanBus(void *argument);
void TaskADC(void *argument);
void TaskSwitchMode(void *argument);

/* USER CODE BEGIN PFP */
void vTaskDelayUntil(TickType_t* const pxPreviousWakeTime, const TickType_t xTimeIncrement);
void PowerLedFlash (uint8_t NumberOfFlash);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//xSemaphoreHandle ADC1_U_Val;
//xSemaphoreHandle ADC2_I_Val;
uint16_t ADC1_U_Val, ADC2_I_Val;
float I_Value,U_Value;
int ADC1_average, ADC2_average;
uint16_t ADC1_mass_value[10];
uint16_t ADC2_mass_value[10];
volatile uint32_t ADC1ConvertedValue[3];
//uint32_t ADC1ConvertedValue[3];
uint32_t ADC2ConvertedValue[1] = {0};
uint8_t TxData [8];
uint8_t RxData [8];
uint8_t RxIDCurrentFrame;
uint8_t flagTIM6, counterFlash;

struct Status{
	uint8_t ErrorCAN;
	uint8_t ErrorOverVoltage;
	uint8_t ErrorOverCurrent;
	uint8_t PowerSwitch;
};
struct Status DeviceStatus;


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
  MX_ADC1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC1ConvertedValue,3);
	
	POWER_ON;
	POWER_LED_OK;
	CAN_ON;
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING|CAN_IT_RX_FIFO0_MSG_PENDING| CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_BUSOFF|CAN_IT_LAST_ERROR_CODE);
	
	/*-------------Настройка-Стартового-Состояния-Устройства--------------*/
	
	DeviceStatus.ErrorCAN = DeviceErrorCanNoConnect;
	DeviceStatus.ErrorOverCurrent = DeviceErrorCurrentNormal;
	DeviceStatus.ErrorOverVoltage = DeviceErrorVoltageNormal;
	DeviceStatus.PowerSwitch = DevicePowerOn;
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of smrtCAN */
  smrtCANHandle = osThreadNew(TaskCanBus, NULL, &smrtCAN_attributes);

  /* creation of ScanADC */
  ScanADCHandle = osThreadNew(TaskADC, NULL, &ScanADC_attributes);

  /* creation of DeviceManagment */
  DeviceManagmentHandle = osThreadNew(TaskSwitchMode, NULL, &DeviceManagment_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.SlaveStartFilterBank=1;
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
	{
	Error_Handler();
	}
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.SlaveStartFilterBank=1;
	if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
	{
	Error_Handler();
	}
	sFilterConfig.FilterBank=14;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
	if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
	{
	Error_Handler();
	}

  /* USER CODE END CAN2_Init 2 */

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
  htim6.Init.Prescaler = 1200;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PinHi_Pin|PWR_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HL1_1_GPIO_Port, HL1_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HL1_2_Pin|HL2_1_Pin|HL2_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PinHi_Pin PWR_ON_Pin */
  GPIO_InitStruct.Pin = PinHi_Pin|PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : m0_Pin m1_Pin m2_Pin */
  GPIO_InitStruct.Pin = m0_Pin|m1_Pin|m2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HL1_1_Pin */
  GPIO_InitStruct.Pin = HL1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HL1_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HL1_2_Pin HL2_1_Pin HL2_2_Pin */
  GPIO_InitStruct.Pin = HL1_2_Pin|HL2_1_Pin|HL2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//колбэк CAN

void HAL_CAN_RxFifo1FullCallBack(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
	
	
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&RxHeader,RxData);
	RxIDCurrentFrame= RxHeader.IDE;
}
/*

*/
void CanSendMessage (void)
{
	
	uint32_t TxMailBox = 0;
	h.ExtId = 0x1C00;
	h.DLC = 8;
	
	TxData[0] = ((uint8_t)U_Value);
	TxData[1] = (uint8_t)I_Value;
	TxData[2] = ADC1_average;
	TxData[3] = ADC2_average;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0xFF;
	TxData[7] = 0xFF;
	if(HAL_CAN_AddTxMessage(&hcan2,&h,TxData,&TxMailBox) !=HAL_OK)
	{
		for (int i; i<10; i++)
		TxData[i] = 0x00;
	}
	//HAL_CAN_AddTxMessage(&hcan2,&h,TxData,&TxMailBox);
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		HAL_ADC_Stop_DMA(&hadc1);
		ADC1_U_Val = ADC1ConvertedValue[0];
		ADC2_I_Val = ADC1ConvertedValue[1];
		ADC1ConvertedValue[0] = 0;
		ADC1ConvertedValue[1] = 0;
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC1ConvertedValue,3);
		//HAL_ADC_Start_DMA(&hadc2,(uint32_t*) &ADC2ConvertedValue,1);
	}
}

void EmergencySparkLED (uint8_t NumberOfFlashes)
{
	if(NumberOfFlashes != 0 & HAL_TIM_Base_GetState(&htim6) == HAL_TIM_STATE_READY)
	{
		//включаем ТС и разрешаем прерывания
		HAL_TIM_Base_Start_IT(&htim6);
	}
	
	else if(NumberOfFlashes == 0)
	{
		HAL_TIM_Base_Stop_IT(&htim6);
		flagTIM6 = 0;
		counterFlash = 0;
	}
	if(NumberOfFlashes != 0 & HAL_TIM_Base_GetState(&htim6) == HAL_TIM_STATE_BUSY)	
	if(counterFlash < NumberOfFlashes)
	{
		switch(flagTIM6)
		{
			case 0:
				POWER_LED_ERROR;
			break;
			case 1:
				POWER_LED_OFF;
			break;
			default:
				flagTIM6 = 0;
				counterFlash++;
			break;
		}
	}		
		if(counterFlash >= NumberOfFlashes)	
		{
			POWER_LED_OFF;
			if(flagTIM6 >= 4)
			{
				counterFlash = 0;
				flagTIM6 = 0;
			}		
		}	
}
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
  /* Infinite loop */
  for(;;)
  {
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
		//osDelay(300);
		if(U_Value < VoltageMinimum)
		{
			EmergencySparkLED(1);
			osDelay(1000);
			//POWER_OFF
		}
		else if(U_Value > VoltageMax)
		{
			EmergencySparkLED(2);
		}
		else if(U_Value > VoltageMax2)
		{
			EmergencySparkLED(3);
		}
		else 
		{
			EmergencySparkLED(2);
		}
		if(I_Value > CurrentMax)
		{
			//EmergencySparkLED(2);
			POWER_OFF;
			POWER_LED_ERROR;
			osDelay(1000);
		}
  
		
		osDelay(10000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TaskCanBus */
/**
* @brief Function implementing the smrtCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskCanBus */
void TaskCanBus(void *argument)
{
  /* USER CODE BEGIN TaskCanBus */
  /* Infinite loop */
  for(;;)
  {
			//CanGetMessage(RxData);
			if(RxIDCurrentFrame == SmrtCanRequestFrame)
			{
				CAN_LED_OK
				CanSendMessage();
				DeviceStatus.ErrorCAN = DeviceErrorCanConnect;
			}
			if(RxIDCurrentFrame != SmrtCanRequestFrame)
			{
				CAN_LED_ERROR;
				DeviceStatus.ErrorCAN = DeviceErrorCanNoConnect;
			}
			if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 0)
			{
				CanSendMessage();
			}
			
			
			osDelay(100);
			
  }
  /* USER CODE END TaskCanBus */
}

/* USER CODE BEGIN Header_TaskADC */
/**
* @brief Function implementing the ScanADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskADC */
void TaskADC(void *argument)
{
  /* USER CODE BEGIN TaskADC */
  /* Infinite loop */
  for(;;)
  {
			//Выборка значения из ADC1 - напряжение 	
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			ADC1_U_Val = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);	
			//Выборка значения из ADC2 - ток 
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
			ADC2_I_Val = HAL_ADC_GetValue(&hadc2);
			HAL_ADC_Stop(&hadc2);
			//добавление значений в массив
			for(int i = 9; i>0; i--)
			{
				if(i!=0)
					ADC1_mass_value[i] = ADC1_mass_value[i-1];//сдвиг старых значений в массиве		
			}
			
			ADC1_mass_value[0] = ADC1_U_Val;
			
			//добавление значений в массив
			for(int i = 9; i>0; i--)
				{
						ADC2_mass_value[i] = ADC2_mass_value[i-1];//сдвиг старых значений в массиве		
				}
			ADC2_mass_value[0] = ADC2_I_Val;//добавление нового значения в массив	
			
			for(int i = 0; i<9; i++)
			{
				ADC1_average += ADC1_mass_value[i];
				ADC2_average += ADC2_mass_value[i];
			}
			ADC1_average/=10;//среднее значение U
			ADC2_average/=10;//среднее значение I
			//рассчет значения тока и напряжения
			I_Value = (float)(ADC2_average - ADC2_CURRENT_ZERO_VALUE)/28;
			U_Value = (float)(ADC1_average)/122;
			
			//Обработка АЦП раз 100мс 
			osDelay(50);
    
  }
  /* USER CODE END TaskADC */
}

/* USER CODE BEGIN Header_TaskSwitchMode */
/**
* @brief Function implementing the DeviceManagment thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskSwitchMode */
void TaskSwitchMode(void *argument)
{
  /* USER CODE BEGIN TaskSwitchMode */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

  }
  /* USER CODE END TaskSwitchMode */
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
	if (htim == &htim6)
	{
		flagTIM6++;
	}

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

