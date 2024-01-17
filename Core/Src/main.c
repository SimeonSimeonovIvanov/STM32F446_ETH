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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../lwip/Target/enc424j600/enc424j600.h"
#include "lwip.h"
#include "semphr.h"
#include "api.h"

#include "bit-array.h"

#include "mb.h"
#include "mbport.h"
#include "mbutils.h"

#include "bacnet/bacdef.h"

#include "bacnet_task.h"
#include "hmi_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */
#define REG_DISC_START							1
#define REG_DISC_SIZE							256

#define REG_COILS_START							1
#define REG_COILS_SIZE							256

#define REG_INPUT_START							1
#define REG_INPUT_NREGS							16

#define REG_HOLDING_START						1
#define REG_HOLDING_NREGS						50
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart4;

/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
uint32_t StartDefaultTaskBuffer[ 200 ];
osStaticThreadDef_t StartDefaultTaskControlBlock;
const osThreadAttr_t DefaultTask_attributes = {
  .name = "DefaultTask",
  .cb_mem = &StartDefaultTaskControlBlock,
  .cb_size = sizeof(StartDefaultTaskControlBlock),
  .stack_mem = &StartDefaultTaskBuffer[0],
  .stack_size = sizeof(StartDefaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myHmiTask */
osThreadId_t myHmiTaskHandle;
uint32_t myHmiTaskBuffer[ 128 ];
osStaticThreadDef_t myHmiTaskControlBlock;
const osThreadAttr_t myHmiTask_attributes = {
  .name = "myHmiTask",
  .cb_mem = &myHmiTaskControlBlock,
  .cb_size = sizeof(myHmiTaskControlBlock),
  .stack_mem = &myHmiTaskBuffer[0],
  .stack_size = sizeof(myHmiTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01",
  .cb_mem = &myQueue01ControlBlock,
  .cb_size = sizeof(myQueue01ControlBlock),
  .mq_mem = &myQueue01Buffer,
  .mq_size = sizeof(myQueue01Buffer)
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01",
  .cb_mem = &myBinarySem01ControlBlock,
  .cb_size = sizeof(myBinarySem01ControlBlock),
};
/* USER CODE BEGIN PV */
osSemaphoreId_t myBinarySemSpiHandle;
const osSemaphoreAttr_t myBinarySemSpi_attributes = {
  .name = "myBinarySemSpi"
};

osSemaphoreId_t myMbTCPSem01Handle;
osStaticSemaphoreDef_t myBinaryMbTCPSem01ControlBlock;
const osSemaphoreAttr_t myBinaryMbTCPSem01_attributes = {
  .name = "mbTCPSem01",
  .cb_mem = &myBinaryMbTCPSem01ControlBlock,
  .cb_size = sizeof(myBinaryMbTCPSem01ControlBlock),
};

static StaticTask_t  ModBusTCPSlaveTaskCB[NUMBER_OF_MB_MASTER_TCP_CONN];
static StackType_t   ModBusTCPSlaveTaskStk[NUMBER_OF_MB_MASTER_TCP_CONN][200];
const osThreadAttr_t ModBusTCPSlaveTask_attributes[NUMBER_OF_MB_MASTER_TCP_CONN] =
{
#if NUMBER_OF_MB_MASTER_TCP_CONN > 0
	{
		.name		= "mbTCPSTask1",
		.cb_mem		= &ModBusTCPSlaveTaskCB[0],
		.cb_size	= sizeof(ModBusTCPSlaveTaskCB[0]),
		.stack_mem	= &ModBusTCPSlaveTaskStk[0][0],
		.stack_size	= sizeof(ModBusTCPSlaveTaskStk[0]),
		.priority	= (osPriority_t) osPriorityNormal,
	},
#endif
#if NUMBER_OF_MB_MASTER_TCP_CONN > 1
	{
		.name		= "mbTCPSTask2",
		.cb_mem		= &ModBusTCPSlaveTaskCB[1],
		.cb_size	= sizeof(ModBusTCPSlaveTaskCB[1]),
		.stack_mem	= &ModBusTCPSlaveTaskStk[1][0],
		.stack_size	= sizeof(ModBusTCPSlaveTaskStk[1]),
		.priority	= (osPriority_t) osPriorityNormal,
	},
#endif
#if NUMBER_OF_MB_MASTER_TCP_CONN > 2
	{
		.name		= "mbTCPSTask3",
		.cb_mem		= &ModBusTCPSlaveTaskCB[2],
		.cb_size	= sizeof(ModBusTCPSlaveTaskCB[2]),
		.stack_mem	= &ModBusTCPSlaveTaskStk[2][0],
		.stack_size	= sizeof(ModBusTCPSlaveTaskStk[2]),
		.priority	= (osPriority_t) osPriorityNormal,
	},
#endif
#if NUMBER_OF_MB_MASTER_TCP_CONN > 3
	{
		.name		= "mbTCPSTask4",
		.cb_mem		= &ModBusTCPSlaveTaskCB[3],
		.cb_size	= sizeof(ModBusTCPSlaveTaskCB[3]),
		.stack_mem	= &ModBusTCPSlaveTaskStk[3][0],
		.stack_size	= sizeof(ModBusTCPSlaveTaskStk[3]),
		.priority	= (osPriority_t) osPriorityNormal,
	},
#endif
#if NUMBER_OF_MB_MASTER_TCP_CONN > 4
	{
		.name		= "mbTCPSTask5",
		.cb_mem		= &ModBusTCPSlaveTaskCB[4],
		.cb_size	= sizeof(ModBusTCPSlaveTaskCB[4]),
		.stack_mem	= &ModBusTCPSlaveTaskStk[4][0],
		.stack_size	= sizeof(ModBusTCPSlaveTaskStk[4]),
		.priority	= (osPriority_t) osPriorityNormal,
	},
#endif
};

static StaticTask_t  ModBusTCPSlaveАcceptTaskCB;
static StackType_t   ModBusTCPSlaveАcceptTaskStk[300];
const osThreadAttr_t ModBusTCPSlaveАcceptTask_attributes =
{
	.name       = "mbTCPSАcceptTask",
	.cb_mem     = &ModBusTCPSlaveАcceptTaskCB,
	.cb_size    = sizeof(ModBusTCPSlaveАcceptTaskCB),
	.stack_mem  = &ModBusTCPSlaveАcceptTaskStk,
	.stack_size = sizeof(ModBusTCPSlaveАcceptTaskStk),
	.priority   = (osPriority_t) osPriorityNormal,
};

static StaticTask_t  ModBusSlaveTaskCB;
static StackType_t   ModBusSlaveTaskStk[128];
const osThreadAttr_t ModBusSlaveTask_attributes =
{
	.name		= "mbSlaveTask",
	.cb_mem		= &ModBusSlaveTaskCB,
	.cb_size	= sizeof(ModBusSlaveTaskCB),
	.stack_mem	= &ModBusSlaveTaskStk,
	.stack_size	= sizeof(ModBusSlaveTaskStk),
	.priority	= (osPriority_t) osPriorityNormal,
};

static osThreadId_t  BacnetTaskHandle;
static StaticTask_t  BacnetTaskCB;
static StackType_t   BacnetTaskStk[2048];//(512 + (MAX_APDU * 4)) * 4];
const osThreadAttr_t BacnetTask_attributes =
{
		.name		= "BacnetTask",
		.cb_mem		= &BacnetTaskCB,
		.cb_size	= sizeof(BacnetTaskCB),
		.stack_mem	= &BacnetTaskStk,
		.stack_size	= sizeof(BacnetTaskStk),
		.priority	= (osPriority_t) osPriorityNormal,
};

extern BACNET_BINARY_PV Binary_Output_Level[/*MAX_BINARY_OUTPUTS*/][BACNET_MAX_PRIORITY];
extern stModbusConn arrModbusConn[NUMBER_OF_MB_MASTER_TCP_CONN];

static HMI_TASK_ARG hmi_task_arg;

static uint8_t arrInput[ 256 ] = { 0 };
static uint8_t arrOutput[ 256 ] = { 0 };
static uint8_t usRS485PortLed[ 2 ] = { 0 };

static uint8_t ucRegDiscBuf[(REG_DISC_SIZE / 8) + 1];
static uint8_t ucRegCoilsBuf[(REG_COILS_SIZE / 8) + 1];
static uint16_t usRegInputBuf[REG_INPUT_NREGS];
static uint16_t uiRegHolding[REG_HOLDING_NREGS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM9_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void MX_TIM9_Init_New(void);
void ModBusTCPSlaveTask(void *argument);
void ModBusSlaveTask(void *argument);
void StartDefaultTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern osSemaphoreId s_xSemaphore;
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
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_UART4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
//__HAL_DBGMCU_FREEZE_TIM9();
  HAL_TIM_PWM_Start( &htim9, TIM_CHANNEL_1 );

  const uint8_t ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
  eMBErrorCode eStatus;
  //eStatus = eMBInit( MB_TCP, 0x0A, 0, 38400, MB_PAR_EVEN );
  eStatus = eMBTCPInit(502);
  eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 );
  eStatus = eMBEnable();
  eStatus = eStatus;

  hmiInitLeds();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySemSpi */
  myBinarySemSpiHandle = osSemaphoreNew(1, 1, &myBinarySemSpi_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  myMbTCPSem01Handle = osSemaphoreNew(1, 1, &myBinaryMbTCPSem01_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &DefaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  BacnetTaskHandle = osThreadNew(BacnetTask, (void*)NULL, &BacnetTask_attributes);

  hmi_task_arg.lpRS485PortLed[0] = &usRS485PortLed[0];
  hmi_task_arg.lpRS485PortLed[1] = &usRS485PortLed[1];
  hmi_task_arg.arrInput = arrInput;
  hmi_task_arg.arrOutput = arrOutput;
  hmi_task_arg.arrInputLen = len_of_array( arrInput );
  hmi_task_arg.arrOutputLen = len_of_array( arrOutput );
  myHmiTaskHandle = osThreadNew(HmiTask, (void*) &hmi_task_arg, &myHmiTask_attributes);

  MX_LWIP_Init();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */
	MX_TIM9_Init_New();
	return;
  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 38400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS1_Pin|CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART4_RTS_GPIO_Port, UART4_RTS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS0_Pin */
  GPIO_InitStruct.Pin = CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS2_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UART4_RTS_Pin */
  GPIO_InitStruct.Pin = UART4_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART4_RTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_TIM9_Init_New(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 4;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 5000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);
}

eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{	// MB_FUNC_READ_DISCRETE_INPUTS          ( 2 )
	volatile eMBErrorCode eStatus = MB_ENOERR;
	volatile short iNDiscrete = ( short )usNDiscrete;
	volatile unsigned short usBitOffset;
	/* Check if we have registers mapped at this block. */
	if( (usAddress >= REG_DISC_START) &&
		(usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE)
	)
	{
		for(int i = 0; i < REG_DISC_SIZE; i++ )
		{
			bitarr_write(ucRegDiscBuf, i, 1 & arrInput[i]);
		}
//		uiModbusTimeOutCounter = 0;
		usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
		while(iNDiscrete > 0)
		{
			*pucRegBuffer++ =
			xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                            (unsigned char)(iNDiscrete>8? 8:iNDiscrete)
			);
			iNDiscrete -= 8;
			usBitOffset += 8;
		}
		arrInput[0]++;
		return MB_ENOERR;
	}

	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;

	/* Check if we have registers mapped at this block. */
	if( (usAddress >= REG_COILS_START) &&
		(usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE)
	) {
		usBitOffset = (unsigned short)(usAddress - REG_COILS_START);
		switch(eMode)
		{
		// Read current values and pass to protocol stack.
		// MB_FUNC_READ_COILS						( 1 )
		case MB_REG_READ:
//			uiModbusTimeOutCounter = 0;
			for(int i = 0; i < REG_COILS_SIZE; i++ )
			{
				if(i < 16/*MAX_BINARY_OUTPUTS*/)
				{
					arrOutput[i] = (BINARY_ACTIVE == Binary_Output_Level[i][15]) ? 1 : 0;
				}
				bitarr_write(ucRegCoilsBuf, i, 1 & arrOutput[i]);
			}
			while( iNCoils > 0 )
			{
				*pucRegBuffer++ =
				xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
								(unsigned char)((iNCoils > 8) ? 8 : iNCoils)
				);
				usBitOffset += 8;
				iNCoils -= 8;
			}
		 return MB_ENOERR;
		 // Update current register values.
		 // MB_FUNC_WRITE_SINGLE_COIL				( 5 )
		 // MB_FUNC_WRITE_MULTIPLE_COILS			( 15 )
		 case MB_REG_WRITE:
//		 	uiModbusTimeOutCounter = 0;
		 	while( iNCoils > 0 )
		 	{
				xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
								(unsigned char)((iNCoils > 8) ? 8 : iNCoils),
								*pucRegBuffer++
				);
				usBitOffset += 8;
				iNCoils -= 8;
			}
		 	for(int i = 0; i < REG_COILS_SIZE; i++ )
		 	{
		 		arrOutput[i] = bitarr_read(ucRegCoilsBuf, i);
		 		if(i < 16/*MAX_BINARY_OUTPUTS*/)
			 	{
			 		Binary_Output_Level[i][15] = (1 & arrOutput[i]) ? BINARY_ACTIVE : BINARY_INACTIVE;
			 	}
		 	}
		 return MB_ENOERR;
		}
	}

	return MB_ENOREG;
}

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
//    	usRegInputBuf[len_of_array(arrADC)]++;
//    	for(int i = 0; i < len_of_array(arrADC); i++ )
//    	{
//    		usRegInputBuf[i] = arrADC[i];
//    	}
    	usRegInputBuf[0]++;
        iRegIndex = ( int )( usAddress - REG_INPUT_START );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
	unsigned int iRegIndex;

	/* Check if we have registers mapped at this block. */
	if( (usAddress >= REG_HOLDING_START) &&
		(usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)
	)
	{
		switch(eMode) {
		case MB_REG_READ:
//			uiModbusTimeOutCounter = 0;
			iRegIndex = (int)(usAddress - 1);
			while( usNRegs > 0 ) {
				*pucRegBuffer++ = uiRegHolding[iRegIndex]>>8;
				*pucRegBuffer++ = uiRegHolding[iRegIndex];
				++iRegIndex;
				--usNRegs;
			}
		 return MB_ENOERR;
		// Update current register values.
		// MB_FUNC_WRITE_MULTIPLE_REGISTERS             (16)
		case MB_REG_WRITE: {
			uint16_t dac = uiRegHolding[4];
			// Запазва текущия работен диапазон на ADC и DAC.
			uint16_t old = 0x0F03 & uiRegHolding[14];
//			uint16_t oldUartControl = 0x000f & uiRegHolding[17];

//			uiModbusTimeOutCounter = 0;
			iRegIndex = (int)(usAddress - 1);
			while( usNRegs > 0 ) {
				uiRegHolding[iRegIndex]  = (*pucRegBuffer++)<<8;
				uiRegHolding[iRegIndex] |= *pucRegBuffer++;
				++iRegIndex;
				--usNRegs;
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(0x0080 & uiRegHolding[4]) {
				uiRegHolding[4] = (0x007f & dac) | (0xff00 & uiRegHolding[4]);
			}

			if(0x8000 & uiRegHolding[4]) {
				uiRegHolding[4] = (0x7f00 & dac) | (0x00ff & uiRegHolding[4]);
			}

			// Ако не е зададен нов работен диапазон, се възстановява стария:
			if(!(0x0003 & uiRegHolding[14])) uiRegHolding[14] |= (0x0003 & old);

			// DAC0 - Запазване на стария работен диапазон:
			if(!(0x0300 & uiRegHolding[14])) uiRegHolding[14] |= (0x0300 & old);
			// DAC1 - Запазване на стария работен диапазон:
			if(!(0x0C00 & uiRegHolding[14])) uiRegHolding[14] |= (0x0C00 & old);

			//if(!(0x0003 & uiRegHolding[17])) uiRegHolding[17] |= (0x0003 & oldUartControl);
			//if(!(0x000c & uiRegHolding[17])) uiRegHolding[17] |= (0x000c & oldUartControl);
		}
		 return MB_ENOERR;
		}
	}

	return MB_ENOREG;
}

void ModBusTCPSlaveTask(void *argument)
{
	stModbusConn *lpModbusConn = (stModbusConn *)argument;
	eMBEventType eEvent;
	struct netbuf *buf;

	while(1)
	{
		if( NULL == lpModbusConn->newconn )
		{
			vTaskSuspend(lpModbusConn->hTask/*NULL*/);
		}

		//netconn_set_sendtimeout(lpModbusConn->newconn, 5000);
		netconn_set_recvtimeout(lpModbusConn->newconn, 5000);
		while( ERR_OK == netconn_recv(lpModbusConn->newconn, &buf) )
		{
			do
			{
				lpModbusConn->len = pbuf_copy_partial(buf->p, lpModbusConn->aucTCPBuf, len_of_array(lpModbusConn->aucTCPBuf), 0);

				eEvent = EV_FRAME_RECEIVED;
				if( xQueueSend(lpModbusConn->xQueueMbRX, (const void *)&eEvent, portMAX_DELAY) )
				{
					eMBEventType eEvent = EV_READY;
					if( xMBPortEventGetTX(lpModbusConn, &eEvent) )
					{
						if( EV_FRAME_SENT == eEvent )
						{
							netconn_write(lpModbusConn->newconn, lpModbusConn->aucTCPBuf, lpModbusConn->len, NETCONN_COPY);
						}
					}
				}
			} while( netbuf_next(buf) > 0 );
			netbuf_delete(buf);
			portYIELD();
		}

		netconn_close(lpModbusConn->newconn);
		netconn_delete(lpModbusConn->newconn);
		lpModbusConn->newconn = NULL;
	}
}

void ModBusTCPSlaveNoPollTask(void *argument)
{
	stModbusConn *lpModbusConn = (stModbusConn *)argument;
	struct netbuf *buf;

	while(1)
	{
		if( NULL == lpModbusConn->newconn )
		{
			vTaskSuspend(lpModbusConn->hTask/*NULL*/);
		}

		//netconn_set_sendtimeout(lpModbusConn->newconn, 5000);
		netconn_set_recvtimeout(lpModbusConn->newconn, 5000);
		while( ERR_OK == netconn_recv(lpModbusConn->newconn, &buf) )
		{
			do
			{
				lpModbusConn->len = pbuf_copy_partial(buf->p, lpModbusConn->aucTCPBuf, len_of_array(lpModbusConn->aucTCPBuf), 0);
				if( MB_ENOERR == eMBNoPollTcp(lpModbusConn) )
				{
					netconn_write(lpModbusConn->newconn, lpModbusConn->aucTCPBuf, lpModbusConn->len, NETCONN_COPY);
				}
			} while( netbuf_next(buf) > 0 );
			netbuf_delete(buf);
			portYIELD();
		}

		netconn_close(lpModbusConn->newconn);
		netconn_delete(lpModbusConn->newconn);
		lpModbusConn->newconn = NULL;
	}
}

void ModBusTCPSlaveАcceptTask(void *argument)
{
	err_t err, accept_err;
	struct netconn *conn;
	int i;

	for( i = 0; i < len_of_array(arrModbusConn); i++ )
	{
		arrModbusConn[i].newconn = NULL;
//		arrModbusConn[i].hTask = osThreadNew(ModBusTCPSlaveTask, &arrModbusConn[i], &ModBusTCPSlaveTask_attributes[i]);
		arrModbusConn[i].hTask = osThreadNew(ModBusTCPSlaveNoPollTask, &arrModbusConn[i], &ModBusTCPSlaveTask_attributes[i]);
	}

	conn = netconn_new(NETCONN_TCP);
	if( NULL != conn )
	{
		err = netconn_bind(conn, IP_ADDR_ANY, 502);
		if( ERR_OK == err )
		{
			netconn_listen(conn);
			while(1)
			{
				i = 0;
				while( (NULL != arrModbusConn[i].newconn) )//|| (eSuspended != eTaskGetState(arrModbusConn[i].hTask)))
				{
					if( ++i >= len_of_array(arrModbusConn) )
					{
						vTaskDelay(5);
						i = 0;
					}
				}
				accept_err = netconn_accept(conn, &arrModbusConn[i].newconn);
				if( ERR_OK == accept_err )
				{
					vTaskResume(arrModbusConn[i].hTask);
				} else {
					arrModbusConn[i].newconn = NULL;
				}
			}
		}
		netconn_delete(conn);
	}

	osThreadExit();
}

void ModBusSlaveTask(void *argument)
{
	for(;;)
	{
		//eMBPoll();
		//eMBPollTcp();
		portYIELD();
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	extern struct netif gnetif;

	while( !netif_is_link_up(&gnetif) )
	{
		vTaskDelay(5);
	}

	osThreadNew(ModBusTCPSlaveАcceptTask, NULL, &ModBusTCPSlaveАcceptTask_attributes);
	osThreadNew(ModBusSlaveTask, NULL, &ModBusSlaveTask_attributes);

	for(;;)
	{
		//enc424j600EventHandler();
		//if( EIR_PKTIF & enc424j600EventHandler() ) //enc424j600ReadReg(EIRL) )
		xSemaphoreTake(myBinarySemSpiHandle, (TickType_t)portMAX_DELAY);
		if(EIR_PKTIF & enc424j600ReadReg(EIR))
		{
			extern osSemaphoreId RxPktSemaphore;
//			enc424j600BFCReg(EIR, EIR_PKTIF);
			if(NULL != RxPktSemaphore)
			{
				osSemaphoreRelease(RxPktSemaphore);
			}
//			usRS485PortLed[0] ^= 1;
		}
		xSemaphoreGive(myBinarySemSpiHandle);
		portYIELD();
	}
  /* USER CODE END 5 */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
