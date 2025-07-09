/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef enum{
	idle, follow_line, turn_right, turn_left, retrace, finish
} state;

state robot_state = idle;

uint8_t sw_pushed = 0;
char msg[100] = "";
uint8_t left2 = 0;
uint8_t left1 = 0;
uint8_t center = 0;
uint8_t right1 = 0;
uint8_t right2 = 0;

uint8_t slow_pulse_width = 45;			// in percentage!
uint8_t fast_pulse_width = 90;

uint32_t front_left_enc_count = 0;
uint32_t front_right_enc_count = 0;
uint32_t back_left_enc_count = 0;
uint32_t back_right_enc_count = 0;

// variables required for turning the robot
uint32_t wheel_enc_count[4] = {0, 0, 0, 0};		// front left, front right, back left, back right
uint32_t travel_dist = 5;						// distance the robot must travel, in encoder indentations, before making a turn

//uint16_t pulse_val = 0;		        // set the capture/compare register to this value to generate a PWM signal with Ton corresponding to the pulse width
//uint16_t inv_pulse_val = 0;			// set the capture/compare register to this value to generate a PWM signal with Toff corresponding to the pulse width
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void move_forward(void);
void move_backward(void);
void steer_right(void);
void steer_left(void);
void stop(void);

uint16_t calc_pulse_val(TIM_HandleTypeDef *htim, uint8_t pulse_width);
uint16_t calc_inv_pulse_val(TIM_HandleTypeDef *htim, uint8_t pulse_width);
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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // sample the IR sensors and store the result in the corresponding variables
	  left2 = (HAL_GPIO_ReadPin(GPIOB, left2_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  left1 = (HAL_GPIO_ReadPin(GPIOB, left1_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  center = (HAL_GPIO_ReadPin(GPIOC, center_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  right1 = (HAL_GPIO_ReadPin(GPIOC, right1_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  right2 = (HAL_GPIO_ReadPin(right2_ir_GPIO_Port, right2_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;

	  // create a state machine to control the robot
	  switch (robot_state) {
		  case idle:
			  stop();
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = follow_line;
			  }
			  break;
		  case follow_line:
			  if (right1 && !left1) {
				  steer_right();
			  }
			  else if (left1 && !right1) {
				  steer_left();
			  }
			  else if (left2 && left1 && center && right1 && right2) {
				  sw_pushed = 0;
				  robot_state = finish;
			  }
			  else {
				  move_forward();
			  }

			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = retrace;
			  }
			  break;
		  case turn_right:
			  if ((front_left_enc_count - wheel_enc_count[0]) >= travel_dist &&
					  (front_right_enc_count - wheel_enc_count[1]) >= travel_dist &&
					  (back_left_enc_count - wheel_enc_count[2]) >= travel_dist &&
					  (back_right_enc_count - wheel_enc_count[3]) >= travel_dist) {
				  if (center) {
					  robot_state = follow_line;
				  }
				  else {
					  steer_right();
				  }
			  }
			  else {
				  move_forward();
			  }

			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  case turn_left:
			  if ((front_left_enc_count - wheel_enc_count[0]) >= travel_dist &&
					  (front_right_enc_count - wheel_enc_count[1]) >= travel_dist &&
					  (back_left_enc_count - wheel_enc_count[2]) >= travel_dist &&
					  (back_right_enc_count - wheel_enc_count[3]) >= travel_dist) {
				  if (center) {
					  robot_state = follow_line;
				  }
				  else {
					  steer_left();
				  }
			  }
			  else {
				  move_forward();
			  }

			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  case retrace:
			  if (right2) {
				  wheel_enc_count[0] = front_left_enc_count;
				  wheel_enc_count[1] = front_right_enc_count;
				  wheel_enc_count[2] = back_left_enc_count;
				  wheel_enc_count[3] = back_right_enc_count;
				  robot_state = turn_right;
			  }
			  else if (left2) {
				  wheel_enc_count[0] = front_left_enc_count;
				  wheel_enc_count[1] = front_right_enc_count;
				  wheel_enc_count[2] = back_left_enc_count;
				  wheel_enc_count[3] = back_right_enc_count;
				  robot_state = turn_left;
			  }
			  else if (right1 && !left1) {
				  steer_left();
			  }
			  else if (left1 && !right1) {
				  steer_right();
			  }
			  else if (left2 && left1 && center && right1 && right2) {
				  sw_pushed = 0;
				  robot_state = finish;
			  }
			  else {
				  move_backward();
			  }

			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  case finish:
			  stop();
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  default:
			  break;
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim6.Init.Prescaler = 23;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
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
  htim16.Init.Prescaler = 2399;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, left_in2_Pin|left_in4_Pin|right_in2_Pin|right_in4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : left_in2_Pin left_in4_Pin right_in2_Pin right_in4_Pin */
  GPIO_InitStruct.Pin = left_in2_Pin|left_in4_Pin|right_in2_Pin|right_in4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : right2_ir_Pin */
  GPIO_InitStruct.Pin = right2_ir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(right2_ir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : right1_ir_Pin center_ir_Pin */
  GPIO_InitStruct.Pin = right1_ir_Pin|center_ir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : left1_ir_Pin left2_ir_Pin */
  GPIO_InitStruct.Pin = left1_ir_Pin|left2_ir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : bumper_sw_Pin */
  GPIO_InitStruct.Pin = bumper_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(bumper_sw_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t calc_pulse_val(TIM_HandleTypeDef *htim, uint8_t pulse_width) {
	return (uint16_t) (((float) (pulse_width) / 100.0) * htim->Init.Period);
}

uint16_t calc_inv_pulse_val(TIM_HandleTypeDef *htim, uint8_t pulse_width) {
	return (uint16_t) (((float) (100 - pulse_width) / 100.0) * htim->Init.Period);
}

void move_forward(void) {
	// code using PWM generation
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_inv_pulse_val(&htim2, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, calc_inv_pulse_val(&htim2, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_pulse_val(&htim1, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_pulse_val(&htim1, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_RESET);
}

void move_backward(void) {
	// code using PWM generation
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_pulse_val(&htim2, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,calc_pulse_val(&htim2, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_inv_pulse_val(&htim1, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_inv_pulse_val(&htim1, slow_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_SET);
}

void steer_right(void) {
	// code using PWM generation
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_inv_pulse_val(&htim2, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, calc_inv_pulse_val(&htim2, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_inv_pulse_val(&htim1, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_inv_pulse_val(&htim1, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_SET);
}

void steer_left(void) {
	// code using PWM generation
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_pulse_val(&htim2, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, calc_pulse_val(&htim2, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_pulse_val(&htim1, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_pulse_val(&htim1, fast_pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_RESET);
}

void stop(void) {
	// code using PWM generation
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_RESET);
}

void EXTI2_3_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(bumper_sw_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == bumper_sw_Pin) {
		HAL_TIM_Base_Start_IT(&htim6);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		HAL_TIM_Base_Stop_IT(htim);

		// change the mode of the external interrupt pin into a GPIO input pin
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = bumper_sw_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(bumper_sw_GPIO_Port, &GPIO_InitStruct);

		if (HAL_GPIO_ReadPin(bumper_sw_GPIO_Port, bumper_sw_Pin) == GPIO_PIN_RESET) {
			if (sw_pushed == 0) {
				sw_pushed = 1;
			}
			else {
				sw_pushed = 0;
			}
		}

		// change the mode of the GPIO pin back into an external interrupt
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(bumper_sw_GPIO_Port, &GPIO_InitStruct);
	}
	else if (htim->Instance == TIM16) {
		// transmit the state of the IR sensors over UART
		sprintf(msg, "IR sensors left to right: %d	%d	%d	%d	%d	Encoders: fl %lu fr %lu bl %lu br %lu\n\r", left2, left1, center, right1, right2, front_left_enc_count, front_right_enc_count, back_left_enc_count, back_right_enc_count);
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
	}
	else {

	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			++back_right_enc_count;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			++back_left_enc_count;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			++front_right_enc_count;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			++front_left_enc_count;
		}
		else {

		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
