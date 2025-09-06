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
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	idle, manual_mode, follow_line, follow_line_until_turn, turn_right, turn_left, turn_around, t_intersection, finish
} state;

typedef enum{
	start_seq1, start_seq2, length, payload, checksum, send_data, req_resend, finished
} receive_fsm;

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
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
state robot_state = idle;

volatile uint8_t sw_pushed = 0;

volatile uint8_t remote_msg_received = 1;

char msg[100] = "";

uint8_t msg_from_remote[50];

uint8_t left_joy_x = 50;
uint8_t left_joy_y = 50;
uint8_t right_joy_x = 50;
uint8_t right_joy_y = 50;

uint8_t in_manual_mode = 0;
uint8_t start_robot = 0;
uint8_t in_tank_mode = 0;
uint8_t manual_mode_pulse_width = 50;

uint8_t started_by_remote = 0;

// state of the IR sensors under the robot (0 = white and 1 = black)
uint8_t left2 = 0;
uint8_t left1 = 0;
uint8_t center = 0;
uint8_t right1 = 0;
uint8_t right2 = 0;

uint8_t after_turn_around = 0;

uint8_t slow_pulse_width = 50;					// pulse width of the signal that drives the motors to move the robot slowly (in percentage!)
uint8_t fast_pulse_width = 98;					// pulse width of the signal that drives the motors to move the robot fast (in percentage!)

// variables to keep track of the current encoder count from the timer
volatile uint32_t front_left_enc_count = 0;
volatile uint32_t front_right_enc_count = 0;
volatile uint32_t back_left_enc_count = 0;
volatile uint32_t back_right_enc_count = 0;

// variables to keep track of the previous encoder count from the timer
uint32_t wheel_enc_count[4] = {0, 0, 0, 0};		// front left, front right, back left, back right

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void move_forward(uint8_t pulse_width);
void move_backward(uint8_t pulse_width);
void steer_right(uint8_t pulse_width);
void steer_left(uint8_t pulse_width);
void stop(void);
void record_current_enc_pos(void);
void transmit_data(uint8_t resend);
uint8_t process_data(uint8_t *darray, uint8_t start, uint8_t end);

uint8_t suff_dist_traveled(uint32_t travel_dist);

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
  MX_DMA_Init();
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
	  // receive data from the remote control
	  if (remote_msg_received) {
		  HAL_GPIO_WritePin(debug_sig_GPIO_Port, debug_sig_Pin, GPIO_PIN_SET);
		  HAL_UART_Receive_DMA(&huart1, msg_from_remote, 12);
		  remote_msg_received = 0;

		  uint8_t msg_len = 0;
		  receive_fsm receiver_state = start_seq1;
		  while (receiver_state != finished) {
			  // state machine for processing the received bytes of data
			  switch(receiver_state) {
			  case start_seq1:
				  if ((char) (msg_from_remote[0]) == 'W') {
					  receiver_state = start_seq2;
				  }
				  else {
					  receiver_state = req_resend;
				  }
				  break;
			  case start_seq2:
				  if ((char) (msg_from_remote[1] == 'Z')) {
					  receiver_state = length;
				  }
				  else {
					  receiver_state = req_resend;
				  }
				  break;
			  case length:
				  msg_len = msg_from_remote[2];
				  receiver_state = payload;
				  break;
			  case payload:	{
				  // the process_data function will return 0 if the robot requested a resend
				  uint8_t ret_val = process_data(msg_from_remote, 3, 3 + msg_len);
				  receiver_state = checksum;
				  break;
			  }
			  case checksum:
				  // perform a cyclic redundancy check
				  // this is not implemented yet!
				  receiver_state = send_data;
				  break;
			  case send_data:
				  // transmit data from the robot to the remote control
				  transmit_data(0);
				  receiver_state = finished;
				  break;
			  case req_resend:
				  transmit_data(1);
				  receiver_state = finished;
				  break;
			  default:
				  break;
			  }
		  }
	  }

	  HAL_GPIO_WritePin(debug_sig_GPIO_Port, debug_sig_Pin, GPIO_PIN_RESET);

	  // sample the IR sensors and store the result in the corresponding variables
	  left2 = (HAL_GPIO_ReadPin(GPIOB, left2_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  left1 = (HAL_GPIO_ReadPin(GPIOB, left1_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  center = (HAL_GPIO_ReadPin(GPIOC, center_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  right1 = (HAL_GPIO_ReadPin(GPIOC, right1_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;
	  right2 = (HAL_GPIO_ReadPin(right2_ir_GPIO_Port, right2_ir_Pin) == GPIO_PIN_SET) ? 1 : 0;

	  // create a state machine to control the robot
	  /*
	   * Note: The control logic for the robot works as follows: the robot will continue moving on it's current path until it runs into an obstacle that
	   * presses it's bumper switch. Then the robot will turn around and take the first turn in the path that it encounters. If the bumper switch is
	   * pressed before it has made it's turn, the robot will stop and shift into idle mode. After the robot makes it's turn, it will shift back into
	   * it's default behavior of following the path until it bumps into something or it reaches the finish line.
	   */
	  switch (robot_state) {
	  	  // do not move until the bumper switch is pressed
		  case idle:
			  stop();

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (!in_manual_mode && start_robot) {
				  robot_state = follow_line;
				  started_by_remote = 1;
			  }
			  else {

			  }

			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = follow_line;
			  }
			  break;
		  // in manual mode the robot is directly controlled by the remote control
		  case manual_mode:
			  if (in_tank_mode) {
				  if (left_joy_y < 25 && right_joy_y > 75) {
					  steer_left(manual_mode_pulse_width);
				  }
				  else if (left_joy_y > 75 && right_joy_y < 25) {
					  steer_right(manual_mode_pulse_width);
				  }
				  else if (left_joy_y > 75 && right_joy_y > 75) {
					  move_forward(manual_mode_pulse_width);
				  }
				  else if (left_joy_y < 25 && right_joy_y < 25) {
					  move_backward(manual_mode_pulse_width);
				  }
				  else {
					  stop();
				  }
			  }
			  else {
				  if (left_joy_y < 25) {
					  move_backward(manual_mode_pulse_width);
				  }
				  else if (left_joy_y > 75) {
					  move_forward(manual_mode_pulse_width);
				  }
				  else if (right_joy_x < 25) {
					  steer_left(manual_mode_pulse_width);
				  }
				  else if (right_joy_x > 75) {
					  steer_right(manual_mode_pulse_width);
				  }
				  else {
					  stop();
				  }
			  }

			  if (!in_manual_mode) {
				  robot_state = idle;
			  }

			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  // follow the line until the bumper switch is pressed
		  case follow_line:
			  // if all IR sensors are blocked, possibly at the T intersection or at the finish line
			  if (left1 && center && right1) {
				  record_current_enc_pos();
				  robot_state = t_intersection;
			  }
			  // if right1 is blocked and left1 is not blocked, robot is moving off the line so steer right
			  else if (right1 && !left1) {
				  steer_right(fast_pulse_width);
			  }
			  // if left1 is blocked and right1 is not blocked, robot is moving off the line so steer left
			  else if (left1 && !right1) {
				  steer_left(fast_pulse_width);
			  }
			  // if none of the IR sensors are blocked or only the center IR sensor is blocked, robot is on the line so keep moving forward
			  else {
				  move_forward(slow_pulse_width);
			  }

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (started_by_remote && !start_robot) {
				  robot_state = idle;
				  started_by_remote = 0;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, turn the robot back the way it came
			  if (sw_pushed) {
				  sw_pushed = 0;
				  record_current_enc_pos();
				  robot_state = turn_around;
			  }
			  break;
		  // follow the line until there is a fork in the path
		  case follow_line_until_turn:
			  // if all IR sensors are blocked, possibly at the T intersection or at the finish line
			  if (left1 && center && right1) {
				  record_current_enc_pos();
				  robot_state = t_intersection;
			  }
			  // if right2 is blocked, the robot is at a fork in the path and should turn right
			  else if (right2) {
				  record_current_enc_pos();
				  robot_state = turn_right;
			  }
			  // if left2 is blocked, the robot is at a fork in the path and should turn left
			  else if (left2) {
				  record_current_enc_pos();
				  robot_state = turn_left;
			  }
			  // if right1 is blocked and left1 is not blocked, robot is moving off the line so steer right
			  else if (right1 && !left1) {
				  steer_right(fast_pulse_width);
			  }
			  // if left1 is blocked and right1 is not blocked, robot is moving off the line so steer left
			  else if (left1 && !right1) {
				  steer_left(fast_pulse_width);
			  }
			  // if none of the IR sensors are blocked or only the center IR sensor is blocked, robot is on the line so keep moving forward
			  else {
				  move_forward(slow_pulse_width);
			  }

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (started_by_remote && !start_robot) {
				  robot_state = idle;
				  started_by_remote = 0;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, put the robot in the idle state
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  // turn right onto a different path
		  case turn_right:
			  // keep turning right for a while to move the center IR sensor off the old path
			  if (suff_dist_traveled(5)) {
				  // if the center IR sensor is blocked, the robot is on the new path
				  if (center) {
					  if (after_turn_around) {
						  after_turn_around = 0;
						  robot_state = follow_line_until_turn;
					  }
					  else {
						  robot_state = follow_line;
					  }
				  }
				  else {
					  steer_right(fast_pulse_width);
				  }
			  }
			  else {
				  steer_right(fast_pulse_width);
			  }

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (started_by_remote && !start_robot) {
				  robot_state = idle;
				  started_by_remote = 0;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, put the robot in the idle state
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  // turn left onto a different path
		  case turn_left:
			  // keep turning left for a while to move the center IR sensor off the old path
			  if (suff_dist_traveled(5)) {
				  // if the center IR sensor is blocked, the robot is on the new path
				  if (center) {
					  if (after_turn_around) {
						  after_turn_around = 0;
						  robot_state = follow_line_until_turn;
					  }
					  else {
						  robot_state = follow_line;
					  }
				  }
				  else {
					  steer_left(fast_pulse_width);
				  }
			  }
			  else {
				  steer_left(fast_pulse_width);
			  }

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (started_by_remote && !start_robot) {
				  started_by_remote = 0;
				  robot_state = idle;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, put the robot in the idle state
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  // turn the robot around 180 degrees back onto the path it has already traversed
		  case turn_around:
			  // move the robot back to ensure it won't bump into the obstacle when turning around
			  if (suff_dist_traveled(5)) {

				  // set this variable to ensure the robot's state will transition into the follow_line_until_turn state instead of the follow_line state
				  after_turn_around = 1;

				  record_current_enc_pos();

				  // the robot will always turn left
				  robot_state = turn_left;
			  }
			  else {
				  move_backward(slow_pulse_width);
			  }

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (started_by_remote && !start_robot) {
				  started_by_remote = 0;
				  robot_state = idle;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, put the robot in the idle state
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  // determine whether the robot is at a T intersection or has reached the finish line
		  case t_intersection:
			  // move forward a little bit and check the IR sensors again
			  if (suff_dist_traveled(2)) {
				  // if all sensors are blocked the robot is at the finish line (because the finish line is thicker than the normal path)
				  if (left2 && left1 && center && right1 && right2) {
					  robot_state = finish;
				  }
				  // otherwise the robot is at a T intersection so turn onto the right path
				  else {
					  robot_state = turn_right;
				  }
			  }
			  else {
				  move_forward(slow_pulse_width);
			  }

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (started_by_remote && !start_robot) {
				  started_by_remote = 0;
				  robot_state = idle;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, put the robot in the idle state
			  if (sw_pushed) {
				  sw_pushed = 0;
				  robot_state = idle;
			  }
			  break;
		  // stop the robot when it has reached the finish line
		  case finish:
			  stop();

			  if (in_manual_mode) {
				  robot_state = manual_mode;
			  }
			  else if (!start_robot) {
				  robot_state = idle;
			  }
			  else {

			  }

			  // if the bumper switch is pressed, put the robot in the idle state
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(debug_sig_GPIO_Port, debug_sig_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : debug_sig_Pin */
  GPIO_InitStruct.Pin = debug_sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(debug_sig_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// generating the PWM signals to drive the motors requires providing the on time of the signal as a fraction of the Period field of the timer
// this function accepts a pulse_width parameter in percentage and generates an integer with the required on time for the PWM signal
uint16_t calc_pulse_val(TIM_HandleTypeDef *htim, uint8_t pulse_width) {
	return (uint16_t) (((float) (pulse_width) / 100.0) * htim->Init.Period);
}

// this function accepts a pulse_width parameter in percentage and generates an integer with the required off time for the PWM signal
uint16_t calc_inv_pulse_val(TIM_HandleTypeDef *htim, uint8_t pulse_width) {
	return (uint16_t) (((float) (100 - pulse_width) / 100.0) * htim->Init.Period);
}

// this function records the current encoder count of the wheels
void record_current_enc_pos(void) {
	wheel_enc_count[0] = front_left_enc_count;
	wheel_enc_count[1] = front_right_enc_count;
	wheel_enc_count[2] = back_left_enc_count;
	wheel_enc_count[3] = back_right_enc_count;
}

// this function determines whether all 4 wheels have traveled the required number of indentations (travel_dist) of the encoders
uint8_t suff_dist_traveled(uint32_t travel_dist) {
	if ((front_left_enc_count - wheel_enc_count[0]) >= travel_dist &&
	    (front_right_enc_count - wheel_enc_count[1]) >= travel_dist &&
		(back_left_enc_count - wheel_enc_count[2]) >= travel_dist &&
		(back_right_enc_count - wheel_enc_count[3]) >= travel_dist) {
		return 1;
	}
	else {
		return 0;
	}
}

void move_forward(uint8_t pulse_width) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_inv_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, calc_inv_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_RESET);
}

void move_backward(uint8_t pulse_width) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,calc_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_inv_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_inv_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_SET);
}

void steer_right(uint8_t pulse_width) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_inv_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, calc_inv_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_inv_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_inv_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_SET);
}

void steer_left(uint8_t pulse_width) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, calc_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, calc_pulse_val(&htim2, pulse_width));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calc_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calc_pulse_val(&htim1, pulse_width));
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_RESET);
}

void stop(void) {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, left_in2_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, left_in4_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, right_in2_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, right_in4_Pin, GPIO_PIN_RESET);
}

void transmit_data(uint8_t resend) {
	uint8_t msg_to_remote[50];

	msg_to_remote[0] = 'Q';
	msg_to_remote[1] = 'Z';

	// number of payload bytes
	msg_to_remote[2] = 5;

	// request the remote to resend the data
	if (resend) {
		msg_to_remote[3] = 'R';
		msg_to_remote[4] = 0;
		msg_to_remote[5] = 0;
		msg_to_remote[6] = 0;
		msg_to_remote[7] = 0;
	}
	// otherwise transmit data to the remote as normal
	else {
		msg_to_remote[3] = left2;
		msg_to_remote[4] = left1;
		msg_to_remote[5] = center;
		msg_to_remote[6] = right1;
		msg_to_remote[7] = right2;
	}

	msg_to_remote[8] = 0;
	HAL_UART_Transmit(&huart1, msg_to_remote, 9, HAL_MAX_DELAY);
}

uint8_t process_data(uint8_t *darray, uint8_t start, uint8_t end) {
	if (*(darray + start) == 'R') {
		in_manual_mode = 0;
		start_robot = 0;
		in_tank_mode = 0;
		manual_mode_pulse_width = 50;
		left_joy_x = 50;
		left_joy_y = 50;
		right_joy_x = 50;
		right_joy_y = 50;
		return 0;
	}
	else {
		in_manual_mode = *(darray + start);
		start_robot = *(darray + start + 1);
		in_tank_mode = *(darray + start + 2);
		manual_mode_pulse_width = *(darray + start + 3);
		left_joy_x = *(darray + start + 4);
		left_joy_y = *(darray + start + 5);
		right_joy_x = *(darray + start + 6);
		right_joy_y = *(darray + start + 7);
		return 1;
	}
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
		HAL_UART_DMAStop(&huart1);
		HAL_UART_Receive_DMA(&huart1, msg_from_remote, 12);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	remote_msg_received = 1;
	__HAL_TIM_SET_COUNTER(&htim16, 0);
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
