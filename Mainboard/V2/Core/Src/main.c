/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct Command { // (1)
	int8_t motor1;
	int8_t motor2;
	int8_t motor3;
	int32_t thrower;
	int32_t servo;
	int32_t ir;
//	float pGain;
//	float iGain;
//	float dGain;
	int32_t pid_type;
	uint8_t delimiter;
	int8_t speed1;
	int8_t speed2;
	int8_t speed3;
	int8_t throwerSpeed;

} Command;


Command command, feedback;
volatile uint8_t command_received = 0;
volatile uint8_t command_received_ticker = 0;

Motor motor1;
Motor motor2;
Motor motor3;


typedef struct Feedback { // (3)
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t delimiter;
} Feedback;

typedef struct Received{
	uint8_t speed1;
	uint8_t speed2;
	uint8_t speed3;
	uint8_t throwerSpeed;
	uint8_t servospeed;
	uint8_t delimiter;
} Received;

Received received = {.speed1 = 125, .speed2 = 125, .speed3 = 125, .throwerSpeed = 0,.servospeed = 0, .delimiter = 0}; // (4)
volatile uint8_t isCommandReceived = 0; // (5)
volatile float pGain, iGain, dGain = 0;


void CDC_On_Receive(uint8_t* buffer, uint32_t* length) {

  if (*length == sizeof(Received)) {
	  memcpy(&received, buffer, sizeof(Received));

	 if (received.delimiter == 0xAA) { // (9)
		  isCommandReceived = 1;
    }
  }
}


void Set_Motor_Speed(volatile uint32_t *channel_a, //copy paste
		volatile uint32_t *channel_b, int32_t motor_speed) {
	if (motor_speed > 0) {
		// forward
		if (motor_speed <= MOTORS_MAX_SPEED) {
			// 0 to 100 compact range
			*channel_a = motor_speed * MOTORS_CO + MOTORS_MIN_CCR;
		} else {
			// 100 to 65535 full range
			*channel_a = motor_speed;
		}
		*channel_b = 0;
	} else if (motor_speed < 0) {
		// backward
		if (motor_speed >= -MOTORS_MAX_SPEED) {
			// -100 to 0 compact range
			*channel_b = motor_speed * -MOTORS_CO + MOTORS_MIN_CCR;
		} else {
			// -65535 to -100 full range
			*channel_b = motor_speed * -1;
		}
		*channel_a = 0;
	} else {
		// stop
		*channel_a = 0;
		*channel_b = 0;
	}
}


int PID2(uint8_t enc, uint8_t setpoint){


	enc = 250 - enc;

	float P = 0.4;
	float I = 0.00001;
	float D = 0;

	static long integral = 0;

	if(setpoint == 0){
		integral = 0;
		return 0;
	}


	//CDC_Transmit_FS(&enc, sizeof(enc));

	float error = setpoint - enc;

	integral += error;

	int out = 50 + P*error + I*integral;

	if(out < 0){
		out = 0;
	}


	if(out > 100) out = 100;

	//CDC_Transmit_FS(&out, sizeof(out));
	return (uint8_t)out;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


    /* Initialize Rotary encoder 2, pin A = PF1, pin B = PA2 */


    /* Rotate LCD */

    /* Initialize Rotary encoder 1, pin A = PF1, pin B = PD1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    unsigned long esctimer = DWT->CYCCNT;
    unsigned long servotimer = DWT->CYCCNT;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
 // HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  	TIM1->CCR1 = 0;
  	TIM1->CCR2 = 0;


  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  	TIM3->CCR1 = 0;
  	TIM3->CCR2 = 0;


  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  	TIM4->CCR1 = 0;
  	TIM4->CCR2 = 0;


typedef struct Feedback{
	int16_t speed1;
	int16_t speed2;
	int16_t speed3;
	int16_t delmiter;
} Feedback;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int ENC2 = 0;
	int prev_ENC2 = 0;
	unsigned long ENCA_S = DWT->CYCCNT;
	unsigned long ENCA_D;
	uint8_t ENCA_P = 0;

	unsigned long ENCA_S1 = DWT->CYCCNT;
	unsigned long ENCA_D1;
	uint8_t ENCA_P3 = 0;

	unsigned long ENCA_S3 = DWT->CYCCNT;
	unsigned long ENCA_D3;
	uint8_t ENCA_P2 = 0;
  while (1)
  {


	  //ESC
	  if((DWT->CYCCNT - esctimer)/96000 > 20){
		  esctimer = DWT->CYCCNT;
		  HAL_GPIO_WritePin(ESCPWM_GPIO_Port, ESCPWM_Pin,1);
		  while((DWT->CYCCNT - esctimer) / 96 < (received.throwerSpeed*3.92+1000));
		  HAL_GPIO_WritePin(ESCPWM_GPIO_Port, ESCPWM_Pin,0);
	  }



	  //SERVO
	  if((DWT->CYCCNT - servotimer)/96000 > 2){
		  servotimer = DWT->CYCCNT;
		  HAL_GPIO_WritePin(SERVOPWM_GPIO_Port, SERVOPWM_Pin,1);
		  while((DWT->CYCCNT - servotimer) / 96 < (received.servospeed*23));
		  HAL_GPIO_WritePin(SERVOPWM_GPIO_Port, SERVOPWM_Pin,0);
	  }







	  	  unsigned long while_timeout = DWT->CYCCNT;
	  	  int timeout_flag = 0;
		  while(HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin) == 1){  // ootab kuni läheb madalaks
			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
				  timeout_flag = 1;
				  break;
			  }
		  }
		  while_timeout = DWT->CYCCNT;
		  while(HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin) == 0){ //ootab kuni läheb kõrgeks
			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
				  timeout_flag = 1;
				  break;
			  }
		  }
		  while_timeout = DWT->CYCCNT;
		  ENCA_S = DWT->CYCCNT;
		  while(HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin) == 1){ // ootab kuni läheb madalaks
			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
				  timeout_flag = 1;
				  break;
			  }
		  }

		  ENCA_D = ((DWT->CYCCNT - ENCA_S) / 2000);
		  if(ENCA_D > 249 || timeout_flag == 1) ENCA_D = 249;
		  ENCA_P = (uint8_t)ENCA_D;





		  while_timeout = DWT->CYCCNT;
		  timeout_flag = 0;
		  while(HAL_GPIO_ReadPin(ENC1A_GPIO_Port, ENC1A_Pin) == 1){  // ootab kuni läheb madalaks
			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
				  timeout_flag = 1;
				  break;
			  }
		  }
		  while_timeout = DWT->CYCCNT;
		  while(HAL_GPIO_ReadPin(ENC1A_GPIO_Port, ENC1A_Pin) == 0){ //ootab kuni läheb kõrgeks
			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
				  timeout_flag = 1;
				  break;
			  }
		  }
		  while_timeout = DWT->CYCCNT;
		  ENCA_S = DWT->CYCCNT;
		  while(HAL_GPIO_ReadPin(ENC1A_GPIO_Port, ENC1A_Pin) == 1){ // ootab kuni läheb madalaks
			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
				  timeout_flag = 1;
				  break;
			  }
		  }

		  ENCA_D = ((DWT->CYCCNT - ENCA_S) / 2000);
		  if(ENCA_D > 249 || timeout_flag == 1) ENCA_D = 249;
		  ENCA_P3 = (uint8_t)ENCA_D;







		  while_timeout = DWT->CYCCNT;
  	  	  timeout_flag = 0;
  		  while(HAL_GPIO_ReadPin(ENC3A_GPIO_Port, ENC3A_Pin) == 1){  // ootab kuni läheb madalaks
  			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
  				  timeout_flag = 1;
  				  break;
  			  }
  		  }
  		  while_timeout = DWT->CYCCNT;
  		  while(HAL_GPIO_ReadPin(ENC3A_GPIO_Port, ENC3A_Pin) == 0){ //ootab kuni läheb kõrgeks
  			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
  				  timeout_flag = 1;
  				  break;
  			  }
  		  }
  		  while_timeout = DWT->CYCCNT;
  		  ENCA_S = DWT->CYCCNT;
  		  while(HAL_GPIO_ReadPin(ENC3A_GPIO_Port, ENC3A_Pin) == 1){ // ootab kuni läheb madalaks
  			  if((DWT->CYCCNT - while_timeout)/2000 > 249){
  				  timeout_flag = 1;
  				  break;
  			  }
  		  }

		  ENCA_D = ((DWT->CYCCNT - ENCA_S) / 2000);
		  if(ENCA_D > 249 || timeout_flag == 1) ENCA_D = 249;
		  ENCA_P2 = (uint8_t)ENCA_D;






		if(received.speed1 > 125){
			uint8_t pi1d = PID2(ENCA_P3, (received.speed1 - 125)*2);
			Set_Motor_Speed(&(TIM4->CCR1), &(TIM4->CCR2), pi1d);
		}
		else if(received.speed1 < 125){
			uint8_t pi1d = PID2(ENCA_P3, 250 - (received.speed1*2));
			Set_Motor_Speed(&(TIM4->CCR1), &(TIM4->CCR2), pi1d*-1);
		}
		else{
			Set_Motor_Speed(&(TIM4->CCR1), &(TIM4->CCR2), 0);
		}



		if(received.speed2 > 125){
			uint8_t pi2d = PID2(ENCA_P, (received.speed2 - 125)*2);
			Set_Motor_Speed(&(TIM1->CCR1), &(TIM1->CCR2), pi2d);
		}
		else if(received.speed2 < 125){
			uint8_t pi2d = PID2(ENCA_P, 250 - (received.speed2*2));
			Set_Motor_Speed(&(TIM1->CCR1), &(TIM1->CCR2), pi2d*-1);
		}
		else{
			Set_Motor_Speed(&(TIM1->CCR1), &(TIM1->CCR2), 0);
		}





		if(received.speed3 > 125){
			uint8_t pi3d = PID2(ENCA_P2, (received.speed3 - 125)*2);
			Set_Motor_Speed(&(TIM3->CCR1), &(TIM3->CCR2), pi3d);
		}
		else if(received.speed3 < 125){
			uint8_t pi3d = PID2(ENCA_P2, 250 - (received.speed3*2));
			Set_Motor_Speed(&(TIM3->CCR1), &(TIM3->CCR2), pi3d*-1);
		}
		else{
			Set_Motor_Speed(&(TIM3->CCR1), &(TIM3->CCR2), 0);
		}






	    //CDC_Transmit_FS(&ENCA_P, sizeof(ENCA_P));



	//	uint8_t pi3d = PID2(ENCA_P3, received.speed3);
	//  CDC_Transmit_FS(&pid, sizeof(pid));
	// 	Set_Motor_Speed(&(TIM4->CCR1), &(TIM4->CCR2), pi3d);


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ESCPWM_Pin|SERVOPWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENC3A_Pin */
  GPIO_InitStruct.Pin = ENC3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC3A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC2_A_Pin ENC1A_Pin */
  GPIO_InitStruct.Pin = ENC2_A_Pin|ENC1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1B_Pin ENC3B_Pin */
  GPIO_InitStruct.Pin = ENC1B_Pin|ENC3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ESCPWM_Pin SERVOPWM_Pin */
  GPIO_InitStruct.Pin = ESCPWM_Pin|SERVOPWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	unsigned long t1 = DWT->CYCCNT;

	//Handle_Encoder(&motor1, TIM2->CNT);
	//Handle_Encoder(&motor2, TIM2->CNT);

	//Handle_Encoder(&motor3, TIM4->CNT);
	if (command.pid_type == 0) {
	//	feedback.motor1 = motor1.cur_enc_speed;
		//received.speed2 = motor2.cur_enc_speed;

		//feedback.motor3 = motor3.cur_enc_speed;
	} else {
	//	feedback.motor1 = motor1.enc_speed_hist_avg;
		//received.speed2 = motor2.enc_speed_hist_avg;
	//	feedback.motor3 = motor3.enc_speed_hist_avg;
	}

	//Calculate_PID(&motor1);
//	Calculate_PID(&motor2);
	//Calculate_PID(&motor3);
	//Set_Motor_Speed(&(TIM1->CCR1), &(TIM1->CCR2), motor1.cur_speed);//motor1.cur_speed
	//Set_Motor_Speed(&(TIM3->CCR1), &(TIM3->CCR2), motor2.cur_speed);
	//Set_Motor_Speed(&(TIM4->CCR1), &(TIM4->CCR2), motor3.cur_speed);

	// servo stopper
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) && !command.ir) {
//		TIM17->CCR1 = 0;
//s	}

	// timeout
	if (command_received_ticker > 0) {
		command_received_ticker -= 1;
	} else {
		// stop wheels
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM4->CCR1 = 0;
		TIM4->CCR2 = 0;


	}
//	unsigned long t2 = DWT->CYCCNT;
//	unsigned long diff = t2 - t1;
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
