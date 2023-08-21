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
#include "sevSeg_shift.h"
#include "alarm.h"
#include "ctouch.h"
#include "sinkingClockVars.h"
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*
 * RTC access objects
 */

RTC_TimeTypeDef currTime = {0};
RTC_DateTypeDef currDate = {0};
RTC_TimeTypeDef userAlarmTime = {0};
RTC_AlarmTypeDef userAlarmObj = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/*
 * Call to fetch the current time from the RTC and send to the LED display.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef updateAndDisplayTime(void);

/*
 * Call to fetch the current alarm from the RTC and send to the LED display.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef updateAndDisplayAlarm(void);

/*
 * Called on interrupt from display button to toggle 7-segment intensity.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef displayButtonISR(void);

/*
 * Called on interrupt from alarm enable button to toggle user alarm.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef alarmEnableISR(void);

/*
 * Called on interrupt from alarm set button to enter alarm set loop.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef alarmSetISR(void);

/*
 * Called on interrupt from hour and minute set buttons.
 * Contain logic to set user alarm hour or minute.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef hourSetISR(void);
HAL_StatusTypeDef minuteSetISR(void);

/*
 * Enters loop to signal user alarm
 */
void userAlarmBeep();

/*
 * Map printf to UART output to read messages on terminal
 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  displayToggle = 2; 		// Display at 100% intensity for next display toggle

  // Initialize all GPIOs to be used with 7 segment display
  sevSeg_Init(shiftDataPin, shiftDataClockPin, shiftStoreClockPin,
					shiftOutputEnablePin, shiftMCLRPin,
					GPIOPortArray, &htim16, &htim1);

  	HAL_StatusTypeDef halRet = updateAndDisplayTime();

  	if(halRet != HAL_OK) {		//check HAL
  		printf("HAL Error - TX current time\n\r");
  	} else {
  		printf("Display Updated with current time\n\r");
  	}

  userAlarmToggle = false;			//Default to off

  /* USER CODE END 2 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
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
  sTime.Hours = 1;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 1;
  sAlarm.AlarmTime.Minutes = 1;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_SECONDS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm B
  */
  sAlarm.AlarmTime.Seconds = 10;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  currTime = sTime;
  currDate = sDate;
  userAlarmObj = sAlarm;
  userAlarmTime = userAlarmObj.AlarmTime;

  /* USER CODE END RTC_Init 2 */

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
  htim16.Init.Prescaler = 244;
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
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, Buzzer_Output_Pin|Shift_Store_Clock_Pin|Shift_Data_Clock_Pin|Shift_Master_Clear_Pin
                          |AM_PM_LED_Pin|Alarm_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Shift_Data_In_GPIO_Port, Shift_Data_In_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Display_Button_Pin Alarm_Set_Button_Pin Hour_Set_Button_Pin Alarm_Enable_Button_Pin
                           Minute_Set_Button_Pin */
  GPIO_InitStruct.Pin = Display_Button_Pin|Alarm_Set_Button_Pin|Hour_Set_Button_Pin|Alarm_Enable_Button_Pin
                          |Minute_Set_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Output_Pin Shift_Store_Clock_Pin Shift_Data_Clock_Pin Shift_Master_Clear_Pin
                           AM_PM_LED_Pin Alarm_LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Output_Pin|Shift_Store_Clock_Pin|Shift_Data_Clock_Pin|Shift_Master_Clear_Pin
                          |AM_PM_LED_Pin|Alarm_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Snooze_Button_Pin */
  GPIO_InitStruct.Pin = Snooze_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Snooze_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Shift_Data_In_Pin */
  GPIO_InitStruct.Pin = Shift_Data_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Shift_Data_In_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef updateAndDisplayTime(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	getRTCTime(&hrtc, &currTime, &currDate);
	sevSeg_updateDigits(&currTime);

	if(currTime.TimeFormat == RTC_HOURFORMAT12_PM) {			// If we are in the PM hours
		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_SET);			// Turn on PM LED
	}
	else {
		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_RESET);		// Else, it is A.M.
	}

	return halRet;

}

HAL_StatusTypeDef updateAndDisplayAlarm(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	getUserAlarmTime(&hrtc, &userAlarmTime);
	sevSeg_updateDigits(&userAlarmTime);

	if(userAlarmTime.TimeFormat == RTC_HOURFORMAT12_PM) {			// If we are in the PM hours
		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_SET);			// Turn on PM LED
	}
	else {
		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_RESET);		// Else, it is A.M.
	}

	return halRet;

}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	  printf("Enter current time minute increment interrupt\n\r");

	  RTC_AlarmTypeDef sAlarm;
	  getUserAlarmObj(hrtc, &sAlarm);
	  getRTCTime(hrtc, &currTime, &currDate);

	  if(sAlarm.AlarmTime.Minutes>58) {
		sAlarm.AlarmTime.Minutes=0;
		printf("Reset alarm time\n\r");
	  } else {
		sAlarm.AlarmTime.Minutes=sAlarm.AlarmTime.Minutes+1;
	  }
		while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}

	  updateAndDisplayTime();

	  printf("Current time: %u : %u : %u\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

}

void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc) {

	printf("Enter user alarm interrupt.\n\r");

	if(userAlarmToggle) {			//Only execute sequence if the alarm is actually on

		userAlarmBeep();				// Enter beeping alarm loop
	}

}

void userAlarmBeep() {

	HAL_TIM_Base_Start(&htim16);						// Begin timer 16 counting (to 500 ms)
	uint16_t timerVal = __HAL_TIM_GET_COUNTER(&htim16);	// Get initial timer value to compare to
	bool displayBlink = false;

	do {						// Beep buzzer and blink display until snooze button is pressed

		updateAndDisplayTime();				// Update to current time and display

		if(__HAL_TIM_GET_COUNTER(&htim16) - timerVal >= (65536 / 2)) {		// Use hardware timer to blink/beep display

			sevSeg_setIntensity(&htim1, sevSeg_intensityDuty[displayBlink + 1]);	// Toggle 0% to 50% duty cycle

			HAL_GPIO_TogglePin(GPIOB, buzzerPin);					// Toggle Buzzer

			timerVal = __HAL_TIM_GET_COUNTER(&htim16);				// Update timer value

			displayBlink = !displayBlink;							// Toggle display blink counter

			//printf("Display Blink = %u\n\r", displayBlink);

		}

	} while(capTouchTrigger(snoozeButtonPin));

	HAL_TIM_Base_Stop(&htim16);

}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {

	HAL_StatusTypeDef halRet;					// Flag for printing interrupt status

	if(GPIO_Pin == displayButtonPin) {
		halRet = displayButtonISR();
		if (halRet != HAL_OK) {
			printf("Error toggling display.\n\r");
		} else {
			printf("Display intensity toggled.\n\r");
		}
	}
	else if(GPIO_Pin == alarmEnableButtonPin) {
		halRet = alarmEnableISR();
		if (halRet != HAL_OK) {
			printf("Error toggling user alarm.\n\r");
		} else {
			printf("User alarm toggled.\n\r");
		}
	}
	else if(GPIO_Pin == alarmSetButtonPin) {
		halRet = alarmSetISR();
		if (halRet != HAL_OK) {
			printf("Error setting user alarm.\n\r");
		} else {
			printf("User alarm set.\n\r");
		}
	}
	else if(GPIO_Pin == hourSetButtonPin) {
		halRet = hourSetISR();
		if (halRet != HAL_OK) {
			printf("Error incrementing hour.\n\r");
		} else {
			printf("Hour increment ISR success.\n\r");
		}
	}
	else if(GPIO_Pin == minuteSetButtonPin) {
		halRet = minuteSetISR();
		if (halRet != HAL_OK) {
			printf("Error incrementing minute.\n\r");
		} else {
			printf("Minute increment ISR success.\n\r");
		}
	}
	else {			//Code should never reach here, but do nothing if it does.
		__NOP();
	}

}

HAL_StatusTypeDef displayButtonISR(void) {

	printf("Entered display toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	updateAndDisplayTime();

	sevSeg_setIntensity(&htim1, sevSeg_intensityDuty[displayToggle]);		//Turn display to proper duty cycle

	if(displayToggle >= 2) {			// Increment display toggle or reset back down to 0;
		displayToggle = 0;
		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_RESET);		// If display is off, turn off AM/PM LED
	} else {
		displayToggle++;
	}

	return halRet;				// Return HAL status

}

HAL_StatusTypeDef alarmEnableISR(void) {

	printf("Entered alarm toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	if(!userAlarmToggle) {					// If alarm is disabled, enable it.

		HAL_GPIO_WritePin(GPIOB, alarmLED, GPIO_PIN_SET);			// Turn on alarm LED
		userAlarmToggle = true;								// Toggle internal flag to true

		printf("User alarm set to: %u:%u:%u.\n\r", userAlarmTime.Hours,
								userAlarmTime.Minutes, userAlarmTime.Seconds);

	}
	else if (userAlarmToggle) {				// If alarm is enabled, disable it.

		HAL_GPIO_WritePin(GPIOB, alarmLED, GPIO_PIN_RESET);			// Turn off alarm LED
		userAlarmToggle = false;							// Toggle internal flag to false

		printf("User alarm disabled.\n\r");
	}
	else {
		__NOP();							//Code should never reach here.
	}

	return halRet;

}

HAL_StatusTypeDef alarmSetISR(void) {

	printf("Enter user alarm set ISR.\n\r");

	getUserAlarmTime(&hrtc, &userAlarmTime);
	printf("User alarm currently set to %u:%u:%u.\n\r", userAlarmTime.Hours,
			userAlarmTime.Minutes, userAlarmTime.Seconds);

	HAL_StatusTypeDef halRet = HAL_OK;

	HAL_TIM_Base_Start(&htim16);						// Begin timer 16 counting (to 500 ms)
	uint16_t timerVal = __HAL_TIM_GET_COUNTER(&htim16);	// Get initial timer value to compare to
	bool displayBlink = false;

	do {											// while the alarm set button is not held down, blink display.

		updateAndDisplayAlarm();

		if(__HAL_TIM_GET_COUNTER(&htim16) - timerVal >= (65536 / 2)) {

			sevSeg_setIntensity (&htim1, sevSeg_intensityDuty[displayBlink + 1]);		// Initialize to whatever duty cycle

			timerVal = __HAL_TIM_GET_COUNTER(&htim16);
			displayBlink = !displayBlink;

		}

	}while(HAL_GPIO_ReadPin(GPIOA, alarmSetButtonPin) == GPIO_PIN_RESET);

	sevSeg_setIntensity(&htim1, sevSeg_intensityDuty[0]);			// Turn display back to full intensity

	HAL_TIM_Base_Stop(&htim16);

	updateAndDisplayTime();
	printf("Current time back to %u:%u:%u.\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

	return halRet;

}

HAL_StatusTypeDef hourSetISR(void) {

	printf("Entered hour set ISR.\n\r");

	HAL_StatusTypeDef halRet = HAL_OK;

	if(HAL_GPIO_ReadPin(GPIOA, alarmSetButtonPin) != GPIO_PIN_SET) {	// If the alarm set button is held down, change user alarm time hour

		getUserAlarmTime(&hrtc, &userAlarmTime);

		if(userAlarmTime.Hours >= 12) {
			userAlarmTime.Hours = 1;
			if(userAlarmTime.TimeFormat == RTC_HOURFORMAT12_AM) {
				userAlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;
			} else {
				userAlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
			}
		}
		else if(userAlarmTime.Hours < 12) {
			userAlarmTime.Hours = userAlarmTime.Hours + 1;
		}
		else {
			__NOP();
		}

		userAlarmObj.AlarmTime = userAlarmTime;

		HAL_RTC_SetAlarm_IT(&hrtc, &userAlarmObj, RTCTimeFormat);
		getUserAlarmTime(&hrtc, &userAlarmTime);

		printf("User alarm hour incremented to %u:%u:%u\n\r", userAlarmTime.Hours,
				userAlarmTime.Minutes, userAlarmTime.Seconds);

	}
	else {									// Otherwise, change current time hour.

		getRTCTime(&hrtc, &currTime, &currDate);
		if(currTime.Hours >= 12) {
			currTime.Hours = 1;
			if(currTime.TimeFormat == RTC_HOURFORMAT12_AM) {
				currTime.TimeFormat = RTC_HOURFORMAT12_PM;
			} else {
				currTime.TimeFormat = RTC_HOURFORMAT12_AM;
			}
		}
		else if(currTime.Hours < 12) {
			currTime.Hours = currTime.Hours + 1;
		}
		else {
			__NOP();
		}
		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		printf("Current time hour incremented to %u:%u:%u.\n\r", currTime.Hours,
				currTime.Minutes, currTime.Seconds);
	}

	return halRet;
}

HAL_StatusTypeDef minuteSetISR(void) {

	printf("Entered minute set ISR.\n\r");

	HAL_StatusTypeDef halRet = HAL_OK;

	if(HAL_GPIO_ReadPin(GPIOA, alarmSetButtonPin) == !GPIO_PIN_SET) {	// If the alarm set button is held down, change user alarm time hour

		getUserAlarmTime(&hrtc, &userAlarmTime);

		if(userAlarmTime.Minutes >= 59) {
			userAlarmTime.Minutes = 0;
			userAlarmTime.Hours = userAlarmTime.Hours + 1;
			if(userAlarmTime.Hours > 12) {
				userAlarmTime.Hours = 1;
			}
			if(userAlarmTime.Hours == 12 && userAlarmTime.TimeFormat == RTC_HOURFORMAT12_AM) {
				userAlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;
			} else if(userAlarmTime.Hours == 12 && userAlarmTime.TimeFormat == RTC_HOURFORMAT12_PM) {
				userAlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
			}
			else {
				__NOP();
			}
		}
		else if(userAlarmTime.Minutes < 59) {
			userAlarmTime.Minutes = userAlarmTime.Minutes + 1;
		}
		else {
			__NOP();
		}

		userAlarmObj.AlarmTime = userAlarmTime;

		HAL_RTC_SetAlarm_IT(&hrtc, &userAlarmObj, RTCTimeFormat);

		printf("User alarm minute incremented to %u:%u:%u\n\r", userAlarmObj.AlarmTime.Hours,
				userAlarmObj.AlarmTime.Minutes, userAlarmObj.AlarmTime.Seconds);

	}
	else {									// Otherwise, change current time hour.

		getRTCTime(&hrtc, &currTime, &currDate);

		if(currTime.Minutes >= 59) {
			currTime.Minutes = 0;
			currTime.Hours = currTime.Hours + 1;
			if(currTime.Hours > 12) {
				currTime.Hours = 1;
			}
			if(currTime.Hours == 12 && currTime.TimeFormat == RTC_HOURFORMAT12_AM) {
				currTime.TimeFormat = RTC_HOURFORMAT12_PM;
			} else if(currTime.Hours == 12 && currTime.TimeFormat == RTC_HOURFORMAT12_PM) {
				currTime.TimeFormat = RTC_HOURFORMAT12_AM;
			}
			else {
				__NOP();
			}
		}
		else if(currTime.Minutes < 59) {
			currTime.Minutes = currTime.Minutes + 1;
		}
		else {
			__NOP();
		}
		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		printf("Current time minute incremented to %u:%u:%u.\n\r", currTime.Hours,
				currTime.Minutes, currTime.Seconds);
	}

	return halRet;
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
