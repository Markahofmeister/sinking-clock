/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "sinkingClockVars.h"
#include "AT42QT1070.h"

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

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

TIM_HandleTypeDef *timerDelay = &htim14;
TIM_HandleTypeDef *timerPWM = &htim2;
uint32_t tim_PWM_CHANNEL = TIM_CHANNEL_3;

/*
 * RTC access objects
 */

RTC_TimeTypeDef currTime = {0};
RTC_DateTypeDef currDate = {0};
RTC_AlarmTypeDef userAlarmObj = {0};
RTC_TimeTypeDef userAlarmTime = {0};

/*
 * State bools
 */
bool alarmSetMode = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM2_Init(void);
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
 * Hour and minute incrementing functions for alarm time and current time
 *
 */

void alarmHourInc(void);
void currHourInc(void);
void alarmMinuteInc(void);
void currMinuteInc(void);

/*
 * Enters loop to signal user alarm
 */
void userAlarmBeep();

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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Initialize all GPIOs to be used with 7 segment display
    sevSeg_Init(shiftDataPin, shiftDataClockPin, shiftStoreClockPin,
				shiftOutputEnablePin, shiftMCLRPin,
				GPIOPortArray, timerDelay, timerPWM, tim_PWM_CHANNEL);

	HAL_StatusTypeDef halRet = updateAndDisplayTime();

	if(halRet != HAL_OK) {		//check HAL
		//printf("HAL Error - TX current time\n\r");
	} else {
		//printf("Display Updated with current time\n\r");
	}

    /*
     * Initialize capacitive touch sensor
     */
    QT1070 capTouch;
    halRet = capTouch_Init(&capTouch, &hi2c1, 0b00001111);
    uint8_t avgFactors_New[7] = {32, 32, 32, 32, 0, 0, 0};
    halRet = capTouch_SetAveragingFactor(&capTouch, avgFactors_New);

	userAlarmToggle = false;			//Default to off

    // User alarm default value
    userAlarmTime.Hours = 1;
    userAlarmTime.Minutes = 1;
    userAlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;

    uint16_t count = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {

	  uint8_t channelTest = 0x00;
	  halRet = capTouch_readChannels(&capTouch, &channelTest);
	  if(channelTest != 0x00) {
		  HAL_GPIO_TogglePin(debugLEDPort, debugLEDPin);
		  count++;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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
  hi2c1.Init.Timing = 0x00602173;
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
  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
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

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x1;
  sAlarm.AlarmTime.Minutes = 0x1;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_SECONDS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibration
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_512HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 244*4;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEBUG_LED_Pin|BUZZER_OUT_Pin|SHIFT_DATA_IN_Pin|SHIFT_DATA_CLK_Pin
                          |SHIFT_MCLR_Pin|ALARM_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHIFT_STORE_CLK_GPIO_Port, SHIFT_STORE_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTOUCH_EN_GPIO_Port, CTOUCH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_Pin BUZZER_OUT_Pin SHIFT_DATA_IN_Pin SHIFT_DATA_CLK_Pin
                           SHIFT_MCLR_Pin ALARM_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin|BUZZER_OUT_Pin|SHIFT_DATA_IN_Pin|SHIFT_DATA_CLK_Pin
                          |SHIFT_MCLR_Pin|ALARM_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SHIFT_STORE_CLK_Pin */
  GPIO_InitStruct.Pin = SHIFT_STORE_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHIFT_STORE_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MINUTE_SET_BUTTON_Pin */
  GPIO_InitStruct.Pin = MINUTE_SET_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MINUTE_SET_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOUR_SET_BUTTON_Pin ALARM_SET_BUTTON_Pin ALARM_EN_BUTTON_Pin DISPLAY_BUTTON_Pin */
  GPIO_InitStruct.Pin = HOUR_SET_BUTTON_Pin|ALARM_SET_BUTTON_Pin|ALARM_EN_BUTTON_Pin|DISPLAY_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CTOUCH_EN_Pin */
  GPIO_InitStruct.Pin = CTOUCH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CTOUCH_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef updateAndDisplayTime(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	getRTCTime(&hrtc, &currTime, &currDate);
	sevSeg_updateDigits(&currTime);

	return halRet;

}

HAL_StatusTypeDef updateAndDisplayAlarm(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	sevSeg_updateDigits(&userAlarmTime);

	return halRet;

}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	  //printf("Enter current time minute increment interrupt\n\r");

	  RTC_AlarmTypeDef sAlarm;
	  getRTCTime(hrtc, &currTime, &currDate);

	  if(sAlarm.AlarmTime.Minutes>58) {
		sAlarm.AlarmTime.Minutes=0;
		//printf("Reset alarm time\n\r");
	  } else {
		sAlarm.AlarmTime.Minutes=sAlarm.AlarmTime.Minutes+1;
	  }
		while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}

	  updateAndDisplayTime();

	  //printf("Current time: %u : %u : %u\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

	  // If alarm is enabled and current time matches user alarm time, set off the alarm.
	  if(userAlarmToggle && userAlarmTime.Hours == currTime.Hours
			  && userAlarmTime.Minutes == currTime.Minutes && userAlarmTime.TimeFormat == currTime.TimeFormat) {
		  userAlarmBeep();
	  }

}

void userAlarmBeep() {

	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 500 ms)
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to
	bool displayBlink = false;

	uint8_t i = 0;

	do {						// Beep buzzer and blink display until snooze button is pressed

		updateAndDisplayTime();				// Update to current time and display

		if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65535 / 2)) {		// Use hardware timer to blink/beep display

			sevSeg_setIntensity(timerPWM, tim_PWM_CHANNEL, sevSeg_intensityDuty[displayBlink]);	// Toggle 0% to 50% duty cycle

			HAL_GPIO_TogglePin(buzzerPort, buzzerPin);					// Toggle Buzzer

			timerVal = __HAL_TIM_GET_COUNTER(timerDelay);				// Update timer value

			displayBlink = !displayBlink;							// Toggle display blink counter

			//printf("Display Blink = %u\n\r", displayBlink);

		}

		i++;		// Get rid of. This is just for testing.

//	} while(capTouchTrigger(snoozeButtonPin));
	} while(i < 5);

	HAL_TIM_Base_Stop(timerDelay);

}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {

	HAL_StatusTypeDef halRet;					// Flag for printing interrupt status

	if(GPIO_Pin == displayButtonPin) {
		halRet = displayButtonISR();
		if (halRet != HAL_OK) {
			//printf("Error toggling display.\n\r");
		} else {
			//printf("Display intensity toggled.\n\r");
		}
	}
	else if(GPIO_Pin == alarmEnableButtonPin) {
		halRet = alarmEnableISR();
		if (halRet != HAL_OK) {
			//printf("Error toggling user alarm.\n\r");
		} else {
			//printf("User alarm toggled.\n\r");
		}
	}
	else if(GPIO_Pin == alarmSetButtonPin) {
		halRet = alarmSetISR();
		if (halRet != HAL_OK) {
			//printf("Error setting user alarm.\n\r");
		} else {
			//printf("User alarm set.\n\r");
		}
	}
	else if(GPIO_Pin == hourSetButtonPin) {
		halRet = hourSetISR();
		if (halRet != HAL_OK) {
			//printf("Error incrementing hour.\n\r");
		} else {
			//printf("Hour increment ISR success.\n\r");
		}
	}
	else if(GPIO_Pin == minuteSetButtonPin) {
		halRet = minuteSetISR();
		if (halRet != HAL_OK) {
			//printf("Error incrementing minute.\n\r");
		} else {
			//printf("Minute increment ISR success.\n\r");
		}
	}
	else {			//Code should never reach here, but do nothing if it does.
		__NOP();
	}

}

HAL_StatusTypeDef displayButtonISR(void) {

	//printf("Entered display toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	updateAndDisplayTime();

	sevSeg_setIntensity(timerPWM, tim_PWM_CHANNEL, sevSeg_intensityDuty[displayToggle]);		//Turn display to proper duty cycle

	if(displayToggle >= 2) {			// Increment display toggle or reset back down to 0;
		displayToggle = 0;
//		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_RESET);		// If display is off, turn off AM/PM LED
	} else {
		displayToggle++;
	}

	return halRet;				// Return HAL status

}

HAL_StatusTypeDef alarmEnableISR(void) {

	//printf("Entered alarm toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	if(!userAlarmToggle) {					// If alarm is disabled, enable it.

		HAL_GPIO_WritePin(alarmLEDPort, alarmLEDPin, GPIO_PIN_SET);			// Turn on alarm LED
		userAlarmToggle = true;								// Toggle internal flag to true

		//printf("User alarm set to: %u:%u:%u.\n\r", userAlarmTime.Hours,
								//userAlarmTime.Minutes, userAlarmTime.Seconds);

	}
	else if (userAlarmToggle) {				// If alarm is enabled, disable it.

		HAL_GPIO_WritePin(alarmLEDPort, alarmLEDPin, GPIO_PIN_RESET);			// Turn off alarm LED
		userAlarmToggle = false;							// Toggle internal flag to false

		//printf("User alarm disabled.\n\r");
	}
	else {
		__NOP();							//Code should never reach here.
	}

	return halRet;

}

HAL_StatusTypeDef alarmSetISR(void) {

	//printf("Enter user alarm set ISR.\n\r");

	//printf("User alarm currently set to %u:%u:%u.\n\r", userAlarmTime.Hours,
			//userAlarmTime.Minutes, userAlarmTime.Seconds);

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 * Wait for switch debounce
	 */

	// First wait for button to deactivate again
	while(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) != GPIO_PIN_SET);

	// Go through debounce
	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

	do {

	}while(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal <= (65536 / 8));


	/*
	 *  Poll for 1 second to see if the alarm set button is pressed again
	 */
	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

	while(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal <= (65536)) {

		if(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) == GPIO_PIN_RESET) {
			alarmSetMode = true;
//			HAL_GPIO_WritePin(debugLEDPort, debugLEDPin, GPIO_PIN_SET);
			break;
		}

	}

	// Go through debounce once again
	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

	do {

	}while(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal <= (65536 / 4));

	/*
	 * Then, if we are in alarm set mode, go through the
	 * alarm set process until the button is pressed again
	 */

	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

	if(alarmSetMode) {

		bool displayBlink = false;

		do {											// while the alarm set button is not held down, blink display.

			updateAndDisplayAlarm();

			if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65536 / 2)) {

//				HAL_GPIO_TogglePin(debugLEDPort, debugLEDPin);

				sevSeg_setIntensity(timerPWM, tim_PWM_CHANNEL, sevSeg_intensityDuty[displayBlink]);		// Initialize to whatever duty cycle

				timerVal = __HAL_TIM_GET_COUNTER(timerDelay);
				displayBlink = !displayBlink;

			}

		}while(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) != GPIO_PIN_RESET);

//		HAL_GPIO_WritePin(debugLEDPort, debugLEDPin, GPIO_PIN_RESET);

		sevSeg_setIntensity(timerPWM, tim_PWM_CHANNEL, sevSeg_intensityDuty[1]);			// Turn display back to 50% intensity

		HAL_TIM_Base_Stop(timerDelay);

		updateAndDisplayTime();

	}

	alarmSetMode = false;		// We have exited alarm set mode

	//printf("Current time back to %u:%u:%u.\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

	return halRet;

}

HAL_StatusTypeDef hourSetISR(void) {

//	printf("Entered hour set ISR.\n\r");
//	HAL_GPIO_TogglePin(debugLEDPort, debugLEDPin);


	HAL_StatusTypeDef halRet = HAL_OK;

	if(alarmSetMode) {	// If the clock is in alarm set mode, change user alarm time hour

		alarmHourInc();

		//printf("User alarm hour incremented to %u:%u:%u\n\r", userAlarmTime.Hours,
				//userAlarmTime.Minutes, userAlarmTime.Seconds);

	}
	else {									// Otherwise, change current time hour.

		currHourInc();

		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		//printf("Current time hour incremented to %u:%u:%u.\n\r", currTime.Hours,
				//currTime.Minutes, currTime.Seconds);
	}

	return halRet;

}

HAL_StatusTypeDef minuteSetISR(void) {

//	printf("Entered minute set ISR.\n\r");
//	HAL_GPIO_TogglePin(debugLEDPort, debugLEDPin);

	HAL_StatusTypeDef halRet = HAL_OK;

	if(alarmSetMode) {	// If the clock is in alarm set mode, change user alarm time hour

		alarmMinuteInc();

		//printf("User alarm minute incremented to %u:%u:%u\n\r", userAlarmTime.Hours,
				//userAlarmTime.Minutes, userAlarmTime.Seconds);

	}
	else {									// Otherwise, change current time hour.

		currMinuteInc();

		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		//printf("Current time minute incremented to %u:%u:%u.\n\r", currTime.Hours,
				//currTime.Minutes, currTime.Seconds);
	}

	return halRet;
}

void alarmHourInc(void) {

	if(userAlarmTime.Hours >= 12) {
		userAlarmTime.Hours = 1;
	}
	else if(userAlarmTime.Hours == 11) {
		if(userAlarmTime.TimeFormat == RTC_HOURFORMAT12_AM) {
			userAlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;
		}
		else {
			userAlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
		}
		userAlarmTime.Hours = 12;
	}
	else if(userAlarmTime.Hours < 11) {
		userAlarmTime.Hours = userAlarmTime.Hours + 1;
	}
	else {
		__NOP();
	}

}

void currHourInc(void) {

	getRTCTime(&hrtc, &currTime, &currDate);

	if(currTime.Hours >= 12) {
		currTime.Hours = 1;
	}
	else if(currTime.Hours == 11) {
		if(currTime.TimeFormat == RTC_HOURFORMAT12_AM) {
			currTime.TimeFormat = RTC_HOURFORMAT12_PM;
		}
		else {
			currTime.TimeFormat = RTC_HOURFORMAT12_AM;
		}
		currTime.Hours = 12;
	}
	else if(userAlarmTime.Hours < 11) {
		currTime.Hours = currTime.Hours + 1;
	}
	else {
		__NOP();
	}

}

void alarmMinuteInc(void) {

	if(userAlarmTime.Minutes >= 59) {
		alarmHourInc();
		userAlarmTime.Minutes = 0;
	}
	else if(userAlarmTime.Minutes < 59) {
		userAlarmTime.Minutes = userAlarmTime.Minutes + 1;
	}
	else {
		__NOP();
	}

}

void currMinuteInc(void) {

	getRTCTime(&hrtc, &currTime, &currDate);

	if(currTime.Minutes >= 59) {
		currHourInc();
		currTime.Minutes = 0;
	}
	else if(currTime.Minutes < 59) {
		currTime.Minutes = currTime.Minutes + 1;
	}
	else {
		__NOP();
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
