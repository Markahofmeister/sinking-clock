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

#ifndef SEVSEG_SHIFT_H_
#include "sevSeg_shift.h"
#endif

#ifndef ALARM_H_
#include "alarm.h"
#endif

#ifndef SINKINGCLOCKVARS_H_
#include "sinkingClockVars.h"
#endif

#ifndef AT42QT1070_H_
#include "AT42QT1070.h"
#endif

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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/*
 * Timer to be used for PWMing display
 */
TIM_HandleTypeDef *timerPWM = &htim2;

/*
 * Timer to be used for non-blocking delays
 */
TIM_HandleTypeDef *timerDelay = &htim14;

/*
 * Timer to be used for long 10-minute snooze
 */
TIM_HandleTypeDef *timerSnooze = &htim16;

/*
 * RCR value for long 10-minute snooze
 */
const uint32_t timerSnooze_RCR = 100;
uint8_t snoozeCounter = 0;

/*
 * State bools
 */

// Used to blare alarm if enabled
bool alarmSetMode = false;

/*
 * Used to indicate whether the clock is in a
 * state to sound a snoozed alarm again after 10 minutes.
 * false = regular operation, true = 10-minute snooze period
 */
bool secondSnooze = false;

/*
 * Cap. touch struct
 */
QT1070 capTouch;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
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
void userAlarmBeep(void);

/*
 * Displays non-critical error on debug LED and continues functionality
 */
void dispFault();

/*
 * Displays Critical Error on clock and halts all functionality
 */
void dispFailure();

/*
 * Updates RTC Backup Registers with user alarm time
 */
void updateRTCBackupReg(void);

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // HAL Status handle for error-checking
  HAL_StatusTypeDef halRet = HAL_OK;

  // Set Smooth Calibration Value
  halRet = HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_8SEC,
    							RTC_SMOOTHCALIB_PLUSPULSES_RESET, rtcCalVal);
  if(halRet != HAL_OK) {
	  // Failure to talk to RTC is a hard failure
	  dispFailure();
  }

  // Init the internal RTC alarm time to track the current time
  halRet = initRTCInternalAlarm(&hrtc, &currTime, &currDate);
  if(halRet != HAL_OK) {
  	  // Failure to initialize RTC alarm is a hard failure
  	  dispFailure();
    }

  // Initialize all GPIOs to be used with 7 segment display
    sevSeg_Init(shiftDataPin, shiftDataClockPin, shiftStoreClockPin,
				shiftOutputEnablePin, shiftMCLRPin,
				GPIOPortArray, timerDelay, timerPWM, tim_PWM_CHANNEL);

    // Set to max brightness
    sevSeg_setIntensity(sevSeg_intensityDuty[2]);

	halRet = updateAndDisplayTime();
	if(halRet != HAL_OK) {
	  // Failure to display current time is a hard failure
	  dispFailure();
	}

    /*
     * Initialize capacitive touch sensor
     */

	// Used to separate return initializations into critical and non-critical errors.
	uint8_t initRet = 0;

    initRet = capTouch_Init(&capTouch, &hi2c1, timerDelay,
    						&capTouchResetPort, capTouchResetPin, capTouchChannels);
    if( (initRet == 1) || (initRet == 3) || (initRet == 4)) {

    	/* Critical Errors:
    	 * 1 = Failure to read correct device ID
    	 * 2 = Failure to read Keys
    	 * 3 = Failure to enable keys
    	 */
    	dispFailure();
    }
    else if (initRet == 2) {
    	/*
    	 * Non-critical Errors:
    	 * 2 = Failure to Recalibrate
    	 */
    	dispFault();
    }
    else if(initRet == 0) {
    	// initRet = 0 = all is well
    	__NOP();
    }

    // Set averaging factor
    uint8_t avgFactors_New[7] = {AVGFact, AVGFact, AVGFact, AVGFact, 0, 0, 0};
    halRet = capTouch_SetAveragingFactor(&capTouch, avgFactors_New);

    if(halRet != HAL_OK) {
    	// This is sensitivity-setting and a non-critical error
    	dispFault();
    }

    // Set detection integration factors
    uint8_t detIntFactors_New[7] = {DIFact, DIFact, DIFact, DIFact, DIFact, DIFact, DIFact};
    halRet = capTouch_SetDetectionIntegrator(&capTouch, detIntFactors_New);
    if(halRet != HAL_OK) {
    	// This is sensitivity-setting and a non-critical error
		dispFault();
    }

    userAlarmToggle = false;			//Default to off


    /*
     * If the bootstrap backup register reads 0x00 (never written to,)
     * initialize the alarm time to a default value.
     *
     * Else, initialize to whatever is stored in backup registers.
     */

    if((uint8_t)HAL_RTCEx_BKUPRead(&hrtc, bootstrapBackupReg) == 0) {

    	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmHourBackupReg, 0x01);
    	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmMinuteBackupReg, 0x00);
    	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmTFBackupReg, RTC_HOURFORMAT12_AM);

    	// Write backup register with a non-zero value to signify that it has been initialized before
    	HAL_RTCEx_BKUPWrite(&hrtc, bootstrapBackupReg, 0xFFFFFFFF);

    }

	userAlarmTime.Hours = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, userAlarmHourBackupReg);
	userAlarmTime.Minutes = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, userAlarmMinuteBackupReg);
	userAlarmTime.TimeFormat = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, userAlarmTFBackupReg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSI);
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
//  sTime.Hours = 0x1;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.SubSeconds = 0x0;
//  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }


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

  // Do not initialize time - pull from whatever is in register
    HAL_RTC_GetTime(&hrtc, &currTime, RTCTimeFormat);

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
  htim16.Init.Prescaler = (58595 / 10) - 1;
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

  // Decrease interrupt priority
  HAL_NVIC_SetPriority(TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM16_IRQn);

  // Clear SR interrupts
  __HAL_TIM_CLEAR_IT(timerSnooze, TIM_IT_UPDATE);
//
//  // Re-write RCR with 10
//	timerSnooze->Instance->RCR &= 0xFF00;
//	timerSnooze->Instance->RCR |= timerSnooze_RCR;


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
  HAL_GPIO_WritePin(CTOUCH_RST_GPIO_Port, CTOUCH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_Pin BUZZER_OUT_Pin SHIFT_DATA_IN_Pin SHIFT_DATA_CLK_Pin
                           SHIFT_MCLR_Pin ALARM_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin|BUZZER_OUT_Pin|SHIFT_DATA_IN_Pin|SHIFT_DATA_CLK_Pin
                          |SHIFT_MCLR_Pin|ALARM_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

  /*Configure GPIO pin : CTOUCH_RST_Pin */
  GPIO_InitStruct.Pin = CTOUCH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CTOUCH_RST_GPIO_Port, &GPIO_InitStruct);

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

/*
 * Pulls updated time from RTC and send new time to user display
 */
HAL_StatusTypeDef updateAndDisplayTime(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	getRTCTime(&hrtc, &currTime, &currDate);
	sevSeg_updateDigits(&currTime);

	return halRet;

}

/*
 * Sends current user alarm time to user display
 * This doesn't pull from the RTC because the RTC alarm is not being used for that
 */

HAL_StatusTypeDef updateAndDisplayAlarm(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	sevSeg_updateDigits(&userAlarmTime);

	return halRet;

}

/*
 * Interrupt callback function for RTC interrupt kickback
 * Occurs every minute increment
 *
 * Pulls alarm time from RTC, increments and sets new alarm time, and updates time.
 * Sets off user alarm if the alarm is enabled.
 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	  RTC_AlarmTypeDef sAlarm = {0};
	  HAL_RTC_GetAlarm(hrtc, &sAlarm, internalAlarm, RTCTimeFormat);

	  getRTCTime(hrtc, &currTime, &currDate);

	  if(currTime.Minutes > 58) {
		sAlarm.AlarmTime.Minutes = 0;
	  } else {
		sAlarm.AlarmTime.Minutes = currTime.Minutes + 1;
	  }
		while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){

			__NOP();

		}

	  updateAndDisplayTime();

	  // If alarm is enabled and current time matches user alarm time, set off the alarm.
	  if(userAlarmToggle && userAlarmTime.Hours == currTime.Hours
			  && userAlarmTime.Minutes == currTime.Minutes && userAlarmTime.TimeFormat == currTime.TimeFormat) {
		  userAlarmBeep();
	  }


}

/*
 * If user alarm is enabled and the RTC time equals the user alarm time, this function is entered.
 *
 * This function blinks the display LEDs and toggles the beeper until the user
 * disables this beeping through cap. touch or the alarm enable button.
 *
 * Functionality depends on whether or not this is the first or second snooze.
 * 		First Snooze: 10-minute timer is started and alarm
 * 		              is beeped again at the end of this 10 minutes.
 * 		Second Snooze: No timer is started and silencing the alarm silences it for good.
 */
void userAlarmBeep() {

	if (secondSnooze) { 		//If the user has already snoozed once,

			// Stop the timer and
			HAL_TIM_Base_Stop_IT(timerSnooze);

			// Reset count to 0
			// only bits 0 - 15 should be changed.
			timerSnooze->Instance->CNT &= 0xFFFF0000;

			// Reset interrupt status register
			timerSnooze->Instance->SR &= 0xFFFC;

//			// Re-write RCR with 10
//			timerSnooze->Instance->RCR &= 0xFF00;
//			timerSnooze->Instance->RCR |= timerSnooze_RCR;

		}

	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 500 ms)
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to
	bool displayBlink = false;

	/*
	 * Determine whether to toggle high or low brightness
	 */
	uint8_t intenSet;
	if(displayToggle == 0) { 				// If the user has full brightness enabled, toggle full btightness
		intenSet = 2;
	}
	else {									// Else, toggle low brightness
		intenSet = 1;
	}

	do {						// Beep buzzer and blink display until snooze button is pressed

		updateAndDisplayTime();				// Update to current time and display

		if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65535 / 2)) {		// Use hardware timer to blink/beep display

			sevSeg_setIntensity(sevSeg_intensityDuty[displayBlink * intenSet]);	// Toggle on/off

			HAL_GPIO_TogglePin(buzzerPort, buzzerPin);					// Toggle Buzzer

			timerVal = __HAL_TIM_GET_COUNTER(timerDelay);				// Update timer value

			displayBlink = !displayBlink;							// Toggle display blink counter

		}


		capTouch_readChannels(&capTouch);

	} while(capTouch.keyStat == 0x00 &&
			(HAL_GPIO_ReadPin(alarmEnableButtonPort, alarmEnableButtonPin) != GPIO_PIN_RESET));

	/*
	 * Stop blinking, turn off buzzer, set 50% duty cycle, update time
	 */
	HAL_TIM_Base_Stop(timerDelay);
	HAL_GPIO_WritePin(buzzerPort, buzzerPin, GPIO_PIN_RESET);
	updateAndDisplayTime();				// Update to current time and display

	sevSeg_setIntensity(sevSeg_intensityDuty[intenSet]);	// Turn display back on
	if(intenSet == 2) {
		displayToggle = 0;
	}
	else {
		displayToggle = 2;
	}

	// If this is the first snooze,
	if(!secondSnooze) {

		// Start the snooze timer to trigger an interrupt after 10 minutes
		HAL_TIM_Base_Start_IT(timerSnooze);

		// Set flag
		secondSnooze = true;

	} else {

		snoozeCounter = 0;

		// Reset flag
		/*
		 * This must be done here because if it's done
		 * in the top conditional, the secondSnooze is always true;
		 */
		secondSnooze = false;

	}

}

/*
 * General Falling-Edge EXTI Callback function
 *
 * Internal conditional determines which function to call
 * based on which button was pressed.
 */
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

/*
 * Used for second snooze functionality
 *
 * This is entered at the end of a 10-minute snooze, at
 * which point the timer kicks back an interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	snoozeCounter++;

	if((htim == timerSnooze) && (secondSnooze == true) && (snoozeCounter == timerSnooze_RCR)) {

		userAlarmBeep();

	}

}

/*
 * Toggles through user time display brightness settings
 */
HAL_StatusTypeDef displayButtonISR(void) {

	//printf("Entered display toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	updateAndDisplayTime();

	sevSeg_setIntensity(sevSeg_intensityDuty[displayToggle]);		//Turn display to proper duty cycle

	if(displayToggle >= 2) {			// Increment display toggle or reset back down to 0;
		displayToggle = 0;
//		HAL_GPIO_WritePin(GPIOB, PMLED, GPIO_PIN_RESET);		// If display is off, turn off AM/PM LED
	} else {
		displayToggle++;
	}

	return halRet;				// Return HAL status

}

/*
 * Toggles user alarm enable bool and alarm enable LED.
 */
HAL_StatusTypeDef alarmEnableISR(void) {

	//printf("Entered alarm toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	if(!userAlarmToggle) {					// If alarm is disabled, enable it.

		HAL_GPIO_WritePin(alarmLEDPort, alarmLEDPin, GPIO_PIN_SET);			// Turn on alarm LED
		userAlarmToggle = true;								// Toggle internal flag to true

	}
	else if (userAlarmToggle) {				// If alarm is enabled, disable it.

		HAL_GPIO_WritePin(alarmLEDPort, alarmLEDPin, GPIO_PIN_RESET);			// Turn off alarm LED
		userAlarmToggle = false;							// Toggle internal flag to false

	}
	else {
		__NOP();							//Code should never reach here.
	}

	// Reset snooze time
	secondSnooze = false;
	snoozeCounter = 0;

	return halRet;

}

/*
 * Delays for 3 seconds, checks if alarm set button is still pressed.
 * If it is, allow user to set the new alarm time
 * and blink display to indicate this
 *
 */
HAL_StatusTypeDef alarmSetISR(void) {

	HAL_StatusTypeDef halRet = HAL_OK;


	/*
	 * Wait for 3 seconds to see if alarm set button is still pressed
	 */

	// Delay for 3 * 1s intervals
	for(uint8_t i = 0; i < 3; i++) {

		HAL_TIM_Base_Stop(timerDelay);
		timerDelay->Instance->CNT = 0;						// Reset timer base
		HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	//	timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

		do {

		} while(__HAL_TIM_GET_COUNTER(timerDelay) < (65535));
	}

	// If button is still pressed, we are in alarm set mode.
	if(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) == GPIO_PIN_RESET) {
		alarmSetMode = true;
	}

	/*
	 * Then, if we are in alarm set mode, go through the
	 * alarm set process until the button is pressed again
	 */


	// Reset Timer
	HAL_TIM_Base_Stop(timerDelay);
	timerDelay->Instance->CNT = 0;
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	uint16_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

	bool alarmSetButtonReset = false;

	if(alarmSetMode) {

		// If we were in second snooze mode, kill it.
		secondSnooze = false;
		snoozeCounter = 0;

		bool displayBlink = false;

		/*
		 * Determine whether to toggle high or low brightness
		 */
		uint8_t intenSet;
		if(displayToggle == 0) { 				// If the user has full brightness enabled, toggle full btightness
			intenSet = 2;
		}
		else {									// Else, toggle low brightness
			intenSet = 1;
		}

		do {											// while the alarm set button is not held down, blink display.

			// Check to make sure the user has released the set button from the initial hold
			if(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) == GPIO_PIN_SET) {
				alarmSetButtonReset = true;
			}

			updateAndDisplayAlarm();

			if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65536 / 2)) {

				sevSeg_setIntensity(sevSeg_intensityDuty[displayBlink * intenSet]);		// Initialize to whatever duty cycle

				timerVal = __HAL_TIM_GET_COUNTER(timerDelay);
				displayBlink = !displayBlink;

			}

		}while((HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) != GPIO_PIN_RESET)
				|| !alarmSetButtonReset);

		sevSeg_setIntensity(sevSeg_intensityDuty[intenSet]);			// Turn display back on
		if(intenSet == 2) {
			displayToggle = 0;
		}
		else {
			displayToggle = 2;
		}


		HAL_TIM_Base_Stop(timerDelay);

		updateAndDisplayTime();

	}

	alarmSetMode = false;		// We have exited alarm set mode

	//printf("Current time back to %u:%u:%u.\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

	return halRet;

}

/*
 * Increment either user alarm hour value or RTC hour value
 */
HAL_StatusTypeDef hourSetISR(void) {


	HAL_StatusTypeDef halRet = HAL_OK;

	if(alarmSetMode) {	// If the clock is in alarm set mode, change user alarm time hour

		alarmHourInc();

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

/*
 * Increment either user alarm minute value or RTC minute value
 */
HAL_StatusTypeDef minuteSetISR(void) {


	HAL_StatusTypeDef halRet = HAL_OK;

	if(alarmSetMode) {	// If the clock is in alarm set mode, change user alarm time hour

		alarmMinuteInc();

	}
	else {									// Otherwise, change current time hour.

		currMinuteInc();

		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		/*
		 * Change internal RTC alarm to keep it triggering
		 */

		RTC_AlarmTypeDef sAlarm = {0};
		HAL_RTC_GetAlarm(&hrtc, &sAlarm, internalAlarm, RTCTimeFormat);

		getRTCTime(&hrtc, &currTime, &currDate);

		if(currTime.Minutes > 58) {
			sAlarm.AlarmTime.Minutes = 0;
		} else {
			sAlarm.AlarmTime.Minutes = currTime.Minutes + 1;
		}
		while(HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}


		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		//printf("Current time minute incremented to %u:%u:%u.\n\r", currTime.Hours,
				//currTime.Minutes, currTime.Seconds);
	}


	return halRet;
}

/*
 * Increment user alarm time hour
 */
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

	// Update RTC backup registers with new user alarm time
	updateRTCBackupReg();

}

/*
 * Increment current time hour
 */
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

	// Reset seconds
	currTime.Seconds = 0;
	currTime.SecondFraction = 0;

}

/*
 * Increment User alarm time minute
 */
void alarmMinuteInc(void) {

	if(userAlarmTime.Minutes >= 59) {
		/*
		 * The below function call is the old version in which
		 * the hour will increment when the minutes roll over.
		 */
		//alarmHourInc();
		userAlarmTime.Minutes = 0;
	}
	else if(userAlarmTime.Minutes < 59) {
		userAlarmTime.Minutes = userAlarmTime.Minutes + 1;
	}
	else {
		__NOP();
	}

	// Update RTC backup registers with new user alarm time
	updateRTCBackupReg();

}

/*
 * Increment current time minute
 */
void currMinuteInc(void) {

	getRTCTime(&hrtc, &currTime, &currDate);

	if(currTime.Minutes >= 59) {

		/*
		 * The below function call is the old version in which
		 * the hour will increment when the minutes roll over.
		 */
		// currHourInc();

		currTime.Minutes = 0;
	}
	else if(currTime.Minutes < 59) {
		currTime.Minutes = currTime.Minutes + 1;
	}
	else {
		__NOP();
	}

	// Reset seconds
	currTime.Seconds = 0;
	currTime.SecondFraction = 0;

}

/*
 * Displays a non-critical fault to indicate reduced functionality
 */
void dispFault(void) {
	HAL_GPIO_WritePin(debugLEDPort, debugLEDPin, GPIO_PIN_SET);
}

/*
 * Displays a critical failure and ceases all operations
 */
void dispFailure(void) {

	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 500 ms)
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to
	bool displayBlink = false;


	do {

		if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65535 / 4)) {		// Use hardware timer to blink/beep display

			HAL_GPIO_TogglePin(alarmLEDPort, alarmLEDPin);

			timerVal = __HAL_TIM_GET_COUNTER(timerDelay);				// Update timer value

			displayBlink = !displayBlink;							// Toggle display blink counter



		}


	} while(1);

}

/*
 * Updates RTC backup register with user alarm time to be later pulled from in the case of a power outage
 */
void updateRTCBackupReg(void) {

	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmHourBackupReg, userAlarmTime.Hours);
	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmMinuteBackupReg, userAlarmTime.Minutes);
	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmTFBackupReg, userAlarmTime.TimeFormat);

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
