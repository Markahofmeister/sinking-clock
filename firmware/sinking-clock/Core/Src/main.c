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
#include "sevSeg.h"
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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*
 * Array of all duty cycles used - 0%, 50%, 100%.
 */

const uint8_t sevSeg_intensityDuty[3] = {0x00, 0x31, 0x63};



/*
 * RTC access objects
 */

RTC_TimeTypeDef currTime;
RTC_DateTypeDef currDate;
RTC_TimeTypeDef userAlarmTime;
RTC_DateTypeDef userAlamrmDate;
RTC_AlarmTypeDef userAlarmObj;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/*
 * Call to fetch the current time from the RTC and send to the LED display.
 */
HAL_StatusTypeDef updateAndDisplayTime(void);

/*
 * Call to fetch the current alarm from the RTC and send to the LED display.
 */
HAL_StatusTypeDef updateAndDisplayAlarm(void);

/*
 * Called on interrupt from display button to toggle 7-segment intensity.
 */
HAL_StatusTypeDef displayButtonISR(void);

/*
 * Called on interrupt from alarm enable button to toggle user alarm.
 */
HAL_StatusTypeDef alarmEnableISR(void);

/*
 * Called on interrupt from alarm set button to enter alarm set loop.
 */
HAL_StatusTypeDef alarmSetISR(void);

/*
 * Called on interrupt from hour and minute set buttons.
 * Contain logic to set user alarm hour or minute.
 */
HAL_StatusTypeDef hourSetISR(void);
HAL_StatusTypeDef minuteSetISR(void);

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  displayToggle = 2; 		// Display at 100% intensity for next display toggle
  initRTCTime(&hrtc, &currTime, &currDate);
  sevSeg_I2C1_Init(&hi2c1);		//Initialize 7-seg

  	HAL_StatusTypeDef halRet = updateAndDisplayTime();

  	if(halRet != HAL_OK) {		//check HAL
  		printf("HAL Error - TX current time\n\r");
  	} else {
  		printf("Display Updated with current time\n\r");
  	}

  userAlarmToggle = false;			//Default to off
  __HAL_RTC_ALARMB_DISABLE(&hrtc);				// Deactivate alarm

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
  hi2c1.Init.Timing = 0x0010061A;
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

  /** Enable the Alarm B
  */
  sAlarm.AlarmTime.Hours = 0x2;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Output_Pin|PM_LED_Pin|Alarm_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Display_Button_Pin Alarm_Set_Button_Pin Alarm_Enable_Button_Pin Hour_Set_Button_Pin
                           Minute_Set_Button_Pin */
  GPIO_InitStruct.Pin = Display_Button_Pin|Alarm_Set_Button_Pin|Alarm_Enable_Button_Pin|Hour_Set_Button_Pin
                          |Minute_Set_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Output_Pin PM_LED_Pin Alarm_LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Output_Pin|PM_LED_Pin|Alarm_LED_Pin;
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
	sevSeg_updateDigits(&hi2c1, &currTime);

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
	sevSeg_updateDigits(&hi2c1, &userAlarmTime);

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
	  } else {
		sAlarm.AlarmTime.Minutes=sAlarm.AlarmTime.Minutes+1;
	  }
		while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}

	  updateAndDisplayTime();

	  printf("Current time: %d : %d : %d\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

}

void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc) {

	printf("Enter user alarm interrupt.\n\r");

	if(userAlarmToggle) {			//Only execute sequence if the alarm is actually on

		HAL_TIM_Base_Start(&htim16);						// Begin timer 16 counting (to 500 ms)
		uint16_t timerVal = __HAL_TIM_GET_COUNTER(&htim16);	// Get initial timer value to compare to
		bool displayBlink = false;

		do {						// Beep buzzer and blink display until snooze button is pressed

			updateAndDisplayTime();

			if(__HAL_TIM_GET_COUNTER(&htim16) - timerVal >= (65536 / 2)) {

				sevSeg_setIntensity(&hi2c1, sevSeg_intensityDuty[displayBlink]);


				HAL_GPIO_TogglePin(GPIOB, buzzerPin);

				timerVal = __HAL_TIM_GET_COUNTER(&htim16);
				displayBlink = !displayBlink;

			}

		} while(!capTouchTrigger(snoozeButtonPin));

	}

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

	sevSeg_setIntensity(&hi2c1, sevSeg_intensityDuty[displayToggle]);		//Turn display to proper duty cycle

	if(displayToggle >= 2) {			// Increment display toggle or reset back down to 0;
		displayToggle = 0;
	} else {
		displayToggle++;
	}

	return halRet;				// Return HAL status

}

HAL_StatusTypeDef alarmEnableISR(void) {

	printf("Entered alarm toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	if(!userAlarmToggle) {					// If alarm is disabled, enable it.

		getUserAlarmTime(&hrtc, &userAlarmTime);

		__HAL_RTC_ALARMB_ENABLE(&hrtc);					//Enable alarm?

		HAL_GPIO_WritePin(GPIOB, alarmLED, GPIO_PIN_SET);			// Turn on alarm LED
		userAlarmToggle = true;								// Toggle internal flag to true

		printf("User alarm set to: %d:%d:%d.\n\r", userAlarmTime.Hours,
								userAlarmTime.Minutes, userAlarmTime.Seconds);

	}
	else if (userAlarmToggle) {				// If alarm is enabled, disable it.

		__HAL_RTC_ALARMB_DISABLE(&hrtc);				// Disable alarm

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
	printf("User alarm currently set to %d:%d:%d.\n\r", userAlarmTime.Hours,
			userAlarmTime.Minutes, userAlarmTime.Seconds);

	HAL_StatusTypeDef halRet = HAL_OK;

	HAL_TIM_Base_Start(&htim16);						// Begin timer 16 counting (to 500 ms)
	uint16_t timerVal = __HAL_TIM_GET_COUNTER(&htim16);	// Get initial timer value to compare to
	bool displayBlink = false;

	do {											// while the alarm set button is not held down, blink display.

		updateAndDisplayAlarm();

		if(__HAL_TIM_GET_COUNTER(&htim16) - timerVal >= (65536 / 2)) {

			sevSeg_setIntensity (&hi2c1, sevSeg_intensityDuty[displayBlink]);		// Initialize to whatever duty cycle

			timerVal = __HAL_TIM_GET_COUNTER(&htim16);
			displayBlink = !displayBlink;

		}

	}while(HAL_GPIO_ReadPin(GPIOA, alarmSetButtonPin) == GPIO_PIN_RESET);

	sevSeg_setIntensity(&hi2c1, sevSeg_intensityDuty[2]);

	HAL_TIM_Base_Stop(&htim16);

	updateAndDisplayTime();
	printf("Current time back to %d:%d:%d.\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

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

		HAL_RTC_SetAlarm(&hrtc, &userAlarmObj, RTC_FORMAT_BCD);
		getUserAlarmTime(&hrtc, &userAlarmTime);

		printf("User alarm hour incremented to %d:%d:%d\n\r", userAlarmTime.Hours,
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
		HAL_RTC_SetTime(&hrtc, &currTime, RTC_FORMAT_BCD);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		printf("Current time hour incremented to %d:%d:%d.\n\r", currTime.Hours,
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

		HAL_RTC_SetAlarm(&hrtc, &userAlarmObj, RTC_FORMAT_BCD);

		printf("User alarm minute incremented to %d:%d:%d\n\r", userAlarmObj.AlarmTime.Hours,
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
		HAL_RTC_SetTime(&hrtc, &currTime, RTC_FORMAT_BCD);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		printf("Current time minute incremented to %d:%d:%d.\n\r", currTime.Hours,
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
