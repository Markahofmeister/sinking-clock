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
#include <stdio.h>
#include <string.h>

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*
 * Declare variables to map button presses to GPIOs
 */
const uint16_t displayButtonPin = GPIO_PIN_0;
const uint16_t alarmEnableButtonPin = GPIO_PIN_1;
const uint16_t alarmSetButtonPin = GPIO_PIN_4;
const uint16_t hourSetButtonPin = GPIO_PIN_5;
const uint16_t minuteSetButtonPin = GPIO_PIN_12;
const uint16_t snoozeButtonPin = GPIO_PIN_11;

/*
 * Declare variables to map interrupts from button GPIOs to LED outputs.
 * Some of these are port B;
 */

const uint16_t displayLEDPin = GPIO_PIN_7;			//Port B
const uint16_t alarmEnableLEDPin = GPIO_PIN_6;		//Port B
const uint16_t alarmSetLEDPin = GPIO_PIN_1;			//Port B
const uint16_t hourSetLEDPin = GPIO_PIN_10;			//Port A
const uint16_t minuteSetLEDPin = GPIO_PIN_9;		//Port A
const uint16_t snoozeButtonLEDPin = GPIO_PIN_0;		//Port B
const uint16_t RTCInterruptLEDPin = GPIO_PIN_6;		//Port A

/*
 * Seven-segment display I2C peripheral address, configuration register addresses,
 * and configuration register data
 */

uint8_t sevSeg_addr = (0x38 << 1);			//MAX5868 I2C address. 0x038 shifted left for the R/W' bit



const uint8_t sevSeg_decodeReg = 0x01;		//Address for decode register
const uint8_t sevSeg_decodeData = 0x0F;		//0b00001111 = decode hex for all segments
//Data buffer to send over I2C
uint8_t sevSeg_decodeBuffer[2] = {sevSeg_decodeReg, sevSeg_decodeData};

uint8_t sevSeg_intensityReg = 0x02;		//Address for intensity register
// Intensity register takes 0bXX000000 to 0bXX111111 for 1/64 step intensity increments

const uint8_t sevSeg_SDReg = 0x04;			//Address for shutdown register
const uint8_t sevSeg_SD_ON = 0x01;			//Display ON - only mess with bit 0
const uint8_t sevSeg_SD_OFF = 0x00;			//Display OFF - only mess with bit 0
//Data buffer to send over I2C
uint8_t sevSeg_SD_ONBuff[10] = {sevSeg_SDReg, sevSeg_SD_ON};
uint8_t sevSeg_SD_OFFBuff[10] = {sevSeg_SDReg, sevSeg_SD_OFF};

const uint8_t sevSeg_testReg = 0x07;			//Address for display test
const uint8_t sevSeg_testOFF = 0x00;			//Display test OFF
const uint8_t sevSeg_testON = 0x01;			//Display test ON
//Data buffer to send over I2C
uint8_t sevSeg_testOFFBuff[2] = {sevSeg_testReg, sevSeg_testOFF};
uint8_t sevSeg_testONBuff[2] = {sevSeg_testReg, sevSeg_testON};


const uint8_t sevSeg_digit0Reg = 0x20;
const uint8_t sevSeg_digit1Reg = 0x21;
const uint8_t sevSeg_digit2Reg = 0x22;
const uint8_t sevSeg_digit3Reg = 0x23;

/*
 * Cap touch I2C peripheral data
 */
const uint8_t capTouch_addr = (0x37 << 1);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static void sevSeg_I2C1_Init(void);

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  sevSeg_I2C1_Init();			//Initialize 7-segment display to test mode

//  RTC_TimeTypeDef currTimeMain;
//  RTC_DateTypeDef currDateMain;
//  HAL_RTC_GetTime(hrtc, &currTimeMain, RTC_HourFormat_12);
//  HAL_RTC_GetDate(hrtc, &currDateMain, RTC_FORMAT_BIN);

  	  if(HAL_GPIO_ReadPin(GPIOA, snoozeButtonPin) == GPIO_PIN_RESET) {
  		  HAL_GPIO_WritePin(GPIOB, displayLEDPin, GPIO_PIN_SET);
  	  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  HAL_StatusTypeDef HalRet;
//
//	  uint8_t dispDigits[10] = {0x00, 0x01, 0x02, 0x03, 0x04,
//			  	  	  	  	  	  0x05, 0x06, 0x07, 0x08, 0x09};
//	  uint8_t sevSeg_digit0Buff[2] = {sevSeg_digit0Reg, 0x00};
//	  uint8_t sevSeg_digit1Buff[2] = {sevSeg_digit1Reg, 0x00};
//
//	  for (uint i = 0; i < 10; i++) {
//
//		sevSeg_digit0Buff[1] = dispDigits[i];
//		sevSeg_digit1Buff[1] = dispDigits[i+1];
//
//		HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_digit0Buff, 2, HAL_MAX_DELAY);
//		HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_digit1Buff, 2, HAL_MAX_DELAY);
//
//		if(HalRet != HAL_OK) {		//check HAL
//			printf("HAL Error - TX digit data\n\r");
//		} else {
//			printf("Digit incremented and displayed\n\r");
//		}
//
//		HAL_Delay(1000);
//
//	  }




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
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x1;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_MINUTES;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, RTC_Interrupt_LED_Pin|Minute_Set_LED_Pin|Hour_Set_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Snooze_LED_Pin|Alarm_Set_LED_Pin|Alarm_Enable_LED_Pin|Display_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Display_Button_Pin Alarm_Enable_Button_Pin Alarm_Set_Button_Pin Hour_Set_Button_Pin
                           Snooze_Button_Pin Minute_Set_Button_Pin */
  GPIO_InitStruct.Pin = Display_Button_Pin|Alarm_Enable_Button_Pin|Alarm_Set_Button_Pin|Hour_Set_Button_Pin
                          |Snooze_Button_Pin|Minute_Set_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RTC_Interrupt_LED_Pin Minute_Set_LED_Pin Hour_Set_LED_Pin */
  GPIO_InitStruct.Pin = RTC_Interrupt_LED_Pin|Minute_Set_LED_Pin|Hour_Set_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Snooze_LED_Pin Alarm_Set_LED_Pin Alarm_Enable_LED_Pin Display_LED_Pin */
  GPIO_InitStruct.Pin = Snooze_LED_Pin|Alarm_Set_LED_Pin|Alarm_Enable_LED_Pin|Display_LED_Pin;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  // Initialize all output pins to low
	HAL_GPIO_WritePin(GPIOB, displayLEDPin, 0);
	HAL_GPIO_WritePin(GPIOB, alarmEnableLEDPin, 0);
	HAL_GPIO_WritePin(GPIOB, alarmSetLEDPin, 0);
	HAL_GPIO_WritePin(GPIOA, hourSetLEDPin, 0);
	HAL_GPIO_WritePin(GPIOA, minuteSetLEDPin, 0);
	HAL_GPIO_WritePin(GPIOB, snoozeButtonLEDPin, 0);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == displayButtonPin) {
		HAL_GPIO_TogglePin(GPIOB, displayLEDPin);
	}
	else if(GPIO_Pin == alarmEnableButtonPin) {
		HAL_GPIO_TogglePin(GPIOB, alarmEnableLEDPin);
	}
	else if(GPIO_Pin == alarmSetButtonPin) {
		HAL_GPIO_TogglePin(GPIOB, alarmSetLEDPin);
	}
	else if(GPIO_Pin == hourSetButtonPin) {
		HAL_GPIO_TogglePin(GPIOA, hourSetLEDPin);
	}
	else if(GPIO_Pin == minuteSetButtonPin) {
		HAL_GPIO_TogglePin(GPIOA, minuteSetLEDPin);
	}
	else if(GPIO_Pin == snoozeButtonPin) {
		HAL_GPIO_TogglePin(GPIOB, snoozeButtonLEDPin);
	}
	else {
		__NOP();
	}
//  if(GPIO_Pin == GPIO_PIN_1) {
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//    printf("ISR Entered\n");
//  } else {
//      __NOP();
//  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

  RTC_AlarmTypeDef sAlarm;
  HAL_RTC_GetAlarm(hrtc,&sAlarm,RTC_ALARM_A,FORMAT_BIN);

  printf("Enter alarm interrupt\n\r");

  RTC_TimeTypeDef currTime;
  RTC_DateTypeDef currDate;
  HAL_RTC_GetTime(hrtc, &currTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(hrtc, &currDate, RTC_FORMAT_BIN);		//get date is necessary, else RTC will not update time

  if(sAlarm.AlarmTime.Seconds>58) {
    sAlarm.AlarmTime.Seconds=0;
  } else {
    sAlarm.AlarmTime.Seconds=sAlarm.AlarmTime.Seconds+1;
  }
    while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}
    HAL_GPIO_TogglePin(GPIOA, RTCInterruptLEDPin);

  printf("Current time: %d : %d : %d\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

}

static void sevSeg_I2C1_Init(void) {


	/*
	 * Master TX format:
	 * HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, 		--> config struct (declared up top)
	 * 						   uint16_t DevAddress, 			--> peripheral address
	 * 						   uint8_t *pData,					--> pointer to buffer of data to be sent
     *                         uint16_t Size, 					--> Size of data
     *                         uint32_t Timeout);				--> timeout until return
	 */

	HAL_StatusTypeDef HalRet;

	//Set display to decode hex data inputs
	HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_decodeBuffer, 2, HAL_MAX_DELAY);

	if(HalRet != HAL_OK) {		//check HAL
		printf("HAL Error - TX decode mode\n\r");
	} else{
		printf("Display set to decode mode\n\r");
	}

	//Disable shutdown mode
	HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_SD_ONBuff, 2, HAL_MAX_DELAY);

	if(HalRet != HAL_OK) {		//check HAL
		printf("HAL Error - TX disable shutdown mode\n\r");
	} else {
		printf("Display shutdown mode disabled\n\r");
	}

	//Set to test mode
	HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_testONBuff, 2, HAL_MAX_DELAY);

	if(HalRet != HAL_OK) {		//check HAL
		printf("HAL Error - TX test mode ON data\n\r");
	} else {
		printf("Test mode enabled - all LEDs on\n\r");
	}

//	uint8_t sevSeg_intensityBuff[2] = {sevSeg_intensityReg, 0b00100000};	//intensity = 32/64
//	HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_intensityBuff, 2, HAL_MAX_DELAY);
//
//	if(HalRet != HAL_OK) {		//check HAL
//		printf("HAL Error - TX intensity level data\n\r");
//	} else {
//		printf("Intensity Set\n\r");
//	}

	HAL_Delay(500);

	//Set to test mode
	HalRet = HAL_I2C_Master_Transmit(&hi2c1, sevSeg_addr, sevSeg_testOFFBuff, 2, HAL_MAX_DELAY);

	if(HalRet != HAL_OK) {		//check HAL
		printf("HAL Error - TX test mode OFF data\n\r");
	} else {
		printf("Test mode disabled - all LEDs off\n\r");
	}

	return;

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
