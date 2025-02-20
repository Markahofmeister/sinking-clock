/*
 * vars.h
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 *
 *  No functions - just variable declarations specifically tailored to sinking clock.
 *  Written for STM32G0xx.
 */

#ifndef INC_SINKINGCLOCKVARS_H_
#define INC_SINKINGCLOCKVARS_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef STDBOOL_H
#include <stdbool.h>
#endif

#ifndef STRING_H
#include <string.h>
#endif

#define RTCTimeFormat RTC_FORMAT_BIN

/*
 * Declare variables to map button presses to GPIOs
 */
// EXTI Pins
const uint16_t displayButtonPin = GPIO_PIN_3;
GPIO_TypeDef *displayButtonPort = GPIOD;

const uint16_t alarmEnableButtonPin = GPIO_PIN_2;
GPIO_TypeDef *alarmEnableButtonPort = GPIOD;

const uint16_t alarmSetButtonPin = GPIO_PIN_15;
GPIO_TypeDef *alarmSetButtonPort = GPIOA;

const uint16_t hourSetButtonPin = GPIO_PIN_0;
GPIO_TypeDef *hourSetButtonPort = GPIOD;

const uint16_t minuteSetButtonPin = GPIO_PIN_1;
GPIO_TypeDef *minuteSetButtonPort = GPIOD;

// Capacitive Touch Reset Pin
const uint16_t capTouchResetPin = GPIO_PIN_8;
GPIO_TypeDef *capTouchResetPort = GPIOB;

// Specifies which channels of cap. touch IC to be used
uint8_t capTouchChannels = 0b00001111;

/*
 * Specifies averaging values for each cap. touch IC channel
 * This value will be right-shifted twice and must be a power of 2
 */
uint8_t AVGFact = 16;

// Specifies detection integration values for each cap. touch IC channel
uint8_t DIFact = 0x04;


/*
 * Map GPIOS to some LED outputs
 */
const uint16_t alarmLEDPin = GPIO_PIN_12;		// Or 10. Unclear.
GPIO_TypeDef *alarmLEDPort = GPIOA;

const uint16_t buzzerPin = GPIO_PIN_8;
GPIO_TypeDef *buzzerPort = GPIOA;

/*
 * GPIO Pins for shift data
 * Array of ports to map each GPIO onto
 */
const uint16_t shiftDataPin = GPIO_PIN_9;
const uint16_t shiftDataClockPin = GPIO_PIN_10;
const uint16_t shiftStoreClockPin = GPIO_PIN_7;
const uint16_t shiftOutputEnablePin = GPIO_PIN_6;
const uint16_t shiftMCLRPin = GPIO_PIN_11;
GPIO_TypeDef *GPIOPortArray[5] = {GPIOA, GPIOA, GPIOC, GPIOC, GPIOA};

/*
 * Array of all duty cycles used - 0%, 50%, 100%.
 */

const uint8_t sevSeg_intensityDuty[3] = {100, 90, 75};

/*
 * Toggle Variables
 */

// Display Toggle: 0 = 0% display Intensity, 1 = 50% display brightness, 2 = 100% display brightness
uint8_t displayToggle;

// Variable to toggle user alarm on or off. Default to off.
bool userAlarmToggle;

/*
 * RTC Calibration
 */

const uint16_t RTC_CLK_OUT_Pin = GPIO_PIN_4;
GPIO_TypeDef *RTC_CLK_OUT_Port = GPIOA;

const uint16_t RTC_CLK_ADC_IN_Pin = GPIO_PIN_2;
GPIO_TypeDef *RTC_CLK_ADC_IN_Port = GPIOA;

// Debug LED

const uint16_t debugLEDPin = GPIO_PIN_0;
GPIO_TypeDef *debugLEDPort = GPIOA;


// RTC Calibration Value
uint32_t rtcCalVal = 0x0139;


/*
 * Timer Channels
 */
uint32_t tim_PWM_CHANNEL = TIM_CHANNEL_3;
uint32_t tim_BUZZER_CHANNEL = TIM_CHANNEL_1;

/*
 * RTC access objects
 */

RTC_TimeTypeDef currTime = {0};
RTC_DateTypeDef currDate = {0};
RTC_AlarmTypeDef userAlarmObj = {0};
RTC_TimeTypeDef userAlarmTime = {0};

/*
 * User alarm backup register locations
 */

uint32_t userAlarmHourBackupReg = RTC_BKP_DR0;
uint32_t userAlarmMinuteBackupReg = RTC_BKP_DR1;
uint32_t userAlarmTFBackupReg = RTC_BKP_DR2;
uint32_t bootstrapBackupReg = RTC_BKP_DR3;

/*
 * RCR value for long 10-minute snooze
 */
const uint32_t timerSnooze_RCR = 100;



#endif /* INC_SINKINGCLOCKVARS_H_ */
