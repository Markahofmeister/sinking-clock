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
const uint16_t displayButtonPin = GPIO_PIN_0;
const uint16_t alarmEnableButtonPin = GPIO_PIN_6;
const uint16_t alarmSetButtonPin = GPIO_PIN_1;
const uint16_t hourSetButtonPin = GPIO_PIN_5;
const uint16_t minuteSetButtonPin = GPIO_PIN_12;

// Input Pin
const uint16_t snoozeButtonPin = GPIO_PIN_11;

/*
 * Map GPIOS to some LED outputs
 */

const uint16_t alarmLED = GPIO_PIN_8;			//Port B
const uint16_t PMLED = GPIO_PIN_7;				//Port B
const uint16_t buzzerPin = GPIO_PIN_1;			//Port B

/*
 * GPIO Pins for shift data
 * Array of ports to map each GPIO onto
 */
uint16_t shiftDataPin = GPIO_PIN_15;
uint16_t shiftDataClockPin = GPIO_PIN_5;
uint16_t shiftStoreClockPin = GPIO_PIN_4;
uint16_t shiftOutputEnablePin = GPIO_PIN_3;
uint16_t shiftMCLRPin = GPIO_PIN_6;
GPIO_TypeDef GPIOPortArray[5] = {GPIOA, GPIOB, GPIOB, GPIOB, GPIOB};

/*
 * Array of all duty cycles used - 0%, 50%, 100%.
 */

const uint8_t sevSeg_intensityDuty[3] = {100, 50, 00};
// The above array is "backwards" w.r.t. what might intuitively makes sense to allow for
// some clever logic in main function ISRs.

/*
 * Toggle Variables
 */

// Display Toggle: 0 = 0% display Intensity, 1 = 50% display brightness, 2 = 100% display brightness
uint8_t displayToggle;

// Variable to toggle user alarm on or off. Default to off.
bool userAlarmToggle;


#endif /* INC_SINKINGCLOCKVARS_H_ */
