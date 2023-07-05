/*
 * sevSeg.h
 *
 *  Created on: Jul 4, 2023
 *      Author: marka
 *
 *  Designed to drive Analog Devices MAX6958 LED QUAD LED display driver.
 *  Works with STM32 HAL
 */

#ifndef INC_SEVSEG_H_
#define INC_SEVSEG_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

/*
 * Initializes seven segment display IC registers
 *
 * @param STM32 HAL I2C handling object pointer
 * @ret none
 */
void sevSeg_I2C1_Init(I2C_HandleTypeDef *hi2c1);

/*
 * Decodes RTC time and displays on 7-segment display.
 *
 * @param STM32 HAL I2C handling object pointer
 * STM32 HAL RTC time handling object pointer
 */
void sevSeg_updateDigits(I2C_HandleTypeDef *hi2c1, RTC_TimeTypeDef *updateTime);

/*
 * Sets intensity from 0% to 100% duty cycle depending on input.
 *
 * @param STM32 HAL I2C handling object pointer
 * @param 8-bit unsigned integer to set intensity, 0-63.
 */
void sevSeg_setIntensity(I2C_HandleTypeDef *hi2c1, uint8_t dutyCycle);


#endif /* INC_SEVSEG_H_ */
