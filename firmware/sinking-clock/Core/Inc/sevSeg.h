/*
 * sevSeg.h
 *
 *  Created on: Jul 4, 2023
 *      Author: marka
 */

#ifndef INC_SEVSEG_H_
#define INC_SEVSEG_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

/*
 * Initializes seven segment display IC registers
 */
void sevSeg_I2C1_Init(I2C_HandleTypeDef *hi2c1);

/*
 * Decodes RTC time and displays on 7-segment display.
 */
void sevSeg_updateDigits(I2C_HandleTypeDef *hi2c1, RTC_TimeTypeDef *updateTime);

/*
 * Sets intensity from 0% to 100% duty cycle depending on input.
 */
void sevSeg_setIntensity(I2C_HandleTypeDef *hi2c1, uint8_t dutyCycle);


#endif /* INC_SEVSEG_H_ */
