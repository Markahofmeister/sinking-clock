/*
 * sevSeg_shift.h
 *
 *  Created on: Jul 25, 2023
 *      Author: marka
 */

#ifndef INC_SEVSEG_SHIFT_H_
#define INC_SEVSEG_SHIFT_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

/*
 * - Clears any existing shift register data
 * - Sets intensity to 50%
 * - Flashes an initializing "Hof" symbol
 */

void sevSeg_Init(uint8_t shiftDataPin, uint8_t shiftDataClockPin, uint8_t shiftStoreClockPin,
					uint8_t shiftOutputEnablePin, uint8_t shiftMCLRPin,
					TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIOPortArray);


void sevSeg_updateDigits(RTC_TimeTypeDef *updateTime);


void sevSeg_setIntensity(uint8_t dutyCycle);


#endif /* INC_SEVSEG_SHIFT_H_ */
