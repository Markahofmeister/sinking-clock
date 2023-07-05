/*
 * ctouch.h
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 *
 *  Simple library to configure/check cap. touch sensor
 *  Works with STM32 HAL
 */

#ifndef INC_CTOUCH_H_
#define INC_CTOUCH_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef STDBOOL_H
#include <stdbool.h>
#endif

/*
 * Checks cap. touch trigger. Assumes active low operation.
 *
 * @param STM32 GPIO pin - must be an input.
 *
 * @ret boolean value - true if cap. touch has been triggered, false if not.
 */
bool capTouchTrigger(uint8_t buttonPin);

#endif /* INC_CTOUCH_H_ */
