/*
 * sevSeg.h
 *
 *  Created on: Jul 4, 2023
 *      Author: marka
 */

#ifndef INC_SEVSEG_H_
#define INC_SEVSEG_H_

//#include "../Src/sevSeg.c"
#include "stm32g0xx_hal.h"
/*
 * Initializes seven segment display IC registers
 */
void sevSeg_I2C1_Init(I2C_HandleTypeDef *hi2c1);


#endif /* INC_SEVSEG_H_ */
