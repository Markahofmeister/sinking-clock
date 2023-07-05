/*
 * ctouch.c
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 */

#include "../Inc/ctouch.h"

bool capTouchTrigger(uint8_t buttonPin) {

	if(HAL_GPIO_ReadPin(GPIOA, buttonPin) == GPIO_PIN_RESET) {		// If button is low, cap. touch has triggered.
		return true;
		printf("Cap. touch triggered.\n\r");
	}

	return false;

}
