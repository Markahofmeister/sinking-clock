/*
 * sevSeg_shift.c
 *
 *  Created on: Jul 23, 2023
 *      Author: marka
 */

#include "../Inc/sevSeg_shift.h"

/*
 * Digit 0 = LSB (ones place of minutes)
 * .
 * .
 * Digit 3 = MSB (tenths place of hours)
 */

// Binary codes for each digit in the form of a 7-segment display.
// All codes must have a 0 in the MSB, as the first output on each register is NC.
const uint8_t dispDigits[10] = {0b01111110, 	// 0
								0b00110000,		// 1
								0b01101101,		// 2
								0b01111001,		// 3
								0b00110011,		// 4
								0b01011011,		// 5
								0b01011111,		// 6
								0b01110000,		// 7
								0b01111111,		// 8
								0b01111011};	// 9

const uint8_t dig3



void sevSeg_Init(uint8_t shiftDataPin, uint8_t shiftDataClockPin, uint8_t shiftStoreClockPin,
					uint8_t shiftOutputEnablePin, uint8_t shiftMCLRPin, TIM_HandleTypeDef *htim) {

}

void sevSeg_updateDigits(uint8_t shiftDataPin, uint8_t shiftDataClockPin,
							uint8_t shiftStoreClockPin, RTC_TimeTypeDef *updateTime) {

	uint8_t sendTime[4] = {updateTime->Hours / 10, updateTime->Hours % 10,
							updateTime->Minutes / 10, updateTime->Minutes % 10};

	if(updateTime->Hours / 10 == 1) {					// Check whether or not to activate 1 for hour

	}
	else {

	}

	uint8_t sendByte;

	for(int i = 1; i <= 3; i++) {

		sendByte = dispDigits[sendTime[i]];

		for(int j = 0; j < 8; j++) {

			// write data pin with byte masked with & 1
			// toggle clock
			// shift bits right

		}
	}


	return;

}

void sevSeg_setIntensity(uint8_t dutyCycle) {

}
