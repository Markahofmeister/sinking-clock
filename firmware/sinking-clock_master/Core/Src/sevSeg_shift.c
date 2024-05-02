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

/*
 * 0b00010000 = Digit 3 LED OFF, LED colons ON, PM LED OFF
 * 0b00110000 = Digit 3 LED ON, LED colons ON, PM LED OFF
 * 0b00011000 = Digit 3 LED OFF, LED colons ON, PM LED ON
 * 0b00111000 = Digit 3 LED ON, LED colons ON, PM LED ON
 */
const uint8_t dig3Seg[4] = {0b00010000, 0b00110000, 0b00011000, 0b00111000};

/*
 * Global variables to be initialized with GPIO assignments
 */
uint16_t shiftData;
uint16_t shiftDataClock;
uint16_t shiftStoreClock;
uint16_t shiftOutputEnable;
uint16_t shiftMCLR;

/*
 * Array of GPIO ports to be initialized for each shift register GPIO
 * GPIO Port A values are placeholders
 * Order:
 * [0] = shift data pin port
 * [1] = shift data clock pin port
 * [2] = shift store clock pin port
 * [3] = shift output enable pin port
 * [4] = shift master clear pin port
 */
GPIO_TypeDef *portArray[5] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};

// Used to avoid conditionals
GPIO_PinState GPIOPinSet[2] = {GPIO_PIN_RESET, GPIO_PIN_SET};


void sevSeg_Init(uint16_t shiftDataPin, uint16_t shiftDataClockPin, uint16_t shiftStoreClockPin,
					uint16_t shiftOutputEnablePin, uint16_t shiftMCLRPin,
					GPIO_TypeDef **GPIOPortArray, TIM_HandleTypeDef *htim, TIM_HandleTypeDef *htim_PWM) {

	shiftData = shiftDataPin;
	shiftDataClock = shiftDataClockPin;
	shiftStoreClock = shiftStoreClockPin;
	shiftOutputEnable = shiftOutputEnablePin;
	shiftMCLR = shiftMCLRPin;

	for(int i = 0; i < 5; i++) {
		portArray[i] = GPIOPortArray[i];
	}

	// Clear any existing shift register data
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[0]);
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[1]);

	// Store cleared data and Enable output
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);
	HAL_GPIO_WritePin(portArray[3], shiftOutputEnable, GPIOPinSet[0]);

	// Set duty cycle to 50%

	sevSeg_setIntensity(htim_PWM, 90);

	//Flash an initializing "Hof" symbol
	uint8_t hofSymb[4] = {0b00000000, 0b00110111, 0b00011101, 0b01000111};

	uint8_t sendByte;					// To be used to shift bits

	for(int i = 0; i <= 3; i++) {

		sendByte = hofSymb[i];

		for(int j = 0; j < 8; j++) {

			// Write data pin with LSB of data
			HAL_GPIO_WritePin(portArray[0], shiftData, GPIOPinSet[sendByte & 1]);

			// Toggle clock GPIO to shift bit into register
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[1]);
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[0]);

			// Once data pin has been written and shifted out, shift data right by one bit.
			sendByte >>= 1;

		}
	}

	// Once all data has been shifted out, toggle store clock register to display data.
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);

	// Delay for 500 ms using hardware timer
	HAL_TIM_Base_Stop(htim);
	HAL_TIM_Base_Start(htim);							// Begin timer counting
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(htim);	// Get initial timer value to compare to

	//Hang in dead loop until 500 ms
	while(__HAL_TIM_GET_COUNTER(htim) - timerVal <= (65535 / 2)){ timerVal = __HAL_TIM_GET_COUNTER(htim); }

	HAL_TIM_Base_Stop(htim);

	// Clear any existing shift register data
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[0]);
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[1]);

	// Store cleared data and Enable output
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);

}

void sevSeg_updateDigits(RTC_TimeTypeDef *updateTime) {

	/*
	 * Determine what time to send to shift registers
	 * digit 3 is a special case - the colons are always on, but the one can be on/off.
	 * Therefore, use array indexing to decide what to send.
	 */
	uint8_t sendTime[4] = {updateTime->Hours / 10, updateTime->Hours % 10,
							updateTime->Minutes / 10, updateTime->Minutes % 10};

	//uint8_t sendTime[4] = {8, 8, 8, 8};

	uint8_t sendByte;					// To be used to shift bits

	/*
	 * If we are in PM, we should reflect this in the PM LED.
	 * This offset will update the digit 3 shift register value with the correct sequence.
	 */
	uint8_t dig3Offset = 0;

	if(updateTime->TimeFormat == RTC_HOURFORMAT12_PM) {
		dig3Offset = 2;
	}

	for(int i = 0; i < 4; i++) {

		sendByte = dispDigits[sendTime[i]];

		if(i == 0) {		// If tenth's place of hour, use special values
			sendByte = dig3Seg[(updateTime->Hours / 10) + dig3Offset];
		}

		for(int j = 0; j < 8; j++) {

			// Write data pin with LSB of data
			HAL_GPIO_WritePin(portArray[0], shiftData, GPIOPinSet[sendByte & 1]);

			// Toggle clock GPIO to shift bit into register
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[1]);
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[0]);

			// Once data pin has been written and shifted out, shift data right by one bit.
			sendByte >>= 1;

		}
	}

	// Once all data has been shifted out, toggle store clock register to display data.

	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);

	return;

}

void sevSeg_setIntensity(TIM_HandleTypeDef *htim_PWM, uint16_t dutyCycle) {

	__HAL_TIM_SET_COMPARE(htim_PWM, TIM_CHANNEL_2, dutyCycle);
	HAL_TIM_PWM_Start(htim_PWM, TIM_CHANNEL_2);

}









