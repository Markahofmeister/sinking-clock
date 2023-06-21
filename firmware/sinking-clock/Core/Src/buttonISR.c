/*
 * buttonISR.c
 *
 *  Created on: Jun 21, 2023
 *      Author: marka
 *
 *  (desc)
 */

const uint16_t displayButtonPin = GPIO_PIN_0;
const uint16_t alarmEnableButtonPin = GPIO_PIN_1;
const uint16_t alarmSetButtonPin = GPIO_PIN_4;
const uint16_t hourSetButtonPin = GPIO_PIN_5;
const uint16_t minuteSetButtonPin = GPIO_PIN_12;
const uint16_t snoozeButtonPin = GPIO_PIN_11;

void callISR(uint16_t GPIO_Pin) {

	if(GPIO_Pin == displayButtonPin) {
		displayISR();
	}
	else if(GPIO_Pin == alarmEnableButtonPin) {
		alarmENISR();
	}
	else if(GPIO_Pin == alarmSetButtonPin) {
		alarmSetISR();
	}
	else if(GPIO_Pin == hourSetButtonPin) {
		hourSetISR();
	}
	else if(GPIO_Pin == minuteSetButtonPin) {
		miuteSetISR();
	}
	else {
		__NOP();
	}
}

uint8_t displayToggle = 0;		// flag to track current state of display
void displayISR() {

	if(displayToggle == 0) { 	// Display is off

		sevSeg_shutdown();		// Turn off display
		displayToggle++;		// increment toggle for next IRQ

	}
	else if (displayToggle == 1) {		// Display is at 50% brightness

		sevSeg_setIntensity(32);	//Set 7-seg intensity to 32/63 = 50%
		displayToggle++;			// increment toggle for next IRQ

	}
	else if (displayToggle == 2) {		// Display is at 100% brightness

		sevSeg_setIntensity(63);	//Set 7-seg intensity to 63/63 = 100%
		displayToggle = 0;			// increment toggle for next IRQ

	}
	else {				// Should never reach here
		__NOP();
	}
}

void alarmENISR() {

}

void alarmSetISR() {

}

void hourSetISR() {

}

void minuteSetISR() {

}



