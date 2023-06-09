/*
 * sevSeg.c
 *
 *  Created on: Jul 4, 2023
 *      Author: marka
 */

#include "../Inc/sevSeg.h"

/*
 * Seven-segment display I2C peripheral address, configuration register addresses,
 * and configuration register data
 */

uint8_t sevSeg_addr = (0x38 << 1);			// MAX5868 I2C address. 0x038 shifted left for the R/W' bit

const uint8_t sevSeg_decodeReg = 0x01;		// Address for decode register
const uint8_t sevSeg_decodeData = 0x0E;		// 0b00001110 = decode hex for all segments except for the 1.
//Data buffer to send over I2C
uint8_t sevSeg_decodeBuffer[2] = {sevSeg_decodeReg, sevSeg_decodeData};

const uint8_t sevSeg_intensityReg = 0x02;		// Address for intensity register
// Intensity register takes 0bXX000000 to 0bXX111111 for 1/63 step intensity increments
// We declare 0% duty cycle, 50% duty cycle, and 100% duty cycle.
uint8_t sevSeg_intensityBuff[2] = {sevSeg_intensityReg, 31};

const uint8_t sevSeg_SDReg = 0x04;			// Address for shutdown register
const uint8_t sevSeg_SD_ON = 0x01;			// Display ON - only mess with bit 0
const uint8_t sevSeg_SD_OFF = 0x00;			// Display OFF - only mess with bit 0
//Data buffer to send over I2C
uint8_t sevSeg_SD_ONBuff[2] = {sevSeg_SDReg, sevSeg_SD_ON};
uint8_t sevSeg_SD_OFFBuff[2] = {sevSeg_SDReg, sevSeg_SD_OFF};

const uint8_t sevSeg_testReg = 0x07;			// Address for display test
const uint8_t sevSeg_testOFF = 0x00;			// Display test OFF
const uint8_t sevSeg_testON = 0x01;				//Display test ON
//Data buffer to send over I2C
uint8_t sevSeg_testOFFBuff[2] = {sevSeg_testReg, sevSeg_testOFF};
uint8_t sevSeg_testONBuff[2] = {sevSeg_testReg, sevSeg_testON};

// I2C Register addresses to access individual digits
const uint8_t sevSeg_digit0Reg = 0x20;
const uint8_t sevSeg_digit1Reg = 0x21;
const uint8_t sevSeg_digit2Reg = 0x22;
const uint8_t sevSeg_digit3Reg = 0x23;

// Array of all possible digit outputs
const uint8_t dispDigits[10] = {0x00, 0x01, 0x02, 0x03, 0x04,
	  	  	  	  	  	  	  	0x05, 0x06, 0x07, 0x08, 0x09};

const uint8_t sevSeg_digit0_OFF = 0x03;		// 0b00000011 = colons on, 1 is off.
const uint8_t sevSeg_digit0_ON = 0x33;		// 0b00110011 = colons on, 1 is on.

//Data buffers to send to each individual LED display - initialized to all 0s.
uint8_t sevSeg_digit0Buff[2] = {sevSeg_digit0Reg, sevSeg_digit0_OFF};
uint8_t sevSeg_digit1Buff[2] = {sevSeg_digit1Reg, dispDigits[0]};
uint8_t sevSeg_digit2Buff[2] = {sevSeg_digit2Reg, dispDigits[0]};
uint8_t sevSeg_digit3Buff[2] = {sevSeg_digit3Reg, dispDigits[0]};


void sevSeg_I2C1_Init(I2C_HandleTypeDef *hi2c1) {


	/*
	 * Master TX format:
	 * HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, 		--> config struct (declared up top)
	 * 						   uint16_t DevAddress, 			--> peripheral address
	 * 						   uint8_t *pData,					--> pointer to buffer of data to be sent
     *                         uint16_t Size, 					--> Size of data
     *                         uint32_t Timeout);				--> timeout until return
	 */

	HAL_StatusTypeDef halRetI2C;			// HAL status to monitor I2C initialization status

	//Set display to decode hex data inputs
	halRetI2C = HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_decodeBuffer, 2, HAL_MAX_DELAY);

	if(halRetI2C != HAL_OK) {		//check HAL
		printf("HAL Error - TX decode mode\n\r");
	} else{
		printf("Display set to decode mode\n\r");
	}

	//Disable shutdown mode
	halRetI2C = HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_SD_ONBuff, 2, HAL_MAX_DELAY);

	if(halRetI2C != HAL_OK) {		//check HAL
		printf("HAL Error - TX disable shutdown mode\n\r");
	} else {
		printf("Display shutdown mode disabled\n\r");
	}

	//Set to test mode
	halRetI2C = HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_testONBuff, 2, HAL_MAX_DELAY);

	if(halRetI2C != HAL_OK) {		//check HAL
		printf("HAL Error - TX test mode ON data\n\r");
	} else {
		printf("Test mode enabled - all LEDs on\n\r");
	}

	// Disable test mode
	halRetI2C = HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_testOFFBuff, 2, HAL_MAX_DELAY);

	if(halRetI2C != HAL_OK) {		//check HAL
		printf("HAL Error - TX test mode OFF data\n\r");
	} else {
		printf("Test mode disabled - all LEDs off\n\r");
	}

	sevSeg_intensityBuff[1] = 31;		// Initialize to 50% duty cycle
	halRetI2C = HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_intensityBuff, 2, HAL_MAX_DELAY);

	if(halRetI2C != HAL_OK) {		//check HAL
		printf("HAL Error - TX intensity level data\n\r");
	} else {
		printf("Intensity Set\n\r");
	}

	return;

}

void sevSeg_updateDigits(I2C_HandleTypeDef *hi2c1, RTC_TimeTypeDef *updateTime) {

	if(updateTime->Hours / 10 == 1) {					// Check whether or not to activate 1 for hour
		sevSeg_digit0Buff[1] = sevSeg_digit0_ON;
	}
	else {
		sevSeg_digit0Buff[1] = sevSeg_digit0_OFF;
	}

	sevSeg_digit1Buff[1] = updateTime->Hours % 10;		// tenths digit of hours value
	sevSeg_digit2Buff[1] = updateTime->Minutes / 10;	// ones digit of minutes value
	sevSeg_digit3Buff[1] = updateTime->Minutes % 10;	// tenths digit of minutes value

	// Transmit updated time digits to display digits
	HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_digit0Buff, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_digit1Buff, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_digit2Buff, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_digit3Buff, 2, HAL_MAX_DELAY);

	return;

}

void sevSeg_setIntensity(I2C_HandleTypeDef *hi2c1, uint8_t dutyCycle) {

	if(dutyCycle >= 0 && dutyCycle <= 63) {		// Only change intensity if input is between 0 and 63.

		sevSeg_intensityBuff[1] = dutyCycle;
		HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_intensityBuff, 2, HAL_MAX_DELAY);

	}

}

