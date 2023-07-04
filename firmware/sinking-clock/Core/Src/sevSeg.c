/*
 * sevSeg.c
 *
 *  Created on: Jul 4, 2023
 *      Author: marka
 */

#include "../Inc/sevSeg.h"
#include <stdio.h>

/*
 * Seven-segment display I2C peripheral address, configuration register addresses,
 * and configuration register data
 */

uint8_t sevSeg_addr = (0x38 << 1);			//MAX5868 I2C address. 0x038 shifted left for the R/W' bit

const uint8_t sevSeg_decodeReg = 0x01;		//Address for decode register
const uint8_t sevSeg_decodeData = 0x0F;		//0b00001111 = decode hex for all segments
//Data buffer to send over I2C
uint8_t sevSeg_decodeBuffer[2] = {sevSeg_decodeReg, sevSeg_decodeData};

const uint8_t sevSeg_intensityReg = 0x02;		//Address for intensity register
// Intensity register takes 0bXX000000 to 0bXX111111 for 1/63 step intensity increments
// We declare 0% duty cycle, 50% duty cycle, and 100% duty cycle.
const uint8_t sevSeg_intensityDuty[3] = {0x00, 0x31, 0x63};
uint8_t sevSeg_intensityBuff[2] = {sevSeg_intensityReg, sevSeg_intensityDuty[1]};

const uint8_t sevSeg_SDReg = 0x04;			//Address for shutdown register
const uint8_t sevSeg_SD_ON = 0x01;			//Display ON - only mess with bit 0
const uint8_t sevSeg_SD_OFF = 0x00;			//Display OFF - only mess with bit 0
//Data buffer to send over I2C
uint8_t sevSeg_SD_ONBuff[10] = {sevSeg_SDReg, sevSeg_SD_ON};
uint8_t sevSeg_SD_OFFBuff[10] = {sevSeg_SDReg, sevSeg_SD_OFF};

const uint8_t sevSeg_testReg = 0x07;			//Address for display test
const uint8_t sevSeg_testOFF = 0x00;			//Display test OFF
const uint8_t sevSeg_testON = 0x01;			//Display test ON
//Data buffer to send over I2C
uint8_t sevSeg_testOFFBuff[2] = {sevSeg_testReg, sevSeg_testOFF};
uint8_t sevSeg_testONBuff[2] = {sevSeg_testReg, sevSeg_testON};

const uint8_t sevSeg_digit0Reg = 0x20;
const uint8_t sevSeg_digit1Reg = 0x21;
const uint8_t sevSeg_digit2Reg = 0x22;
const uint8_t sevSeg_digit3Reg = 0x23;


const uint8_t dispDigits[10] = {0x00, 0x01, 0x02, 0x03, 0x04,
	  	  	  	  	  	  	  	0x05, 0x06, 0x07, 0x08, 0x09};

//Data buffers to send to each individual LED display
uint8_t sevSeg_digit0Buff[2] = {sevSeg_digit0Reg, dispDigits[0]};
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

	HAL_StatusTypeDef halRetI2C;

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

	sevSeg_intensityBuff[1] = sevSeg_intensityDuty[1];		// Initialize to 50% duty cycle
	halRetI2C = HAL_I2C_Master_Transmit(hi2c1, sevSeg_addr, sevSeg_intensityBuff, 2, HAL_MAX_DELAY);

	if(halRetI2C != HAL_OK) {		//check HAL
		printf("HAL Error - TX intensity level data\n\r");
	} else {
		printf("Intensity Set\n\r");
	}

	return;

}

