/*
 * AT42QT1070.c
 *
 * I2C driver for Atmel AT42QT1070 capacitive touch sensor
 *
 *  Created on: May 5, 2024
 *      Author: Mark Hofmeister
 *
 */

#include "AT42QT1070.h"


uint8_t capTouch_Init(QT1070 *capTouch, I2C_HandleTypeDef *hi2c) {

	HAL_StatusTypeDef halRet = HAL_OK;

	capTouch->hi2c = hi2c;

	// Verify device ID
	uint8_t deviceIDRet = 0x00;

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
										capTouch_DeviceIDReg, 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &deviceIDRet, 1, HAL_MAX_DELAY);

	if(deviceIDRet != DEVICE_ID) {
		return 1;
	}

	// Force Device Recalibration
	uint8_t deviceCal[2] = {capTouch_CalibrateReg, 0xFF};
	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									deviceCal, 1, HAL_MAX_DELAY);

	/*
	 * Check that calibration sequence is complete
	 */
	uint8_t detectionStatusRet = 0x00;
	uint8_t calibrationFlag;
	do {

		halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
										(uint8_t*)capTouch_DetectionStatusReg, 1, HAL_MAX_DELAY);
		halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &detectionStatusRet, 1, HAL_MAX_DELAY);

		calibrationFlag = (detectionStatusRet & 0b10000000) >> 7;


	} while(calibrationFlag == 1);


	/*
	 * Get initial Reading of channels
	 */
	uint8_t keyStatusRet = 0x00;

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									(uint8_t*)capTouch_KeyStatusReg, 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &keyStatusRet, 1, HAL_MAX_DELAY);


	uint8_t avgRet[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t avgRegs[7] = {capTouch_AvgFactor0Reg, capTouch_AvgFactor1Reg, capTouch_AvgFactor2Reg, capTouch_AvgFactor3Reg,
							capTouch_AvgFactor4Reg, capTouch_AvgFactor5Reg, capTouch_AvgFactor6Reg};
	int i;
	for (i = 0; i <= 6; i++) {

		halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
								&(avgRegs[i]), 1, HAL_MAX_DELAY);
			halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &(avgRet[i]), 1, HAL_MAX_DELAY);

	}

	return 0;

}

HAL_StatusTypeDef capTouch_disableKeys(uint8_t keys) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}



