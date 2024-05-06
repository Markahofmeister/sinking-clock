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


uint8_t capTouch_Init(QT1070 *capTouch, I2C_HandleTypeDef *hi2c, uint8_t keyEnFlags) {

	HAL_StatusTypeDef halRet = HAL_OK;

	capTouch->hi2c = hi2c;

	// Verify device ID
	uint8_t deviceIDRet = 0x00;
	halRet = capTouch_ReadDeviceID(capTouch, &deviceIDRet);

	if(deviceIDRet != DEVICE_ID || halRet != HAL_OK) {
		return 1;
	}

	capTouch->deviceID = deviceIDRet;

	// Force Device Recalibration
	halRet = capTouch_Recalibrate(capTouch);
	if(halRet != HAL_OK) {
		return 2;
	}

	// Wait until calibration sequence completes
	while(capTouch_checkCal(capTouch)) {}

	// Get initial reading of channels
	uint8_t keyStatus = 0x00;
	halRet = capTouch_readChannels(capTouch, &keyStatus);
	if(halRet != HAL_OK) {
		return 3;
	}

	uint8_t avgRet[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	uint8_t avgRegs[1] = {capTouch_AvgFactor0Reg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
								&(avgRegs[0]), 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, avgRet, 7, HAL_MAX_DELAY);

//	halRet = capTouch_enableKeys(capTouch, keyEnFlags);

	return 0;

}

HAL_StatusTypeDef capTouch_ReadDeviceID(QT1070 *capTouch, uint8_t *dataBuff) {

	uint8_t deviceIDRet_I2C = 0x00;

	HAL_StatusTypeDef halRet = HAL_OK;

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
										capTouch_DeviceIDReg, 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &deviceIDRet_I2C, 1, HAL_MAX_DELAY);

	*dataBuff = deviceIDRet_I2C;

	return halRet;

}

HAL_StatusTypeDef capTouch_Recalibrate(QT1070 *capTouch) {

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 * Writing any non-zero value to the calibrate register
	 * will enter a recalibration mode
	 */

	uint8_t deviceCal[2] = {capTouch_CalibrateReg, 0xFF};
	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									deviceCal, 1, HAL_MAX_DELAY);

	return halRet;

}

uint8_t capTouch_checkCal(QT1070 *capTouch) {

	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t detectionStatusRet = 0x00;
	uint8_t calibrationFlag;

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									(uint8_t*)capTouch_DetectionStatusReg, 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &detectionStatusRet, 1, HAL_MAX_DELAY);

	calibrationFlag = (detectionStatusRet & 0b10000000) >> 7;

	return calibrationFlag;

}

HAL_StatusTypeDef capTouch_readChannels(QT1070 *capTouch, uint8_t *dataBuff) {

	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t keyStatusRet = 0x00;

	uint8_t keyStatReg[1] = {capTouch_KeyStatusReg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									&(keyStatReg[0]), 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &keyStatusRet, 1, HAL_MAX_DELAY);

	*dataBuff = keyStatusRet;

	return halRet;

}

HAL_StatusTypeDef capTouch_enableKeys(QT1070 *capTouch, uint8_t dataBuff) {

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 *  Read in current averaging values for each key
	 *  The device will automatically increment register addressing for subsequent reads,
	 *  so only one Master Transmit call is required.
	 */
	uint8_t avgRet[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
							(uint8_t*)capTouch_AvgFactor0Reg, 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, avgRet, 7, HAL_MAX_DELAY);

	uint8_t avgNew[7];

	uint8_t i;
	for(i = 0; i <= 6; i++) {

		avgNew[i] = avgRet[i] * ((dataBuff >> i) & 0b00000001);

	}



	return halRet;

}


