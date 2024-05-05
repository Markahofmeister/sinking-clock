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

	return 0;

}

