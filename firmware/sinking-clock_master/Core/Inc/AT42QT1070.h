/*
 * AT42QT1070.h
 *
 * I2C driver for Atmel AT42QT1070 capacitive touch sensor
 *
 *  Created on: May 5, 2024
 *      Author: Mark Hofmeister
 */

#ifndef INC_AT42QT1070_H_
#define INC_AT42QT1070_H_

//#ifndef MAIN_H
//#include "main.h"
//#endif

#ifndef STM32G0XX_HAL_H_
#include "stm32g0xx_hal.h"
#endif

//#ifndef STM32G0XX_HAL_CONF_H_
//#include "stm32g0xx_hal_conf.h"
//#endif

/*
 * Cap. touch IC hard-coded values
 */

#define DEVICE_ADDRESS 0x1B
#define DEVICE_ID 0x2E


/*
 * Internal Register Address Allocations
 */

// Device ID/Status Registers
const uint8_t capTouch_DeviceIDReg = 0x00;
const uint8_t capTouch_FWReg = 0x01;
const uint8_t capTouch_DetectionStatusReg = 0x02;
const uint8_t capTouch_KeyStatusReg = 0x03;

// Key Signal Registers, 0-6
const uint8_t capTouch_KeySig0Reg = 0x04;
const uint8_t capTouch_KeySig1Reg = 0x06;
const uint8_t capTouch_KeySig2Reg = 0x08;
const uint8_t capTouch_KeySig3Reg = 0x0A;
const uint8_t capTouch_KeySig4Reg = 0x0C;
const uint8_t capTouch_KeySig5Reg = 0x0E;
const uint8_t capTouch_KeySig6Reg = 0x10;

// Reference Data Registers, 0-6
const uint8_t capTouch_ReferenceData0Reg = 0x12;
const uint8_t capTouch_ReferenceData1Reg = 0x14;
const uint8_t capTouch_ReferenceData2Reg = 0x16;
const uint8_t capTouch_ReferenceData3Reg = 0x18;
const uint8_t capTouch_ReferenceData4Reg = 0x1A;
const uint8_t capTouch_ReferenceData5Reg = 0x1C;
const uint8_t capTouch_ReferenceData6Reg = 0x1E;

// Negative Threshold Registers, 0-6
const uint8_t capTouch_NThr0Reg = 0x20;
const uint8_t capTouch_NThr1Reg = 0x21;
const uint8_t capTouch_NThr2Reg = 0x22;
const uint8_t capTouch_NThr3Reg = 0x23;
const uint8_t capTouch_NThr4Reg = 0x24;
const uint8_t capTouch_NThr5Reg = 0x25;
const uint8_t capTouch_NThr6Reg = 0x26;

// Averaging Factor/Adjacent Key Suppression Registers, 0-6
const uint8_t capTouch_AvgFactor0Reg = 0x27;
const uint8_t capTouch_AvgFactor1Reg = 0x28;
const uint8_t capTouch_AvgFactor2Reg = 0x29;
const uint8_t capTouch_AvgFactor3Reg = 0x2A;
const uint8_t capTouch_AvgFactor4Reg = 0x2B;
const uint8_t capTouch_AvgFactor5Reg = 0x2C;
const uint8_t capTouch_AvgFactor6Reg = 0x2D;

// Detection Integrator Registers, 0-6
const uint8_t capTouch_DetInteg0Reg = 0x2E;
const uint8_t capTouch_DetInteg1Reg = 0x2F;
const uint8_t capTouch_DetInteg2Reg = 0x30;
const uint8_t capTouch_DetInteg3Reg = 0x31;
const uint8_t capTouch_DetInteg4Reg = 0x32;
const uint8_t capTouch_DetInteg5Reg = 0x33;
const uint8_t capTouch_DetInteg6Reg = 0x34;

// Calibration/Utility Registers
const uint8_t capTouch_FOMOGuargReg = 0x35;
const uint8_t capTouch_LowPowerReg = 0x36;
const uint8_t capTouch_MaxOnDurReg = 0x37;
const uint8_t capTouch_CalibrateReg = 0x38;
const uint8_t capTouch_RESETReg = 0x39;


typedef struct {

	I2C_HandleTypeDef *hi2c;

} QT1070;

/*
 * Sensor Initialization
 * QT1070 *capTouch = Pointer to capcitive touch struct handle
 * I2C_HandleTypeDef *hi2c = I2C handle through which to talk to cap. touch sensor
 *
 * Error Codes:
 * 		1 = Error Reading device ID
 */
HAL_StatusTypeDef capTouch_Init(QT1070 *capTouch, I2C_HandleTypeDef *hi2c);




#endif /* INC_AT42QT1070_H_ */
