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
//
//#ifndef STM32G0XX_HAL_CONF_H_
//#include "stm32g0xx_hal_conf.h"
//#endif

/*
 * Cap. touch IC hard-coded values
 */

#define DEVICE_ADDRESS 0x1B << 1
#define DEVICE_ID 0x2E


/*
 * Internal Register Address Allocations
 */

 //Device ID/Status Registers
#define capTouch_DeviceIDReg 0x00
#define capTouch_FWReg 0x01
#define capTouch_DetectionStatusReg 0x02
#define capTouch_KeyStatusReg 0x03

// Key Signal Registers, 0-6
#define capTouch_KeySig0Reg 0x04
#define capTouch_KeySig1Reg 0x06
#define capTouch_KeySig2Reg 0x08
#define capTouch_KeySig3Reg 0x0A
#define capTouch_KeySig4Reg 0x0C
#define capTouch_KeySig5Reg 0x0E
#define capTouch_KeySig6Reg 0x10

// Reference Data Registers, 0-6
#define capTouch_ReferenceData0Reg 0x12
#define capTouch_ReferenceData1Reg 0x14
#define capTouch_ReferenceData2Reg 0x16
#define capTouch_ReferenceData3Reg 0x18
#define capTouch_ReferenceData4Reg 0x1A
#define capTouch_ReferenceData5Reg 0x1C
#define capTouch_ReferenceData6Reg 0x1E

// Negative Threshold Registers, 0-6
#define capTouch_NThr0Reg 0x20
#define capTouch_NThr1Reg 0x21
#define capTouch_NThr2Reg 0x22
#define capTouch_NThr3Reg 0x23
#define capTouch_NThr4Reg 0x24
#define capTouch_NThr5Reg 0x25
#define capTouch_NThr6Reg 0x26

// Averaging Factor/Adjacent Key Suppression Registers, 0-6
#define capTouch_AvgFactor0Reg 0x27
#define capTouch_AvgFactor1Reg 0x28
#define capTouch_AvgFactor2Reg 0x29
#define capTouch_AvgFactor3Reg 0x2A
#define capTouch_AvgFactor4Reg 0x2B
#define capTouch_AvgFactor5Reg 0x2C
#define capTouch_AvgFactor6Reg 0x2D

// Detection Integrator Registers, 0-6
#define capTouch_DetInteg0Reg 0x2E
#define capTouch_DetInteg1Reg 0x2F
#define capTouch_DetInteg2Reg 0x30
#define capTouch_DetInteg3Reg 0x31
#define capTouch_DetInteg4Reg 0x32
#define capTouch_DetInteg5Reg 0x33
#define capTouch_DetInteg6Reg 0x34

// Calibration/Utility Registers
#define capTouch_FOMOGuargReg 0x35
#define capTouch_LowPowerReg 0x36
#define capTouch_MaxOnDurReg 0x37
#define capTouch_CalibrateReg 0x38
#define capTouch_RESETReg 0x39


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
