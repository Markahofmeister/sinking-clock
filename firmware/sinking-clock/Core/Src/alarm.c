/*
 * alarm.c
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 */

#include "../Inc/alarm.h"

/*
 * Alarm specification macros
 */
#define internalAlarm 	RTC_ALARM_A
#define userAlarm 		RTC_ALARM_B

void initRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate) {

	currTime->Hours = 12;				// Initialize current RTC time to default values
	currTime->Minutes = 58;
	currTime->Seconds = 50;
	currTime->TimeFormat = RTC_HOURFORMAT12_AM;			//This is initially in the A.M., so P.M. LED is off.

	currDate->Year = 0;					// Initialize current RTC date to default values
	currDate->Month = RTC_MONTH_JANUARY;
	currDate->Date = 0;

	HAL_RTC_SetTime(hrtc, currTime, RTC_FORMAT_BCD);
	HAL_RTC_SetDate(hrtc, currDate, RTC_FORMAT_BCD);

	printf("Current time defaulted to: %d:%d:%d\n\r", currTime->Hours, currTime->Minutes, currTime->Seconds);

}

void getRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate) {

	// Store both current time and current date in time and date pointers.
	HAL_RTC_GetTime(hrtc, currTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(hrtc, currDate, RTC_FORMAT_BCD);

}

void getUserAlarmTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *userAlarmTime) {

	// Store alarm data in alarm object pointer and extract alarm time data from alarm object
	RTC_AlarmTypeDef userAlarmObj;
	HAL_RTC_GetAlarm(hrtc, &userAlarmObj, userAlarm, RTC_FORMAT_BCD);
	*userAlarmTime = userAlarmObj.AlarmTime;

}

void getUserAlarmObj(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *userAlarmObj) {

	// Store alarm data in alarm object pointer
	HAL_RTC_GetAlarm(hrtc, userAlarmObj, internalAlarm, FORMAT_BIN);

}


