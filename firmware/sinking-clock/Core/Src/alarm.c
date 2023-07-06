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

	currTime->Hours = 1;				// Initialize current RTC time to default values
	currTime->Minutes = 00;
	currTime->Seconds = 00;
	currTime->TimeFormat = RTC_HOURFORMAT12_AM;			//This is initially in the A.M., so P.M. LED is off.

	currDate->Year = 0;					// Initialize current RTC date to default values
	currDate->Month = RTC_MONTH_JANUARY;
	currDate->Date = 0;

	HAL_StatusTypeDef halRet = HAL_OK;
	halRet = HAL_RTC_SetTime(hrtc, currTime, RTC_FORMAT_BCD);
	halRet = HAL_RTC_SetDate(hrtc, currDate, RTC_FORMAT_BCD);

	if(halRet == HAL_OK) {
		printf("Current time defaulted to: %u:%u:%u\n\r", currTime->Hours, currTime->Minutes, currTime->Seconds);
	}
	else {
		printf("Error defaulting RTC time.\n\r");
	}

	RTC_AlarmTypeDef internalAlarm_init = {0};
	internalAlarm_init.AlarmTime.Hours = currTime->Hours;
	internalAlarm_init.AlarmTime.Minutes = currTime->Minutes + 1;
	internalAlarm_init.AlarmTime.Seconds = currTime->Seconds;
	internalAlarm_init.AlarmTime.SubSeconds = currTime->SubSeconds;
	internalAlarm_init.AlarmTime.TimeFormat = currTime->TimeFormat;
	internalAlarm_init.AlarmTime.TimeFormat = currTime->TimeFormat;
	internalAlarm_init.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	internalAlarm_init.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	internalAlarm_init.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
								  |RTC_ALARMMASK_SECONDS;
	internalAlarm_init.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	internalAlarm_init.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	internalAlarm_init.AlarmDateWeekDay = 0x1;
	internalAlarm_init.Alarm = internalAlarm;

	halRet = HAL_RTC_SetAlarm_IT(hrtc, &internalAlarm_init, RTC_FORMAT_BCD);

	RTC_AlarmTypeDef internalAlarm_initTest;
	HAL_RTC_GetAlarm(hrtc, &internalAlarm_initTest, internalAlarm, RTC_FORMAT_BCD);

	if(halRet == HAL_OK) {
		printf("Internal alarm A defaulted to %u:%u:%u.\n\r", internalAlarm_initTest.AlarmTime.Hours,
				internalAlarm_initTest.AlarmTime.Minutes, internalAlarm_initTest.AlarmTime.Seconds);
	}

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


