/*
 * alarm.c
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 */

#include "../Inc/alarm.h"


void initRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate) {

	currTime->Hours = 1;				// Initialize current RTC time to default values
	currTime->Minutes = 00;
	currTime->Seconds = 00;
	currTime->TimeFormat = RTC_HOURFORMAT12_AM;			//This is initially in the A.M., so P.M. LED is off.

	currDate->Year = 0;					// Initialize current RTC date to default values
	currDate->Month = RTC_MONTH_JANUARY;
	currDate->Date = 0;

	HAL_StatusTypeDef halRet = HAL_OK;
	halRet = HAL_RTC_SetTime(hrtc, currTime, RTCTimeFormat);
	halRet = HAL_RTC_SetDate(hrtc, currDate, RTCTimeFormat);

//	if(halRet == HAL_OK) {
//		printf("Current time defaulted to: %u:%u:%u\n\r", currTime->Hours, currTime->Minutes, currTime->Seconds);
//	}
//	else {
//		printf("Error defaulting RTC time.\n\r");
//	}

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

	halRet = HAL_RTC_SetAlarm_IT(hrtc, &internalAlarm_init, RTCTimeFormat);

	RTC_AlarmTypeDef internalAlarm_initTest;
	halRet = HAL_RTC_GetAlarm(hrtc, &internalAlarm_initTest, internalAlarm, RTCTimeFormat);

//	if(halRet == HAL_OK) {
//		printf("Internal alarm A defaulted to %u:%u:%u.\n\r", internalAlarm_initTest.AlarmTime.Hours,
//				internalAlarm_initTest.AlarmTime.Minutes, internalAlarm_initTest.AlarmTime.Seconds);
//	}

}

void getRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate) {

	// Store both current time and current date in time and date pointers.
	HAL_RTC_GetTime(hrtc, currTime, RTCTimeFormat);
	HAL_RTC_GetDate(hrtc, currDate, RTCTimeFormat);

}

