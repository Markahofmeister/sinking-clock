/*
 * alarm.h
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 */

#ifndef INC_ALARM_H_
#define INC_ALARM_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

/*
 * Declare RTC time and date to defaults and update RTC with values
 */
void initRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate);

/*
 * Gets updated RTC time and stores in access object pointers
 */
void getRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate);

/*
 * Gets the time of user's alarm and stores in access object pointers
 */
void getUserAlarmTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *userAlarmTime);

/*
 * Updates entire alarm object with user alarm settings
 */
void getUserAlarmObj(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *userAlarmObj);

#endif /* INC_ALARM_H_ */
