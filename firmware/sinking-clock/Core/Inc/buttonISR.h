/*
 * buttonISR.h
 *
 *  Created on: Jun 21, 2023
 *      Author: marka
 *
 *  (desc)
 */

#ifndef INC_BUTTONISR_H_
#define INC_BUTTONISR_H_

#include "../Src/buttonISR.c"

/*
 * @brief Simple decision tree to call ISRs depending on which GPIO button was pressed
 *
 * @param unsigned 16-bit integer representing GPIO pin value
 * @retval none
 */
void callISR(uint16_t GPIO_Pin);

/*
 * @brief toggles display from 50% brightness, 100% brightness, or 0% brightness (fully off)
 *
 * @param
 * @retval
 */

void displayISR();

/*
 * @brief Toggles whether or not alarm is enabled, as well as alarm LED.
 *
 * @param
 * @retval
 */
void alarmENISR();

/*
 * @brief Enters a loop in which the user can set the time of the alarm.
 * Calls ISRs for hour set and minute set.
 *
 * @param
 * @retval
 */
void alarmSetISR();

/*
 * @brief Increments the hour of either the current time or alarm time.
 *
 * @param
 * @retval
 */
void hourSetISR();

/*
 * @brief Increments the hour of either the current time or alarm time.
 *
 * @param
 * @retval
 */
void minuteSetISR();


#endif /* INC_BUTTONISR_H_ */
