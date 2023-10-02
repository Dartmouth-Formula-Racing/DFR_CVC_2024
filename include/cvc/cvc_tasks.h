/*
 * cvc_tasks.h
 *
 *  Created on: Sep 14, 2023
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_TASKS_H_
#define INC_CVC_TASKS_H_

/**
 * @brief Thread for communication functions.
 * @param argument: Not used
 * @retval None
 */
void Communication(void *argument);

/**
 * @brief Thread for interpreting communication data.
 * @param argument: Not used
 * @retval None
 */
void CommunicationProcessing(void *argument);

/**
 * @brief Thread for control functions.
 * @param argument: Not used
 * @retval None
 */
void Control(void *argument);

#endif /* INC_CVC_TASKS_H_ */