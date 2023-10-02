/*
 * cvc_init.h
 *
 *  Created on: Sep 15, 2023
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_INIT_H_
#define INC_CVC_INIT_H_

/**
 * @brief Initalizes onboard LEDs.
 * @param argument: Not used
 * @retval None
 */
void CVC_LEDInit(void);

/**
 * @brief Initializes all the necessary peripherals for the CVC.
 * @param argument: Not used
 * @retval None
 */
void CVC_Init(void);

#endif /* INC_CVC_INIT_H_ */