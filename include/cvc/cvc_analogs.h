/*
 * cvc_analogs.h
 *
 *  Created on: Apr 20, 2024
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_ANALOGS_H
#define INC_CVC_ANALOGS_H

/**
 * @brief Starts ADC conversion.
 * @param None
 * @retval None
 */
void Analogs_Configure(void);

/**
 * @brief Reads throttle position from ADC and stores it in global CVC data.
 * @param None
 * @retval None
 */
void Analogs_ReadThrottle(void);

/**
 * @brief Reads LV voltage from ADC and stores it in global CVC data.
 * @param None
 * @retval None 
 */
void Analogs_ReadVoltage(void);

#endif /* INC_CVC_ANALOGS_H */