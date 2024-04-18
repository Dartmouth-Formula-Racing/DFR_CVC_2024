/*
 * cvc_analogs.h
 *
 *  Created on: Apr 16, 2024
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_ANALOGS_H_
#define INC_CVC_ANALOGS_H_

#include <stm32f767xx.h>

// Enum for identifying analog inputs
typedef enum {
    Throttle1 = 0,
    LVVoltage,
} CVC_analog_id_t;

/**
 * @brief Function for reading analog input
 * @param id - analog input to read
 * @return 12-bit ADC value 
 */
uint32_t CVC_Analog_Read(CVC_analog_id_t id);

#endif /* INC_CVC_ANALOGS_H_ */