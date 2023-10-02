/*
 * cvc_can.h
 *  This file contains global variables for the CVC to use.
 *  Received and parsed CAN data is stored here publicly.
 *  This file also contains parsing functions for CAN data.
 *  Created on: Oct 2, 2023
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_DATA_H_
#define INC_CVC_DATA_H_

#include <stm32f7xx.h>
#include <FreeRTOS.h>
#include <semphr.h>

// Mutex for CVC data
extern SemaphoreHandle_t CVC_DataMutex;

// Allows using names instead of indexes for CVC data
// Counts from 0
typedef enum {
    PLC_OUTPUTS = 0,
    PLC_INPUTS,

    NUM_VALUES,
} CVC_data_id_t;

// Main CVC data vector
extern volatile uint32_t CVC_data[NUM_VALUES];

/**
 * @brief Initializes CVC data
 * @param None
 * @retval None
 */
void CVC_DataInit(void);

#endif /* INC_CVC_DATA_H_ */