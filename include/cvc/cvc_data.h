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

#include <FreeRTOS.h>
#include <semphr.h>
#include <stm32f7xx.h>

// Mutex for CVC data
extern SemaphoreHandle_t CVC_DataMutex;

// Allows using names instead of indexes for CVC data
// Counts from 0
typedef enum {
    PLC_OUTPUTS = 0,
    PLC_INPUTS,
    CVC_RANDOM, // Random number for testing
    NUM_VALUES,
} CVC_data_id_t;

// Data types for CAN data, used for parsing
typedef enum {
    INT_10,    // Signed integer, divided by 10 for real value
    INT_100,   // Signed integer, divided by 100 for real value
    INT_1000,  // Signed integer, divided by 1000 for real value
    BOOLEAN,   // Boolean value, 0 or 1
    FLOAT,     // Float value
    INT,       // Signed integer, no division
    UINT,      // Unsigned integer, no division
} type_t;

// CAN data structure for storing different data types in one vector
typedef struct {
    type_t type;
    uint64_t data;
} CVC_data_t;

// Main CVC data vector
extern volatile CVC_data_t CVC_data[NUM_VALUES];

/**
 * @brief Initializes CVC data
 * @param None
 * @retval None
 */
void CVC_DataInit(void);

/**
 * @brief Converts CVC data from data vector to actual value as a float
 * @param CAN_data_id_t id: Index of CVC data to convert
 * @retval float: Converted value
 */
float CVC_DataToFloat(CVC_data_id_t id);

/**
 * @brief Converts CVC data from data vector to actual value as an unsigned integer
 * @param CAN_data_id_t id: Index of CVC data to convert
 * @return uint64_t
 */
uint64_t CVC_DataToUint(CVC_data_id_t id);

/**
 * @brief Converts CVC data from data vector to actual value as a signed integer
 * @param CAN_data_id_t id: Index of CVC data to convert
 * @return int32_t
 */
int32_t CVC_DataToInt(CVC_data_id_t id);

/**
 * @brief Stores CVC data in data vector
 * @param CVC_data_id_t id: ID of data to store
 * @param void *data: Data to store
 * @param uint8_t len: Length of data to store in bits
 * @param type_t type: Type of data to store
 * @retval None
 */
void CVC_SetData(CVC_data_id_t id, void *data, uint8_t len, type_t type);

/**
 * @brief Gets CVC data from data vector
 * @param CVC_data_id_t id: ID of data to get
 * @retval CVC_data_t: Data from data vector
 */
CVC_data_t CVC_GetData(CVC_data_id_t id);

/**
 * @brief Temporary function for testing CVC data functions
 * @param None
 * @retval None
 */
void CVC_TestSetGetData(void);

#endif /* INC_CVC_DATA_H_ */