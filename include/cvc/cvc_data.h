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
// Yes this is incredibly inefficient but this
// only uses a fraction of our available memory.
typedef enum {
    // === EMUS BMS ===
    // Overall Parameters
    BMS_IGNITION,
    BMS_CHARGER_MAINS,
    BMS_FAST_CHARGE,
    BMS_LEAKAGE_DETECTED,
    BMS_CHARGER_ENABLED,
    BMS_HEATER_ENABLED,
    BMS_BATTERY_CONTACTOR,
    BMS_BATTERY_FAN,
    BMS_POWER_REDUCTION,
    BMS_CHARGING_INTERLOCK,
    BMS_DCDC_ENABLED,
    BMS_CONTACTOR_PRECHARGE,
    BMS_CHARGING_STATE,
    BMS_CHARGING_DURATION,
    BMS_LAST_CHARGING_ERROR,
    BMS_LIVE_CELL_COUNT,
    // Diagnostic Codes
    BMS_UNDERVOLTAGE,
    BMS_OVERVOLTAGE,
    BMS_DISCHARGE_OVERCURRENT,
    BMS_CHARGE_OVERCURRENT,
    BMS_CELL_MODULE_OVERHEAT,
    BMS_LEAKAGE,
    BMS_NO_CELL_COMMUNICATION,
    BMS_WARN_LOW_CELL_VOLTAGE,
    BMS_WARN_HIGH_DISCHARGE_CURRENT,
    BMS_WARN_HIGH_CELL_MODULE_TEMP,
    BMS_CELL_OVERHEAT,
    BMS_NO_CURRENT_SENSOR,
    BMS_PACK_UNDERVOLTAGE,
    BMS_CELL_VOLTAGE_VALID,
    BMS_CELL_MODULE_TEMP_VALID,
    BMS_BALANCE_RATE_VALID,
    BMS_LIVE_CELL_COUNT_VALID,
    BMS_CHARGING_FINISHED,
    BMS_CELL_TEMP_VALID,
    // Battery Voltage Overall Parameters
    BMS_MIN_CELL_VOLTAGE,
    BMS_MAX_CELL_VOLTAGE,
    BMS_AVG_CELL_VOLTAGE,
    BMS_TOTAL_VOLTAGE,
    // Cell Module Temperature Overall Parameters
    BMS_MIN_CELL_MODULE_TEMP,
    BMS_MAX_CELL_MODULE_TEMP,
    BMS_AVG_CELL_MODULE_TEMP,
    // Cell Module Temperature Overall Parameters
    BMS_MIN_CELL_TEMP,
    BMS_MAX_CELL_TEMP,
    BMS_AVG_CELL_TEMP,
    // Cell Balancing Rate Overall Parameters
    BMS_MIN_BALANCE_RATE,
    BMS_MAX_BALANCE_RATE,
    BMS_AVG_BALANCE_RATE,
    // State of Charge Parameters
    BMS_CURRENT,
    BMS_ESTIMATED_CHARGE,
    BMS_ESTIMATED_SOC,
    // Configuration Parameters - TODO: Figure out how to handle this
    // Contactor Control
    BMS_CONTACTOR_STATE,
    // Energy Parameters
    BMS_ESTIMATED_CONSUMPTION,
    BMS_ESTIMATED_ENERGY,
    BMS_ESTIMATED_DISTANCE_REMAINING,
    BMS_DISTANCE_TRAVELED,
    // Events - TODO: Figure out how to handle this
    // === PLC ===
    PLC_OUTPUTS,
    PLC_INPUTS,
    LED_STATE,
    // === Length of CVC data array ===
    // This must be the last value in the enum
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