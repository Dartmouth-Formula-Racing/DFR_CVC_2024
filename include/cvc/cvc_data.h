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


    // === Inverter ===
    //  Temperature Parameters
    INVERTER_POWER_MODULE_A_TEMP,
    INVERTER_POWER_MODULE_B_TEMP,
    INVERTER_POWER_MODULE_C_TEMP,
    INVERTER_GATE_DRIVER_BOARD_TEMP, 
    INVERTER_CONTROL_BOARD_TEMP,
    INVERTER_RTD_INPUT_1_TEMP,
    INVERTER_RTD_INPUT_2_TEMP,
    INVERTER_STALL_BURST_MODEL_TEMP,
    INVERTER_COOLANT_TEMP,
    INVERTER_HOT_SPOT_TEMP,
    INVERTER_MOTOR_TEMP,
    // Torque Parameters
    INVERTER_TORQUE_SHUDDER,
    // Digital Input Status Parameters
    INVERTER_FORWARD_SWITCH,
    INVERTER_REVERSE_SWITCH,
    INVERTER_BRAKE_SWITCH,
    INVERTER_REGEN_DISABLE_SWITCH,
    INVERTER_IGNITION_SWITCH,
    INVERTER_START_SWITCH,
    INVERTER_VALET_MODE,
    // Motor Position Parameters
    INVERTER_MOTOR_ANGLE, 
    INVERTER_MOTOR_SPEED,
    INVERTER_ELECTRICAL_OUTPUT_FREQUENCY,
    INVERTER_DELTA_RESOLVER_FILTERED,
    // Current Partameters
    INVERTER_PHASE_A_CURRENT,
    INVERTER_PHASE_B_CURRENT,
    INVERTER_PHASE_C_CURRENT,
    INVERTER_DC_BUS_CURRENT,
    // Voltage Parameters
    INVERTER_DC_BUS_VOLTAGE,
    INVERTER_OUTPUT_VOLTAGE, 
    INVERTER_VAB_VD_VOLTAGE,
    INVERTER_VAB_VQ_VOLTAGE,
    // Flux Parameters
    INVERTER_FLUX_COMMAND,
    INVERTER_FLUX_FEEDBACK, 
    INVERTER_ID_CURRENT,
    INVERTER_IQ_CURRENT,
    // Internal Voltage Parameters
    INVERTER_1_5_REFERENCE_VOLTAGE,
    INVERTER_2_5_REFERENCE_VOLTAGE,
    INVERTER_5_0_REFERENCE_VOLTAGE,
    INVERTER_12_0_REFERENCE_VOLTAGE,
    // Internal State Parameters
    INVERTER_VSM_STATE,
    INVERTER_PWM_FREQUENCY,
    INVERTER_STATE,
    INVERTER_RELAY_1_STATUS,
    INVERTER_RELAY_2_STATUS,
    INVERTER_RELAY_3_STATUS,
    INVERTER_RELAY_4_STATUS,
    INVERTER_RELAY_5_STATUS,
    INVERTER_RELAY_6_STATUS,
    INVERTER_RUN_MODE,
    INVERTER_SELF_SENSING_ASSIST,
    INVERTER_ACTIVE_DISCHARGE_STATE,
    INVERTER_COMMAND_MODE,
    INVERTER_ROLLING_COUNTER_VALUE,
    INVERTER_ENABLE_STATE,
    INVERTER_BURST_MODEL_MODE,
    INVERTER_START_MODE_ACTIVE,
    INVERTER_ENABLE_LOCKOUT,
    INVERTER_DIRECTION_COMMAND,
    INVERTER_BMS_ACTIVE,
    INVERTER_BMS_LIMITING_TORQUE,
    INVERTER_LIMIT_MAX_SPEED,
    INVERTER_LIMIT_HOT_SPOT,
    INVERTER_LOW_SPEED_LIMITING,
    INVERTER_COOLANT_TEMP_LIMITING,
    INVERTER_LIMIT_STALL_BURST_MODEL,
    // Fault codes
    INVERTER_POST_FAULT_LO,
    INVERTER_POST_FAULT_HI, 
    INVERTER_RUN_FAULT_LO, 
    INVERTER_RUN_FAULT_HI,
    // High Speed Parameters
    INVERTER_TORQUE_COMMAND_HI,
    INVERTER_TORQUE_FEEDBACK_HI,
    INVERTER_MOTOR_SPEED_HI,
    INVERTER_DC_BUS_VOLTAGE_HI,

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