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
    // === AEM 30-22O6 VDM ===
    // GPS Data
    VDM_GPS_LATITUDE, 
    VDM_GPS_LONGITUDE, 
    VDM_GPS_SPEED,
    VDM_GPS_ALTITUDE, 
    VDM_GPS_TRUE_COURSE, 
    VDM_GPS_SATELLITES_IN_USE, 
    VDM_GPS_DATA_VALID,
    // GPS Date Time Parameters
    VDM_GPS_DATE_TIME_VALID, 
    VDM_UTC_YEAR, 
    VDM_UTC_MONTH,
    VDM_UTC_DAY,
    VDM_UTC_HOUR,
    VDM_UTC_MINUTE,
    VDM_UTC_SECOND, 
    // Acceleration data
    VDM_ACCELERATION_X,
    VDM_ACCELERATION_Y,
    VDM_ACCELERATION_Z,
    // Yaw rate Data
    VDM_YAW_RATE_X,
    VDM_YAW_RATE_Y,
    VDM_YAW_RATE_Z,
    // Events - TODO: Figure out how to handle this


    // === Inverter ===
    //  Temperature Parameters
    INVERTER1_POWER_MODULE_A_TEMP,
    INVERTER1_POWER_MODULE_B_TEMP,
    INVERTER1_POWER_MODULE_C_TEMP,
    INVERTER1_GATE_DRIVER_BOARD_TEMP, 
    INVERTER1_CONTROL_BOARD_TEMP,
    INVERTER1_RTD_INPUT_1_TEMP,
    INVERTER1_RTD_INPUT_2_TEMP,
    INVERTER1_STALL_BURST_MODEL_TEMP,
    INVERTER1_COOLANT_TEMP,
    INVERTER1_HOT_SPOT_TEMP,
    INVERTER1_MOTOR_TEMP,
    INVERTER2_POWER_MODULE_A_TEMP,
    INVERTER2_POWER_MODULE_B_TEMP,
    INVERTER2_POWER_MODULE_C_TEMP,
    INVERTER2_GATE_DRIVER_BOARD_TEMP, 
    INVERTER2_CONTROL_BOARD_TEMP,
    INVERTER2_RTD_INPUT_1_TEMP,
    INVERTER2_RTD_INPUT_2_TEMP,
    INVERTER2_STALL_BURST_MODEL_TEMP,
    INVERTER2_COOLANT_TEMP,
    INVERTER2_HOT_SPOT_TEMP,
    INVERTER2_MOTOR_TEMP,
    // Torque Parameters
    INVERTER1_TORQUE_SHUDDER,
    INVERTER2_TORQUE_SHUDDER,
    // Digital Input Status Parameters
    INVERTER1_FORWARD_SWITCH,
    INVERTER1_REVERSE_SWITCH,
    INVERTER1_BRAKE_SWITCH,
    INVERTER1_REGEN_DISABLE_SWITCH,
    INVERTER1_IGNITION_SWITCH,
    INVERTER1_START_SWITCH,
    INVERTER1_VALET_MODE,
    INVERTER2_FORWARD_SWITCH,
    INVERTER2_REVERSE_SWITCH,
    INVERTER2_BRAKE_SWITCH,
    INVERTER2_REGEN_DISABLE_SWITCH,
    INVERTER2_IGNITION_SWITCH,
    INVERTER2_START_SWITCH,
    INVERTER2_VALET_MODE,
    // Motor Position Parameters
    INVERTER1_MOTOR_ANGLE, 
    INVERTER1_MOTOR_SPEED,
    INVERTER1_ELECTRICAL_OUTPUT_FREQUENCY,
    INVERTER1_DELTA_RESOLVER_FILTERED,
    INVERTER2_MOTOR_ANGLE, 
    INVERTER2_MOTOR_SPEED,
    INVERTER2_ELECTRICAL_OUTPUT_FREQUENCY,
    INVERTER2_DELTA_RESOLVER_FILTERED,
    // Current Partameters
    INVERTER1_PHASE_A_CURRENT,
    INVERTER1_PHASE_B_CURRENT,
    INVERTER1_PHASE_C_CURRENT,
    INVERTER1_DC_BUS_CURRENT,
    INVERTER2_PHASE_A_CURRENT,
    INVERTER2_PHASE_B_CURRENT,
    INVERTER2_PHASE_C_CURRENT,
    INVERTER2_DC_BUS_CURRENT,
    // Voltage Parameters
    INVERTER1_DC_BUS_VOLTAGE,
    INVERTER1_OUTPUT_VOLTAGE, 
    INVERTER1_VAB_VD_VOLTAGE,
    INVERTER1_VAB_VQ_VOLTAGE,
    INVERTER2_DC_BUS_VOLTAGE,
    INVERTER2_OUTPUT_VOLTAGE, 
    INVERTER2_VAB_VD_VOLTAGE,
    INVERTER2_VAB_VQ_VOLTAGE,
    // Flux Parameters
    INVERTER1_FLUX_COMMAND,
    INVERTER1_FLUX_FEEDBACK, 
    INVERTER1_ID_CURRENT,
    INVERTER1_IQ_CURRENT,
    INVERTER2_FLUX_COMMAND,
    INVERTER2_FLUX_FEEDBACK, 
    INVERTER2_ID_CURRENT,
    INVERTER2_IQ_CURRENT,
    // Internal Voltage Parameters
    INVERTER1_1_5_REFERENCE_VOLTAGE,
    INVERTER1_2_5_REFERENCE_VOLTAGE,
    INVERTER1_5_0_REFERENCE_VOLTAGE,
    INVERTER1_12_0_REFERENCE_VOLTAGE,
    INVERTER2_1_5_REFERENCE_VOLTAGE,
    INVERTER2_2_5_REFERENCE_VOLTAGE,
    INVERTER2_5_0_REFERENCE_VOLTAGE,
    INVERTER2_12_0_REFERENCE_VOLTAGE,
    // Internal State Parameters
    INVERTER1_VSM_STATE,
    INVERTER1_PWM_FREQUENCY,
    INVERTER1_STATE,
    INVERTER1_RELAY_1_STATUS,
    INVERTER1_RELAY_2_STATUS,
    INVERTER1_RELAY_3_STATUS,
    INVERTER1_RELAY_4_STATUS,
    INVERTER1_RELAY_5_STATUS,
    INVERTER1_RELAY_6_STATUS,
    INVERTER1_RUN_MODE,
    INVERTER1_SELF_SENSING_ASSIST,
    INVERTER1_ACTIVE_DISCHARGE_STATE,
    INVERTER1_COMMAND_MODE,
    INVERTER1_ROLLING_COUNTER_VALUE,
    INVERTER1_ENABLE_STATE,
    INVERTER1_BURST_MODEL_MODE,
    INVERTER1_START_MODE_ACTIVE,
    INVERTER1_ENABLE_LOCKOUT,
    INVERTER1_DIRECTION_COMMAND,
    INVERTER1_BMS_ACTIVE,
    INVERTER1_BMS_LIMITING_TORQUE,
    INVERTER1_LIMIT_MAX_SPEED,
    INVERTER1_LIMIT_HOT_SPOT,
    INVERTER1_LOW_SPEED_LIMITING,
    INVERTER1_COOLANT_TEMP_LIMITING,
    INVERTER1_LIMIT_STALL_BURST_MODEL,
    INVERTER2_VSM_STATE,
    INVERTER2_PWM_FREQUENCY,
    INVERTER2_STATE,
    INVERTER2_RELAY_1_STATUS,
    INVERTER2_RELAY_2_STATUS,
    INVERTER2_RELAY_3_STATUS,
    INVERTER2_RELAY_4_STATUS,
    INVERTER2_RELAY_5_STATUS,
    INVERTER2_RELAY_6_STATUS,
    INVERTER2_RUN_MODE,
    INVERTER2_SELF_SENSING_ASSIST,
    INVERTER2_ACTIVE_DISCHARGE_STATE,
    INVERTER2_COMMAND_MODE,
    INVERTER2_ROLLING_COUNTER_VALUE,
    INVERTER2_ENABLE_STATE,
    INVERTER2_BURST_MODEL_MODE,
    INVERTER2_START_MODE_ACTIVE,
    INVERTER2_ENABLE_LOCKOUT,
    INVERTER2_DIRECTION_COMMAND,
    INVERTER2_BMS_ACTIVE,
    INVERTER2_BMS_LIMITING_TORQUE,
    INVERTER2_LIMIT_MAX_SPEED,
    INVERTER2_LIMIT_HOT_SPOT,
    INVERTER2_LOW_SPEED_LIMITING,
    INVERTER2_COOLANT_TEMP_LIMITING,
    INVERTER2_LIMIT_STALL_BURST_MODEL,
    // Fault codes
    INVERTER1_POST_FAULT_LO,
    INVERTER1_POST_FAULT_HI, 
    INVERTER1_RUN_FAULT_LO, 
    INVERTER1_RUN_FAULT_HI,
    INVERTER2_POST_FAULT_LO,
    INVERTER2_POST_FAULT_HI, 
    INVERTER2_RUN_FAULT_LO, 
    INVERTER2_RUN_FAULT_HI,
    // High Speed Parameters
    INVERTER1_TORQUE_COMMAND_HI,
    INVERTER1_TORQUE_FEEDBACK_HI,
    INVERTER1_MOTOR_SPEED_HI,
    INVERTER1_DC_BUS_VOLTAGE_HI,
    INVERTER2_TORQUE_COMMAND_HI,
    INVERTER2_TORQUE_FEEDBACK_HI,
    INVERTER2_MOTOR_SPEED_HI,
    INVERTER2_DC_BUS_VOLTAGE_HI,
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