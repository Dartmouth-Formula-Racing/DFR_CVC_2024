/*
 * cvc_control.h
 *
 *  Created on: Apr 24, 2024
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_CONTROL_H
#define INC_CVC_CONTROL_H

#define CONTROL_TASK_MS 10
#define CONTROL_TORQUECOMMAND_MS CONTROL_TASK_MS

#define THROTTLE_INVALID_TIMER 1000                 // Time before throttle is marked invalid
#define MAX_THROTTLE_DIFFERENCE (int)(0.05 * 4095)  // 5% of 12-bit ADC range
#define MIN_THROTTLE_RANGE 520                      // Lower bound for valid throttle range
#define MAX_THROTTLE_RANGE 4096                     // Upper bound for valid throttle range
#define MIN_THROTTLE 700                            // 0% throttle
#define MAX_THROTTLE 3900                           // 100% throttle

#define MAX_THROTTLE_CONT_VALUE 100  // Maximum ADC reading to indicate continuity (pulled down by throttle)

#define BUZZER_TIME 1500      // Time in milliseconds for buzzer
#define MAX_THROTTLE_RTD 750  // Maximum throttle for entering ready to drive (5%)

#define MAX_TORQUE (int)(1210 * 1.00)  // Maximum torque for motor
#define TORQUE_VECTOR_GAIN 1.0         // Gain for torque vectoring
#define STEERING_LEFT_VOLTAGE 0.71      // Voltage for at left-most steering position
#define STEERING_RIGHT_VOLTAGE 4.27     // Voltage for at right-most steering position

#define ERROR_ALERT_INTERVAL 100  // Time in milliseconds for error alert

#define MIN_PRECHARGE_VOLTAGE 162.0  // Minimum voltage for AIR2 closing (2.7V per cell * 60 cells)
#define MIN_PRECHARGE_PERCENT 0.90   // Threshold for bus voltage to be considered precharged relative to batter voltage (90% of battery voltage)

#define MAX_SAG_PERCENT 0.60  // Threshold for bus voltage sag before forcing not ready to drive

#define VOLTAGE_DROP_TIMEOUT 500 // Time before voltage is considered invalid before leaving RTD

#define PRECHARGE_TIME 5000  // Time in milliseconds for precharge to take place
#define PRECHARGE_HOLD_TIME 500  // Minimum time in milliseconds for the contactors to be closed before the vehicle can be discharged

#define ACCEL_ROLLING_AVERAGE_SAMPLES 3  // Number of samples to average for acceleration
#define TRACTION_CONTROL_GAIN 1.5        // Gain for traction control
#define MOTOR_ACCEL_MAX 359.435652       // Acceleration at which wheel breaks traction (RPM/s)

typedef enum {
    INITIAL,
    VOLTAGE_CHECK,
    WAIT_FOR_PRECHARGE,
    PRECHARGE_STAGE1,
    PRECHARGE_STAGE2,
    PRECHARGE_STAGE3,
    NOT_READY_TO_DRIVE,
    BUZZER,
    READY_TO_DRIVE,
} CVC_state_t;

extern volatile CVC_state_t CVC_State;

/**
 * @brief Initializes the control module and state machine.
 * @param None
 * @retval None
 */
void Control_Init(void);

/**
 * @brief Updates the state machine based on the current state and inputs.
 * @param None
 * @retval None
 */
void Control_StateMachine(void);

/**
 * @brief Function for running control functions.
 * @param None
 * @retval None
 */
void Control_ControlTask(void);

/**
 * @brief Updates the state machine based on the current state and inputs.
 * @param None
 * @retval None
 */
void Control_StateMachine(void);

/**
 * @brief Blinks brake light and sounds buzzer if there is an error.
 * @param None
 * @retval None
 */
void Control_ErrorAlertTask(void);

/**
 * @brief Handles mixing of throttle inputs and error checking.
 * This function requires that both throttle inputs agree on the throttle position.
 * This function also checks for whether the throttle is in a valid range.
 * @param None
 * @retval None
 */
void Control_Throttle(void);

/**
 * @brief Calculates motor torque based on throttle input.
 * @param None
 * @retval None
 */
void Control_CalculateTorque(void);

/**
 * @brief Sends calculated torque to inverters.
 * @param None
 * @retval None
 */
void Control_TorqueCommand(void);
#endif /* INC_CVC_CONTROL_H */