/*
 * cvc_statemachine.h
 *
 *  Created on: Oct 25, 2023
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_STATEMACHINE_H_
#define INC_CVC_STATEMACHINE_H_

typedef enum {
    VOLTAGE_CHECK,
    WAIT_FOR_PRECHARGE,
    PRECHARGE,
    READY_TO_DRIVE,
    DRIVE,
    BUZZER,
    CVC_WARN,
    CVC_ERROR,
    CHARGING,
    CHARGE_DONE,
    CHARGE_ERROR,
} CVC_STATE_t;

extern volatile CVC_STATE_t CVC_STATE;

/**
 * @brief Initializes the CVC state machine
 * @param None
 * @retval None
 */
void CVC_StateMachineInit(void);

/**
 * @brief Runs the CVC state machine, should be called in Control task repeatedly
 * @param None
 * @retval None
 */
void CVC_StateMachine(void);

#endif /* INC_CVC_STATEMACHINE_H_ */