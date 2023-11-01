#include "cvc_statemachine.h"
#include "cvc_constants.h"
#include "stdbool.h"

volatile CVC_STATE_t CVC_STATE = VOLTAGE_CHECK;

/**
 * @brief Checks if GLVS and accumulator voltages are within acceptable range
 * 
 * @return true 
 * @return false 
 */
bool VoltageCheck();

void CVC_StateMachineInit() {
    CVC_STATE = VOLTAGE_CHECK;
}

// VOLTAGE_CHECK,
// WAIT_FOR_PRECHARGE,
// PRECHARGE,
// READY_TO_DRIVE,
// DRIVE,
// BUZZER,
// CVC_WARN,
// CHARGING,
// CHARGE_DONE,
// CHARGE_ERROR,

void CVC_StateMachine() {
    switch (CVC_STATE) {
        case VOLTAGE_CHECK:
            if (VoltageCheck()) {
                CVC_STATE = WAIT_FOR_PRECHARGE;
            } else {
                CVC_STATE = CVC_ERROR;
            }
            break;
        case WAIT_FOR_PRECHARGE:
            break;
        case PRECHARGE:
            break;
        case READY_TO_DRIVE:
            break;
        case DRIVE:
            break;
        case BUZZER:
            break;
        case CVC_WARN:
            break;
        case CVC_ERROR:
            break;
        case CHARGING:
            break;
        case CHARGE_DONE:
            break;
        case CHARGE_ERROR:
            break;
    }
}

/**
 * @brief Checks if GLVS and accumulator voltages are within acceptable range
 * 
 * @return true 
 * @return false 
 */
bool VoltageCheck() {
    // TODO: Actually implement GLVS_VOLTAGE and ACC_VOLTAGE
    #define GLVS_VOLTAGE 12
    #define ACC_VOLTAGE 230
    if (GLVS_VOLTAGE < GLVS_MIN_VOLTAGE || GLVS_VOLTAGE > GLVS_MAX_VOLTAGE) {
        return false;
    }
    if (ACC_VOLTAGE < ACCUMULATOR_MIN_VOLTAGE || ACC_VOLTAGE > ACCUMULATOR_MAX_VOLTAGE) {
        return false;
    }
    
    return true;
}