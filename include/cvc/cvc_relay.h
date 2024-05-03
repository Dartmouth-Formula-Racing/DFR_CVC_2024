/*
 * cvc_relay.h
 *
 *  Created on: Apr 7, 2024
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_RELAY_H
#define INC_CVC_RELAY_H

#include <FreeRTOS.h>
#include <semphr.h>
#include <stm32f7xx_hal_can.h>

#define RELAY_SEND_MS 50
#define RELAY_TEST_TASK_MS 50

// Enum for identifying relay outputs
typedef enum {
    AIR2 = 0,
    BatteryFans,
    Pumps,
    Fans,
    BrakeLight,
    ChargeEnable,
    Buzzer,
    LVChargeControl,
} CVC_relay_id_t;

// 8-bit struct for storing relay states
typedef struct {
    uint8_t AIR2 : 1;
    uint8_t BatteryFans : 1;
    uint8_t Pumps : 1;
    uint8_t Fans : 1;
    uint8_t BrakeLight : 1;
    uint8_t ChargeEnable : 1;
    uint8_t Buzzer : 1;
    uint8_t LVChargeControl : 1;
} CVC_relay_state_t;

extern SemaphoreHandle_t CVC_RelayMutex;
extern CVC_relay_state_t CVC_RelayState;

/**
 * @brief Configures relay driver.
 * @param None
 * @retval None
 */
void Relay_Configure(void);

/**
 * @brief Enables relay driver output.
 * @param None
 * @retval None 
 */
void Relay_Enable(void);

/**
 * @brief Disables relay driver output.
 * @param None
 * @retval None
 */
void Relay_Disable(void);

/**
 * @brief Sets relay state.
 * @param relay_id: Relay to set
 * @param state: State to set relay to
 * @retval None
 */
void Relay_Set(CVC_relay_id_t relay_id, uint8_t state);

/**
 * @brief Sends current relay state to relay driver.
 * @param None
 * @retval None
 */
void Relay_Send(void);

/**
 * @brief Task for sending current relay state to relay driver.
 * @param None
 * @retval None
 */
void Relay_SendTask(void);

/**
 * @brief Function for reading IMD latching relay state
 * @param None
 * @retval uint8_t IMD relay state 
 */
uint8_t Relay_IMDState(void);

/**
 * @brief Function for reading AMS latching relay state
 * @param None
 * @retval uint8_t AMS relay state 
 */
uint8_t Relay_AMSState(void);

#endif /* INC_CVC_RELAY_H */