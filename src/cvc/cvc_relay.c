/*
 * cvc_relay.c
 *
 *  Created on: Apr 7, 2024
 *      Author: Andrei Gerashchenko
 */

#include <FreeRTOS.h>
#include <cvc_relay.h>
#include <main.h>
#include <stdbool.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_spi.h>

#include <cvc_data.h>

extern SPI_HandleTypeDef hspi1;
SemaphoreHandle_t CVC_RelayMutex;
CVC_relay_state_t CVC_RelayState = {0};
bool CVC_RelayEnabled = false;

void Relay_Configure() {
    CVC_RelayMutex = xSemaphoreCreateMutex();
}

void Relay_Enable() {
    HAL_GPIO_WritePin(OUT_nDIS_GPIO_Port, OUT_nDIS_Pin, GPIO_PIN_SET);
}

void Relay_Disable() {
    HAL_GPIO_WritePin(OUT_nDIS_GPIO_Port, OUT_nDIS_Pin, GPIO_PIN_RESET);
}

void Relay_Set(CVC_relay_id_t relay_id, uint8_t state) {
    if (xSemaphoreTake(CVC_RelayMutex, portMAX_DELAY) == pdTRUE) {
        switch (relay_id) {
            case AIR2:
                CVC_RelayState.AIR2 = state;
                break;
            case BatteryFans:
                CVC_RelayState.BatteryFans = state;
                break;
            case Pumps:
                CVC_RelayState.Pumps = state;
                break;
            case Fans:
                CVC_RelayState.Fans = state;
                break;
            case BrakeLight:
                CVC_RelayState.BrakeLight = state;
                break;
            case ChargeEnable:
                CVC_RelayState.ChargeEnable = state;
                break;
            case Buzzer:
                CVC_RelayState.Buzzer = state;
                break;
            case LVChargeControl:
                CVC_RelayState.LVChargeControl = state;
                break;
            default:
                break;
        }
        xSemaphoreGive(CVC_RelayMutex);
    }
}

void Relay_Send() {
    if (xSemaphoreTake(CVC_RelayMutex, portMAX_DELAY) == pdTRUE) {
        uint8_t state = 0;
        state |= CVC_RelayState.AIR2 << 0;
        state |= CVC_RelayState.BatteryFans << 1;
        state |= CVC_RelayState.Pumps << 2;
        state |= CVC_RelayState.Fans << 3;
        state |= CVC_RelayState.BrakeLight << 4;
        state |= CVC_RelayState.ChargeEnable << 5;
        state |= CVC_RelayState.Buzzer << 6;
        state |= CVC_RelayState.LVChargeControl << 7;
        xSemaphoreGive(CVC_RelayMutex);

        taskENTER_CRITICAL();
        HAL_GPIO_WritePin(OUT_nCS_GPIO_Port, OUT_nCS_Pin, GPIO_PIN_RESET);

        // Ensure 5 nanosecond delay between CS and SCLK falling edge
        for (volatile uint8_t i = 0; i < 100; i++) {
            __asm__("NOP");
        }

        HAL_SPI_Transmit(&hspi1, &state, sizeof(state), HAL_MAX_DELAY);

        // Ensure 10 nanosecond delay between SCLK rising edge and CS
        for (volatile uint8_t i = 0; i < 200; i++) {
            __asm__("NOP");
        }

        HAL_GPIO_WritePin(OUT_nCS_GPIO_Port, OUT_nCS_Pin, GPIO_PIN_SET);
        taskEXIT_CRITICAL();
    }
}

void Relay_SendTask() {
    const TickType_t interval = RELAY_SEND_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        Relay_Send();
        last = xTaskGetTickCount();
    }
}

uint8_t Relay_IMDState() {
    return HAL_GPIO_ReadPin(IMD_STATE_GPIO_Port, IMD_STATE_Pin);
}

uint8_t Relay_AMSState() {
    return HAL_GPIO_ReadPin(AMS_STATE_GPIO_Port, AMS_STATE_Pin);
}