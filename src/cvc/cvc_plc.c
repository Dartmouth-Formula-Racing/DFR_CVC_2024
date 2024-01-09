/*
 * cvc_plc.c
 *
 *  Created on: Sep 27, 2023
 *      Author: Andrei Gerashchenko
 */

#include <clt01_38s.h>
#include <cmsis_os.h>
#include <cvc_data.h>
#include <cvc_plc.h>
#include <vni8200xp.h>
#include <x_nucleo_plc01a1.h>

// State of PLC input and output pins
PLC_INPUT_t PLC_Inputs = (PLC_INPUT_t){0};
PLC_OUTPUT_t PLC_Outputs = (PLC_OUTPUT_t){0};

void PLC_BlinkTask() {
    static uint8_t output = 0;

    const TickType_t interval = 500 / portTICK_PERIOD_MS;
    static TickType_t last = 0;
    // Invert state every 500 ms
    if ((xTaskGetTickCount() - last) >= interval) {
        // Set all to 0
        PLC_Outputs = (PLC_OUTPUT_t){0};
        // Turn on current output
        switch (output) {
            case 0:
                PLC_Outputs.OUT1 = 1;
                break;
            case 1:
                PLC_Outputs.OUT2 = 1;
                break;
            case 2:
                PLC_Outputs.OUT3 = 1;
                break;
            case 3:
                PLC_Outputs.OUT4 = 1;
                break;
            case 4:
                PLC_Outputs.OUT5 = 1;
                break;
            case 5:
                PLC_Outputs.OUT6 = 1;
                break;
            case 6:
                PLC_Outputs.OUT7 = 1;
                break;
            case 7:
                PLC_Outputs.OUT8 = 1;
                break;
            default:
                PLC_Outputs = (PLC_OUTPUT_t){0};
                break;
        }
        output = (output == 7) ? 0 : (output + 1);
        last = xTaskGetTickCount();
    }
}

// Convert output struct to uint8_t
uint8_t PLC_Encode(PLC_OUTPUT_t output) {
    return (output.OUT8 << 7) | (output.OUT7 << 6) | (output.OUT6 << 5) | (output.OUT5 << 4) | (output.OUT4 << 3) | (output.OUT3 << 2) | (output.OUT2 << 1) | output.OUT1;
}

// Convert input uint8_t to struct
void PLC_Decode(uint8_t value) {
    PLC_Inputs.IN8 = (value >> 7) & 0x01;
    PLC_Inputs.IN7 = (value >> 6) & 0x01;
    PLC_Inputs.IN6 = (value >> 5) & 0x01;
    PLC_Inputs.IN5 = (value >> 4) & 0x01;
    PLC_Inputs.IN4 = (value >> 3) & 0x01;
    PLC_Inputs.IN3 = (value >> 2) & 0x01;
    PLC_Inputs.IN2 = (value >> 1) & 0x01;
    PLC_Inputs.IN1 = value & 0x01;
}

void PLC_CommunicationTask() {
    // Interval for communicating with PLC peripheral
    const TickType_t interval = 40 / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        // Critical section - prevent context switch while transmitting
        taskENTER_CRITICAL();
        // BSP_CURRENT_LIMITER_Read() returns a pointer to a uint8_t[2], 2nd byte contains inputs
        uint8_t inputs = *(BSP_CURRENT_LIMITER_Read() + 1*sizeof(uint8_t));
        PLC_Decode(inputs);
        taskEXIT_CRITICAL();

        // Critical section - prevent context switch while transmitting
        taskENTER_CRITICAL();
        BSP_RELAY_EN_Out();
        uint8_t outputs = PLC_Encode(PLC_Outputs);
        BSP_RELAY_SetOutputs(&outputs);
        taskEXIT_CRITICAL();

        last = xTaskGetTickCount();
    }
}

void PLC_Configure() {
    // Initialize PLC outputs
    BSP_Relay_Init();
    // Initialize PLC inputs
    BSP_CurrentLimiter_Init();
    // Reset PLC outputs
    BSP_RELAY_Reset();
    HAL_Delay(100);
}