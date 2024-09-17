#include <cvc_12v_sense.h>
#include <cvc_can.h>
#include <cvc_control.h>
#include <cvc_data.h>
#include <cvc_relay.h>
#include <main.h>
#include <stdbool.h>
#include <stm32f7xx_hal_gpio.h>

#define REDUNDANT_THROTTLE 0        // 0 until front sensor board is installed
#define THROTTLE_POT2_CONTINUITY 1  // Use other potentiometer as resistor to check continuity

volatile CVC_state_t CVC_State = INITIAL;
volatile bool CVC_Error = false;  // Error signal, blinks brake light and beeps buzzer if true

uint8_t Control_RequestedState_Temp() {
    return HAL_GPIO_ReadPin(LV_CHARGE_STATE_GPIO_Port, LV_CHARGE_STATE_Pin);
}

void Control_Init() {
    CVC_State = INITIAL;
    Relay_Set(BrakeLight, 1);
}

void Control_StateMachine() {
    static TickType_t buzzerStartTime = 0;
    static TickType_t prechargeStartTime = 0;
    static TickType_t voltageDropStartTime = 0;
    static bool voltageDropTimer = false;
    static bool air2State = false;
    static bool neutralRequested = false;
    volatile CVC_data_t throttle;
    volatile CVC_data_t throttle_valid;
    volatile float HV_voltage = 0.0;
    volatile float Inv1_voltage = 0.0;
    volatile float Inv2_voltage = 0.0;
    volatile bool shutdownCircuit = false;
    uint8_t drive_mode = NEUTRAL;
    volatile uint8_t requested_drive_mode = NEUTRAL;

    // Get car data
    drive_mode = CVC_GetData(CVC_DRIVE_MODE).data;  // 0 = neutral, 1 = drive, 2 = reverse
    // requested_drive_mode = CVC_GetData(DASH_REQUESTED_STATE).data;  // 0 = neutral, 1 = drive, 2 = reverse
    requested_drive_mode = Control_RequestedState_Temp();  // 0 = neutral, 1 = drive, no reverse
    shutdownCircuit = CVC_12V_Read(AIR_Power);
    HV_voltage = (float)(CVC_GetData(BMS_TOTAL_VOLTAGE).data) * (0.01);
    Inv1_voltage = (float)((int16_t)CVC_GetData(INVERTER1_DC_BUS_VOLTAGE).data) * (0.1);
    Inv2_voltage = (float)((int16_t)CVC_GetData(INVERTER2_DC_BUS_VOLTAGE).data) * (0.1);

    if (requested_drive_mode == NEUTRAL) {
        neutralRequested = true;
    }

    switch (CVC_State) {
        case INITIAL:
            drive_mode = NEUTRAL;
            air2State = false;          // Open AIR2
            CVC_State = VOLTAGE_CHECK;  // Directly transition to voltage check
            break;
        case VOLTAGE_CHECK:
            drive_mode = NEUTRAL;
            air2State = false;  // Open AIR2
            // TODO: Implement LV voltage checks

            // Check if HV voltage is above MIN_PRECHARGE_VOLTAGE or if AIR power is on
            if (HV_voltage >= MIN_PRECHARGE_VOLTAGE) {
                CVC_State = WAIT_FOR_PRECHARGE;
            } else {
                CVC_State = VOLTAGE_CHECK;
            }
            break;
        case WAIT_FOR_PRECHARGE:
            drive_mode = NEUTRAL;
            air2State = false;  // Open AIR2
            // Wait for precharge button to be pressed and shutdown circuit to be closed
            if (CVC_12V_Read(PrechargeBtn) && shutdownCircuit) {
                CVC_State = PRECHARGE_START;
                prechargeStartTime = xTaskGetTickCount();
            }
            break;
        case PRECHARGE_START:
            drive_mode = NEUTRAL;
            air2State = false;  // Open AIR2
            // Wait for precharge time to elapse
            if (xTaskGetTickCount() - prechargeStartTime >= PRECHARGE_TIME / portTICK_PERIOD_MS) {
                CVC_State = PRECHARGE_END;
            } else {
                CVC_State = PRECHARGE_START;
            }
            break;
        case PRECHARGE_END:
            drive_mode = NEUTRAL;
            // Wait for HV bus voltage to reach 90% of HV battery voltage
            if (Inv1_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT && Inv2_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT) {
                CVC_State = NOT_READY_TO_DRIVE;
                air2State = true;  // Close AIR2
            } else {
                CVC_State = WAIT_FOR_PRECHARGE;
                air2State = false;  // Open AIR2
            }
            break;
        case NOT_READY_TO_DRIVE:
            // Enter buzzer state if drive or reverse is requested and throttle is valid and throttle is less than 5%
            throttle = CVC_GetData(CVC_THROTTLE);
            throttle_valid = CVC_GetData(CVC_THROTTLE_VALID);
            drive_mode = NEUTRAL;
            // Return to wait for precharge state if bus voltage drops below HV_voltage * MIN_PRECHARGE_PERCENT
            if (Inv1_voltage < HV_voltage * MIN_PRECHARGE_PERCENT || Inv2_voltage < HV_voltage * MIN_PRECHARGE_PERCENT) {
                CVC_State = WAIT_FOR_PRECHARGE;
                air2State = false;  // Open AIR2
                break;
            }

            // if ((requested_drive_mode == DRIVE || requested_drive_mode == REVERSE) && neutralRequested) {
            //     if (throttle_valid.data == 1 && throttle.data < MAX_THROTTLE_RTD) {
            //         neutralRequested = 0;  // clear requested neutral, require neutral again before next drive
            //         CVC_State = BUZZER;
            //         buzzerStartTime = xTaskGetTickCount();
            //         Relay_Set(Buzzer, 1);  // Turn on buzzer
            //     }
            // }

            // RTD button bypass, remove when dashboard is fixed
            if ((throttle_valid.data == 1) & (throttle.data < MAX_THROTTLE_RTD)) {
                CVC_State = READY_TO_DRIVE;
                drive_mode = DRIVE;
            }

            break;
        case BUZZER:
            neutralRequested = 0;  // clear requested neutral, require neutral again before next drive
            drive_mode = NEUTRAL;
            // Return to wait for precharge state if bus voltage drops below HV_voltage * MIN_PRECHARGE_PERCENT
            if (Inv1_voltage < HV_voltage * MIN_PRECHARGE_PERCENT && Inv2_voltage < HV_voltage * MIN_PRECHARGE_PERCENT) {
                CVC_State = WAIT_FOR_PRECHARGE;
                air2State = false;     // Open AIR2
                Relay_Set(Buzzer, 0);  // Turn off buzzer
                break;
            }
            // Enter ready to drive state once buzzer time has elapsed
            if (xTaskGetTickCount() - buzzerStartTime >= BUZZER_TIME / portTICK_PERIOD_MS) {
                CVC_State = READY_TO_DRIVE;
                Relay_Set(Buzzer, 0);  // Turn off buzzer
            }
            break;
        case READY_TO_DRIVE:
            neutralRequested = 0;  // clear requested neutral, require neutral again before next drive
            // Return to wait for precharge state if bus voltage drops below HV_voltage * MIN_PRECHARGE_PERCENT
            if (Inv1_voltage < HV_voltage * MAX_SAG_PERCENT && Inv2_voltage < HV_voltage * MAX_SAG_PERCENT) {
                if (!voltageDropTimer) {
                    voltageDropStartTime = xTaskGetTickCount();
                    voltageDropTimer = true;
                }
                if (voltageDropTimer) {
                    if (xTaskGetTickCount() - voltageDropStartTime >= (VOLTAGE_DROP_TIMEOUT / portTICK_PERIOD_MS)) {
                        CVC_State = WAIT_FOR_PRECHARGE;
                        drive_mode = NEUTRAL;
                        air2State = false;  // Open AIR2
                    }
                }
                break;
            } else {
                voltageDropTimer = false;
            }

            throttle_valid = CVC_GetData(CVC_THROTTLE_VALID);
            if (throttle_valid.data == 0) {
                CVC_State = WAIT_FOR_PRECHARGE;
                drive_mode = NEUTRAL;
                air2State = false;  // Open AIR2
                break;
            }

            // if (requested_drive_mode == NEUTRAL) {
            //     CVC_State = NOT_READY_TO_DRIVE;
            //     drive_mode = NEUTRAL;
            // } else if (requested_drive_mode == DRIVE && drive_mode == NEUTRAL) {
            //     drive_mode = DRIVE;
            // } else if (requested_drive_mode == REVERSE && drive_mode == NEUTRAL) {
            //     drive_mode = REVERSE;
            // }
            break;
    }
    // Update drive mode
    CVC_SetData(CVC_DRIVE_MODE, &drive_mode, sizeof(drive_mode));
    // Update AIR2 state
    Relay_Set(AIR2, air2State);
    Relay_Set(Pumps, CVC_State == READY_TO_DRIVE);
    // Relay_Set(BatteryFans, CVC_State == READY_TO_DRIVE);
}

void Control_ErrorAlertTask() {
    const TickType_t interval = ERROR_ALERT_INTERVAL / portTICK_PERIOD_MS;
    static TickType_t last = 0;
    static bool errorBlinkState = false;
    if (CVC_Error) {
        if ((xTaskGetTickCount() - last) >= interval) {
            last = xTaskGetTickCount();
            errorBlinkState = !errorBlinkState;
            Relay_Set(BrakeLight, errorBlinkState);
            Relay_Set(Buzzer, errorBlinkState);
        }
    }
}

void Control_ControlTask() {
    const TickType_t interval = CONTROL_TASK_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        last = xTaskGetTickCount();
        Control_Throttle();
        Control_StateMachine();
        Control_CalculateTorque();
    }
}

void Control_Throttle() {
    bool throttle_valid = true;
    CVC_data_t throttle1data = CVC_GetData(CVC_THROTTLE_ADC);
    uint16_t throttle1 = (uint16_t)throttle1data.data;
    static TickType_t throttle_invalid_timer = 0;
    static bool throttle_invalid_counting = false;
#if THROTTLE_POT2_CONTINUITY
    CVC_data_t throttle2data = CVC_GetData(INVERTER2_ANALOG_INPUT_1);
    uint16_t throttle2 = (uint16_t)throttle2data.data;
#endif
#if REDUNDANT_THROTTLE
    CVC_data_t throttle2data = CVC_GetData(CVC_THROTTLE_2);
    uint16_t throttle2 = (uint16_t)throttle2data.data;
#endif

// Check if throttle1 and throttle2 are within MAX_THROTTLE_DIFFERENCE
#if REDUNDANT_THROTTLE
    if (throttle1 > throttle2) {
        if (throttle1 - throttle2 > MAX_THROTTLE_DIFFERENCE) {
            throttle_valid = false;
        }
    } else {
        if (throttle2 - throttle1 > MAX_THROTTLE_DIFFERENCE) {
            throttle_valid = false;
        }
    }
#endif
    // Check if throttle1 is within MIN_THROTTLE_RANGE and MAX_THROTTLE_RANGE
    if (throttle1 < MIN_THROTTLE_RANGE || throttle1 > MAX_THROTTLE_RANGE) {
        if (!throttle_invalid_counting) {
            throttle_invalid_counting = true;
            throttle_invalid_timer = xTaskGetTickCount();
        }
        if (throttle_invalid_counting) {
            if (xTaskGetTickCount() - throttle_invalid_timer >= (THROTTLE_INVALID_TIMER / portTICK_PERIOD_MS)) {
                throttle_valid = false;
            }
        }
    } else {
        throttle_invalid_counting = false;
    }
    // Check if throttle2 is pulling down ADC1 on inverter
    if (throttle2 > MAX_THROTTLE_CONT_VALUE) {
        if (!throttle_invalid_counting) {
            throttle_invalid_counting = true;
            throttle_invalid_timer = xTaskGetTickCount();
        }
        if (throttle_invalid_counting) {
            if (xTaskGetTickCount() - throttle_invalid_timer >= (THROTTLE_INVALID_TIMER / portTICK_PERIOD_MS)) {
                throttle_valid = false;
            }
        }
    } else {
        throttle_invalid_counting = false;
    }
// Check if throttle2 is within MIN_THROTTLE_RANGE and MAX_THROTTLE_RANGE
#if REDUNDANT_THROTTLE
    if (throttle2 < MIN_THROTTLE_RANGE || throttle2 > MAX_THROTTLE_RANGE) {
        throttle_valid = false;
    }
#endif
    // Throttle is valid, calculate throttle percentage
    if (throttle_valid) {
        // Clamp throttle values to MIN_THROTTLE and MAX_THROTTLE
        if (throttle1 < MIN_THROTTLE) {
            throttle1 = MIN_THROTTLE;
        }
        if (throttle1 > MAX_THROTTLE) {
            throttle1 = MAX_THROTTLE;
        }
    }
    // Update throttle valid data
    CVC_SetData(CVC_THROTTLE_VALID, &throttle_valid, sizeof(throttle_valid));
    // Update throttle percentage data
    CVC_SetData(CVC_THROTTLE, &throttle1, 2);
}

void Control_CalculateTorque() {
    volatile float throttle = 0.0;
    volatile CVC_data_t throttle_data = CVC_GetData(CVC_THROTTLE);
    volatile CVC_data_t throttle_valid = CVC_GetData(CVC_THROTTLE_VALID);
    volatile int16_t throttleDifference = (uint16_t)throttle_data.data - MIN_THROTTLE;

    volatile int16_t torque = 0;
    volatile int16_t torque_left = 0;
    volatile int16_t torque_right = 0;
    uint8_t command_left[8] = {0};
    uint8_t command_right[8] = {0};

    throttle = (float)throttleDifference / ((float)MAX_THROTTLE - (float)MIN_THROTTLE);

    // volatile int16_t steering_data = CVC_GetData(SENSOR_STEERING_ANGLE).data;  // 0 to 4095, center is 2048
    volatile uint16_t steering_data = CVC_GetData(INVERTER1_ANALOG_INPUT_1).data;
    volatile uint8_t drive_mode = CVC_GetData(CVC_DRIVE_MODE).data;  // 0 = neutral, 1 = drive, 2 = reverse
    // volatile float steering = (float)(steering_data - 2048) / 2048;                  // -1 to 1
    volatile float steering = (float)(steering_data - ADC_WHEEL_RIGHT) / (float)(ADC_WHEEL_LEFT - ADC_WHEEL_RIGHT);

    // Bytes 0-1: Torque command (-32768 to 32767) Nm/10 [0, 1]
    // Bytes 2-3: Speed command (-32768 to 32767) RPM [2, 3]
    // Byte 4: Direction (0 = reverse, 1 = forward) [4]
    // Byte 5 bit 0: Inverter enable (0 = disable, 1 = enable) [5]
    // Byte 5 bit 1: Inverter discharge (0 = disable, 1 = enable) [5]
    // Byte 5 bit 2: Speed mode enable (0 = disable, 1 = enable) [5]
    // Bytes 6-7: Torque limit (-32768 to 32767) Nm/10 [6, 7]

    // Default command values are everything 0 or disabled
    command_left[0] = 0;
    command_left[1] = 0;
    command_left[2] = 0;
    command_left[3] = 0;
    command_left[4] = 0;
    command_left[5] = 0;
    command_left[6] = 0;
    command_left[7] = 0;

    if (CVC_State == READY_TO_DRIVE && throttle_valid.data == 1 && throttle > 0) {
        if (drive_mode == DRIVE) {  // drive
            // Enable inverters
            command_left[5] |= (1 << 0);   // Enable inverter
            command_right[5] |= (1 << 0);  // Enable inverter
            if (throttle > 0) {
                command_left[4] = 1;  // Forward direction
                // command_right[4] = 0;  // Reverse direction -- Motor is flipped
                command_right[4] = 1;  // Forward direction -- Somehow the right motor is spinning backwards
                // Calculate torque for pressed pedal
                torque = (int16_t)(throttle * MAX_TORQUE);
                // Calculate torque vectoring based on steering angle
                // torque_left = torque + (int16_t)(steering * TORQUE_VECTOR_GAIN * torque);  // Calculate torque vectoring based on steering angle
                // torque_right = torque - (int16_t)(steering * TORQUE_VECTOR_GAIN * torque);

                // Don't have steering angle yet
                torque_left = torque;
                torque_right = torque;

                // Clamp torque values to 0-MAX_TORQUE
                if (torque_left < 0) {
                    torque_left = 0;
                }
                if (torque_left > MAX_TORQUE) {
                    torque_left = MAX_TORQUE;
                }
                if (torque_right < 0) {
                    torque_right = 0;
                }
                if (torque_right > MAX_TORQUE) {
                    torque_right = MAX_TORQUE;
                }
                // Set torque command
                command_left[0] = (uint8_t)(torque_left & 0xFF);         // Select lower byte of torque_left
                command_left[1] = (uint8_t)(torque_left >> 8 & 0xFFF);   // Select upper byte of torque_left
                command_right[0] = (uint8_t)(torque_right & 0xFF);       // Select lower byte of torque_right
                command_right[1] = (uint8_t)(torque_right >> 8 & 0xFF);  // Select upper byte of torque_right
            }
            // } else if (drive_mode == REVERSE) {  // reverse

        } else {  // neutral
            // Disable inverters
            command_left[5] &= ~(1 << 0);   // Disable inverter
            command_right[5] &= ~(1 << 0);  // Disable inverter
            // Set torque command to 0
            command_left[0] = 0;
            command_left[1] = 0;
            command_right[0] = 0;
            command_right[1] = 0;
            // Set speed command to 0
            command_left[2] = 0;
            command_left[3] = 0;
            command_right[2] = 0;
            command_right[3] = 0;
        }
    } else {                            // Disable inverters
        command_left[5] &= ~(1 << 0);   // Disable inverter
        command_right[5] &= ~(1 << 0);  // Disable inverter
        // Set torque command to 0
        command_left[0] = 0;
        command_left[1] = 0;
        command_right[0] = 0;
        command_right[1] = 0;
        // Set speed command to 0
        command_left[2] = 0;
        command_left[3] = 0;
        command_right[2] = 0;
        command_right[3] = 0;
    }

    uint64_t command_left_int = 0;
    uint64_t command_right_int = 0;
    for (int i = 0; i < 8; i++) {
        command_left_int |= (uint64_t)command_left[i] << (8 * i);
        command_right_int |= (uint64_t)command_right[i] << (8 * i);
    }
    CVC_SetData(CVC_LEFT_TORQUE, &command_left_int, sizeof(command_left_int));
    CVC_SetData(CVC_RIGHT_TORQUE, &command_right_int, sizeof(command_right_int));
}

void Control_TorqueCommand() {
    const TickType_t interval = CONTROL_TORQUECOMMAND_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) < interval) {
        return;
    }
    last = xTaskGetTickCount();

    uint64_t command_left_int = 0;
    uint64_t command_right_int = 0;
    uint8_t command_left[8] = {0};
    uint8_t command_right[8] = {0};
    command_left_int = CVC_GetData(CVC_LEFT_TORQUE).data;
    command_right_int = CVC_GetData(CVC_RIGHT_TORQUE).data;
    for (int i = 0; i < 8; i++) {
        command_left[i] = (uint8_t)(command_left_int >> (8 * i) & 0xFF);
        command_right[i] = (uint8_t)(command_right_int >> (8 * i) & 0xFF);
    }

    CAN_Transmit(CAN_INVERTER_BASE_ID1 + 32, command_left, sizeof(command_left) / sizeof(command_left[0]), CAN_INVERTER_USE_EXT);
    CAN_Transmit(CAN_INVERTER_BASE_ID2 + 32, command_right, sizeof(command_right) / sizeof(command_right[0]), CAN_INVERTER_USE_EXT);
}