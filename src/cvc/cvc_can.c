/*
 * cvc_can.c
 *
 *  Created on: Sep 30, 2023
 *      Author: Andrei Gerashchenko
 */

#include <FreeRTOS.h>
#include <cvc_can.h>
#include <cvc_data.h>
#include <cvc_relay.h>
#include <main.h>
#include <queue.h>
#include <stdbool.h>
#include <stm32f7xx_hal_can.h>

extern CAN_HandleTypeDef hcan1;
QueueHandle_t CAN_TxQueue;
QueueHandle_t CAN_RxQueue;

void CAN_Configure() {
    CAN_TxQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_Queue_Frame_t));
    CAN_RxQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_Queue_Frame_t));
}

// ===================== CAN Communication Task Functions =====================

/**
 * @brief Internal function to receive CAN messages and place them in Rx queue.
 * @param None
 * @retval None
 */
void CANRxToQueue() {
    // Return early if no messages have been received
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        return;
    }

    CAN_Queue_Frame_t rx_frame;
    taskENTER_CRITICAL();
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_frame.Rx_header, rx_frame.data);
    taskEXIT_CRITICAL();
    // Put in Rx queue
    xQueueSendToBack(CAN_RxQueue, &rx_frame, 0);
}

/**
 * @brief Internal function to transmit CAN messages from Tx queue.
 * @param None
 * @retval None
 */
void CANTxFromQueue() {
    // Return early if there are no messages in Tx queue
    if (uxQueueMessagesWaiting(CAN_TxQueue) == 0) {
        return;
    }

    // Ensure that CAN peripheral is ready to transmit
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        return;
    }

    CAN_Queue_Frame_t tx_frame;
    uint32_t mailbox;

    // Get message from Tx queue
    xQueueReceive(CAN_TxQueue, &tx_frame, 0);
    taskENTER_CRITICAL();
    HAL_CAN_AddTxMessage(&hcan1, &tx_frame.Tx_header, tx_frame.data, &mailbox);
    taskEXIT_CRITICAL();
}

void CAN_CommunicationTask() {
    // Interval for CAN communication
    const TickType_t interval = CAN_PERIPHERAL_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        CANTxFromQueue();
        CANRxToQueue();
        last = xTaskGetTickCount();
    }
}

void CAN_BroadcastTask() {
    // Interval for broadcast
    const TickType_t interval = CAN_BROADCAST_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        CAN_Broadcast_Dashboard_Critical();
        last = xTaskGetTickCount();
    }
}

// ===================== CAN Processing Task Functions =====================

void CAN_Transmit(uint32_t id, uint8_t data[8], uint8_t len, bool isExt) {
    CAN_Queue_Frame_t tx_frame;
    tx_frame.Tx_header.StdId = id;
    tx_frame.Tx_header.ExtId = id;
    tx_frame.Tx_header.IDE = isExt ? CAN_ID_EXT : CAN_ID_STD;
    tx_frame.Tx_header.RTR = CAN_RTR_DATA;
    tx_frame.Tx_header.DLC = len;
    for (int i = 0; i < len; i++) {
        tx_frame.data[i] = data[i];
    }
    xQueueSendToBack(CAN_TxQueue, &tx_frame, 0);
}

/**
 * @brief Internal function to interpret CAN messages in Rx queue.
 * Parses received CAN messages and updates global variables.
 * @param None
 * @retval None
 */
void CANInterpretRx() {
    // Return early if there are no messages in Rx queue
    if (uxQueueMessagesWaiting(CAN_RxQueue) == 0) {
        return;
    }

    CAN_Queue_Frame_t rx_frame;

    // Get message from Rx queue
    xQueueReceive(CAN_RxQueue, &rx_frame, 0);

    if (rx_frame.Rx_header.StdId == CAN_DASHBOARD_BASE_11) {
        // Dashboard - Base Address
        CAN_Parse_Dashboard(rx_frame);
    }
    if (rx_frame.Rx_header.ExtId == CAN_DASHBOARD_BASE_11) {
        CAN_Parse_Dashboard(rx_frame);
    }

    // Select parsing function based on CAN ID
    // TODO: Find a cleaner way to do this
    if (rx_frame.Rx_header.IDE == CAN_ID_EXT) {  // 29-bit CAN messages
        // ===== EMUS BMS CAN IDs =====
        // Format: Byte 0: Base (MSB) | Byte 1: Base (LSB) | Byte 3 and 4: Message ID
        if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0000)) {
            // EMUS BMS Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x00
            CAN_Parse_EMUS_OverallParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0007)) {
            // EMUS BMS Diagnostic Codes - Byte 3 = 0x00 and Byte 4 = 0x07
            CAN_Parse_EMUS_DiagnosticCodes(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0001)) {
            // EMUS BMS Battery Voltage Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x01
            CAN_Parse_EMUS_BatteryVoltageOverallParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0002)) {
            // EMUS BMS Cell Module Temperature Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x02
            CAN_Parse_EMUS_CellModuleTemperatureOverallParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0008)) {
            // EMUS BMS Cell Temperature Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x08
            CAN_Parse_EMUS_CellTemperatureOverallParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0003)) {
            // EMUS BMS Cell Balancing Rate Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x03
            CAN_Parse_EMUS_CellBalancingRateOverallParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0500)) {
            // EMUS BMS State of Charge Parameters - Byte 3 = 0x05 and Byte 4 = 0x00
            CAN_Parse_EMUS_StateOfChargeParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0400)) {
            // EMUS BMS Configuration Parameters - Byte 3 = 0x04 and Byte 4 = 0x00
            CAN_Parse_EMUS_ConfigurationParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0401)) {
            // EMUS BMS Contactor Control - Byte 3 = 0x04 and Byte 4 = 0x01
            CAN_Parse_EMUS_ContactorControl(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0600)) {
            // EMUS BMS Energy Parameters - Byte 3 = 0x06 and Byte 4 = 0x00
            CAN_Parse_EMUS_EnergyParameters(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_EMUS_BASE_29 << 16) | 0x0405)) {
            // EMUS BMS Events - Byte 3 = 0x04 and Byte 4 = 0x05
            CAN_Parse_EMUS_Events(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_VDM_BASE_29 << 16) | 0X0000)) {
            // VDM GPS Latitude and Longitude - 0x0000A0000
            CAN_Parse_VDM_GPSLatitudeLongitude(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_VDM_BASE_29 << 16) | 0X0001)) {
            // VDM GPS Data - 0x0000A0001
            CAN_Parse_VDM_GPSData(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_VDM_BASE_29 << 16) | 0X0002)) {
            // VDM GPS Date and Time - 0x0000A0002
            CAN_Parse_VDM_GPSDateTime(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_VDM_BASE_29 << 16) | 0X0003)) {
            // VDM Acceleration Data - 0x0000A0003
            CAN_Parse_VDM_AccelerationData(rx_frame);
        } else if (rx_frame.Rx_header.ExtId == ((CAN_VDM_BASE_29 << 16) | 0X0004)) {
            // VDM Yaw Rate Data - 0x0000A0004
            CAN_Parse_VDM_YawRateData(rx_frame);
        }
    } else if (rx_frame.Rx_header.IDE == CAN_ID_STD) {  // 11-bit CAN messages
        // ===== Dashboard CAN IDs =====
        if (rx_frame.Rx_header.StdId == (CAN_DASHBOARD_BASE_11 + 0)) {
            // Dashboard - Base Address + 0
            CAN_Parse_Dashboard(rx_frame);
        }

        // ===== Sensor Board CAN IDs =====
        if (rx_frame.Rx_header.StdId == (CAN_SENSORBOARD_BASE_11 + 0)) {
            // Sensor board - Base Address + 0
            CAN_Parse_SensorBoard(rx_frame);
        }
        // ===== PM100DX Inverter1 CAN IDs =====
        if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 0)) {
            // Inverter Temperatures #1 - Base Address + 0
            CAN_Parse_Inverter_Temp1(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 1)) {
            // Inverter Temperatures #2 - Base Address + 1
            CAN_Parse_Inverter_Temp2(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 2)) {
            // Inverter Temperatures #3 and Torque Shudder - Base Address + 2
            CAN_Parse_Inverter_Temp3TorqueShudder(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 3)) {
            // Inverter Analog Input Status - Base Address + 3
            CAN_Parse_Inverter_AnalogInputStatus(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 4)) {
            // Inverter Digital Input Status - Base Address + 4
            CAN_Parse_Inverter_DigitalInputStatus(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 5)) {
            // Inverter Motor Position Parameters - Base Address + 5
            CAN_Parse_Inverter_MotorPositionParameters(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 6)) {
            // Inverter Current Parameters - Base Address + 6
            CAN_Parse_Inverter_CurrentParameters(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 7)) {
            // Inverter Voltage Parameters - Base Address + 7
            CAN_Parse_Inverter_VoltageParameters(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 8)) {
            // Inverter Flux Parameters - Base Address + 8
            CAN_Parse_Inverter_FluxParameters(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 9)) {
            // Inverter Internal Voltage Parameters - Base Address + 9
            CAN_Parse_Inverter_InternalVoltageParameters(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 10)) {
            // Inverter Internal State Parameters - Base Address + 10
            CAN_Parse_Inverter_InternalStateParameters(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 11)) {
            // Inverter Fault Codes - Base Address + 11
            CAN_Parse_Inverter_FaultCodes(rx_frame, 1);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID1 + 16)) {
            // Inverter High Speed Parameters - Base Address + 16
            CAN_Parse_Inverter_HighSpeedParameters(rx_frame, 1);
        }
        // ===== PM100DX Inverter2 CAN IDs =====
        if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 0)) {
            // Inverter Temperatures #1 - Base Address + 0
            CAN_Parse_Inverter_Temp1(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 1)) {
            // Inverter Temperatures #2 - Base Address + 1
            CAN_Parse_Inverter_Temp2(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 2)) {
            // Inverter Temperatures #3 and Torque Shudder - Base Address + 2
            CAN_Parse_Inverter_Temp3TorqueShudder(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 3)) {
            // Inverter Analog Input Status - Base Address + 3
            CAN_Parse_Inverter_AnalogInputStatus(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 4)) {
            // Inverter Digital Input Status - Base Address + 4
            CAN_Parse_Inverter_DigitalInputStatus(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 5)) {
            // Inverter Motor Position Parameters - Base Address + 5
            CAN_Parse_Inverter_MotorPositionParameters(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 6)) {
            // Inverter Current Parameters - Base Address + 6
            CAN_Parse_Inverter_CurrentParameters(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 7)) {
            // Inverter Voltage Parameters - Base Address + 7
            CAN_Parse_Inverter_VoltageParameters(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 8)) {
            // Inverter Flux Parameters - Base Address + 8
            CAN_Parse_Inverter_FluxParameters(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 9)) {
            // Inverter Internal Voltage Parameters - Base Address + 9
            CAN_Parse_Inverter_InternalVoltageParameters(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 10)) {
            // Inverter Internal State Parameters - Base Address + 10
            CAN_Parse_Inverter_InternalStateParameters(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 11)) {
            // Inverter Fault Codes - Base Address + 11
            CAN_Parse_Inverter_FaultCodes(rx_frame, 0);
        } else if (rx_frame.Rx_header.StdId == (CAN_INVERTER_BASE_ID2 + 16)) {
            // Inverter High Speed Parameters - Base Address + 16
            CAN_Parse_Inverter_HighSpeedParameters(rx_frame, 0);
        }
    }
}

/**
 * @brief Internal function to interpret CAN messages in Rx queue.
 * Parses received CAN messages and updates global variables.
 * @param None
 * @retval None
 */
void CAN_InterpretTask() {
    // Interval for CAN interpretation
    const TickType_t interval = CAN_PROCESSING_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        CANInterpretRx();
        last = xTaskGetTickCount();
    }
}

// ========================= CAN Parsing Functions =========================

// ========== Dashboard Parsing Functions ==========
/**
 * @brief Parses Dashboard 11-bit CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_Dashboard(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    CVC_data[DASH_REQUESTED_STATE].data = frame.data[0];
    xSemaphoreGive(CVC_DataMutex);
}

// TODO: Implement 11-bit CAN message parsing functions
// Only 29-bit CAN message parsing functions are currently implemented
// ========== EMUS BMS Parsing Functions ==========

/**
 * @brief Parses EMUS BMS 29-bit Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_OverallParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Input signals
    // Bit 0: Ignition key
    // Bit 1: Charger mains
    // Bit 2: Fast charge
    // Bit 3: Leakage sensor
    // Bits 4-7: Don't care
    CVC_data[BMS_IGNITION].data = (frame.data[0] >> 0) & 0x01;
    CVC_data[BMS_CHARGER_MAINS].data = (frame.data[0] >> 1) & 0x01;
    CVC_data[BMS_FAST_CHARGE].data = (frame.data[0] >> 2) & 0x01;
    CVC_data[BMS_LEAKAGE_DETECTED].data = (frame.data[0] >> 3) & 0x01;
    // Byte 1: Output signals
    // Bit 0: Charger enable
    // Bit 1: Heater enable
    // Bit 2: Battery contactor
    // Bit 3: Battery fan
    // Bit 4: Power reduction
    // Bit 5: Charging interlock
    // Bit 6: DCDC control
    // Bit 7: Contactor precharge
    CVC_data[BMS_CHARGER_ENABLED].data = (frame.data[1] >> 0) & 0x01;
    CVC_data[BMS_HEATER_ENABLED].data = (frame.data[1] >> 1) & 0x01;
    CVC_data[BMS_BATTERY_CONTACTOR].data = (frame.data[1] >> 2) & 0x01;
    CVC_data[BMS_BATTERY_FAN].data = (frame.data[1] >> 3) & 0x01;
    CVC_data[BMS_POWER_REDUCTION].data = (frame.data[1] >> 4) & 0x01;
    CVC_data[BMS_CHARGING_INTERLOCK].data = (frame.data[1] >> 5) & 0x01;
    CVC_data[BMS_DCDC_ENABLED].data = (frame.data[1] >> 6) & 0x01;
    CVC_data[BMS_CONTACTOR_PRECHARGE].data = (frame.data[1] >> 7) & 0x01;
    // Byte 2: Number of live cells (MSB)
    // Byte 3: Charging state
    // 0 = Disconnected
    // 1 = Pre-heating
    // 2 = Pre-charging
    // 3 = Main charging
    // 4 = Balancing
    // 5 = Charging finished
    // 6 = Charging error
    CVC_data[BMS_CHARGING_STATE].data = frame.data[3];
    // Byte 4: Charging stage duration (MSB)
    // Byte 5: Charging stage duration (LSB)
    CVC_data[BMS_CHARGING_DURATION].data = (frame.data[4] << 8) | frame.data[5];
    // Charging stage duration is in minutes
    // Byte 6: Last charging error
    // 0 = No error
    // 1 = No cell communication at start of charging or communication lost during pre-charging (using CAN charger)
    // 2 = No cell communication using non-CAN charger
    // 3 = Maximum chargin stage duration exceeded
    // 4 = Cell communication lost during charging (CAN charger)
    // 5 = Cannot set cell module balancing threshold
    // 6 = Cell or cell module temperature too high
    // 7 = Cell commmunication lost during pre-heating (CAN charger)
    CVC_data[BMS_LAST_CHARGING_ERROR].data = frame.data[6];
    // Byte 7: Number of live cells (LSB)
    CVC_data[BMS_LIVE_CELL_COUNT].data = (frame.data[2] << 8) | frame.data[7];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Diagnostic Codes CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_DiagnosticCodes(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Protection flags (LSB)
    // Bit 0: Cell undervoltage
    // Bit 1: Cell overvoltage
    // Bit 2: Discharge overcurrent
    // Bit 3: Charge overcurrent
    // Bit 4: Cell module overheat
    // Bit 5: Leakage
    // Bit 6: No cell communication
    // Bit 7: Don't care
    CVC_data[BMS_UNDERVOLTAGE].data = (frame.data[0] >> 0) & 0x01;
    CVC_data[BMS_OVERVOLTAGE].data = (frame.data[0] >> 1) & 0x01;
    CVC_data[BMS_DISCHARGE_OVERCURRENT].data = (frame.data[0] >> 2) & 0x01;
    CVC_data[BMS_CHARGE_OVERCURRENT].data = (frame.data[0] >> 3) & 0x01;
    CVC_data[BMS_CELL_MODULE_OVERHEAT].data = (frame.data[0] >> 4) & 0x01;
    CVC_data[BMS_LEAKAGE].data = (frame.data[0] >> 5) & 0x01;
    CVC_data[BMS_NO_CELL_COMMUNICATION].data = (frame.data[0] >> 6) & 0x01;
    // Byte 1: Warning (power reduction) flags
    // Bit 0: Low cell voltage
    // Bit 1: High discharge current
    // Bit 2: High cell module temperature
    CVC_data[BMS_WARN_LOW_CELL_VOLTAGE].data = (frame.data[1] >> 0) & 0x01;
    CVC_data[BMS_WARN_HIGH_DISCHARGE_CURRENT].data = (frame.data[1] >> 1) & 0x01;
    CVC_data[BMS_WARN_HIGH_CELL_MODULE_TEMP].data = (frame.data[1] >> 2) & 0x01;
    // Byte 3: Protection flags (MSB)
    // Bit 3: Cell overheat
    // Bit 4: No current sensor
    // Bit 5: Pack undervoltage
    CVC_data[BMS_CELL_OVERHEAT].data = (frame.data[3] >> 3) & 0x01;
    CVC_data[BMS_NO_CURRENT_SENSOR].data = (frame.data[3] >> 4) & 0x01;
    CVC_data[BMS_PACK_UNDERVOLTAGE].data = (frame.data[3] >> 5) & 0x01;
    // Byte 4: Battery status flags
    // Bit 0: Cell voltages valid
    // Bit 1: Cell module temperatures valid
    // Bit 2: Cell balancing rates valid
    // Bit 3: Cell balancing thresholds valid
    // Bit 4: Charging finished
    // Bit 5: Cell temperatures valid
    CVC_data[BMS_CELL_VOLTAGE_VALID].data = (frame.data[4] >> 0) & 0x01;
    CVC_data[BMS_CELL_MODULE_TEMP_VALID].data = (frame.data[4] >> 1) & 0x01;
    CVC_data[BMS_BALANCE_RATE_VALID].data = (frame.data[4] >> 2) & 0x01;
    CVC_data[BMS_LIVE_CELL_COUNT_VALID].data = (frame.data[4] >> 3) & 0x01;
    CVC_data[BMS_CHARGING_FINISHED].data = (frame.data[4] >> 4) & 0x01;
    CVC_data[BMS_CELL_TEMP_VALID].data = (frame.data[4] >> 5) & 0x01;
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Battery Voltage Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_BatteryVoltageOverallParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Min cell voltage
    CVC_data[BMS_MIN_CELL_VOLTAGE].data = frame.data[0] + 200;
    // Byte 1: Max cell voltage
    CVC_data[BMS_MAX_CELL_VOLTAGE].data = frame.data[1] + 200;
    // Byte 2: Average cell voltage
    CVC_data[BMS_AVG_CELL_VOLTAGE].data = frame.data[2] + 200;
    // Byte 4: Total voltage (LSB)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[4]);
    // Byte 3: Total voltage (2nd byte)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[3] << 8) | CVC_data[BMS_TOTAL_VOLTAGE].data;
    // Byte 6: Total voltage (3rd byte)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[6] << 16) | CVC_data[BMS_TOTAL_VOLTAGE].data;
    // Byte 5: Total voltage (MSB)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[5] << 24) | CVC_data[BMS_TOTAL_VOLTAGE].data;
    // Byte 7: Don't care
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Cell Module Temperature Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellModuleTemperatureOverallParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Min cell module temperature
    CVC_data[BMS_MIN_CELL_MODULE_TEMP].data = frame.data[0];
    // Byte 1: Max cell module temperature
    CVC_data[BMS_MAX_CELL_MODULE_TEMP].data = frame.data[1];
    // Byte 2: Average cell module temperature
    CVC_data[BMS_AVG_CELL_MODULE_TEMP].data = frame.data[2];
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Cell Temperature Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellTemperatureOverallParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Min cell temperature
    CVC_data[BMS_MIN_CELL_TEMP].data = frame.data[0];
    // Byte 1: Max cell temperature
    CVC_data[BMS_MAX_CELL_TEMP].data = frame.data[1];
    // Byte 2: Average cell temperature
    CVC_data[BMS_AVG_CELL_TEMP].data = frame.data[2];
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Cell Balancing Rate Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellBalancingRateOverallParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Min cell balancing rate
    CVC_data[BMS_MIN_BALANCE_RATE].data = frame.data[0];
    // Byte 1: Max cell balancing rate
    CVC_data[BMS_MAX_BALANCE_RATE].data = frame.data[1];
    // Byte 2: Average cell balancing rate
    CVC_data[BMS_AVG_BALANCE_RATE].data = frame.data[2];
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit State of Charge Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_StateOfChargeParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Current (MSB)
    // Byte 1: Current (LSB)
    CVC_data[BMS_CURRENT].data = (frame.data[0] << 8) | frame.data[1];
    // Byte 2: Estimated charge (MSB)
    // Byte 3: Estimated charge (LSB)
    CVC_data[BMS_ESTIMATED_CHARGE].data = (frame.data[2] << 8) | frame.data[3];
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Estimated state of charge
    CVC_data[BMS_ESTIMATED_SOC].data = frame.data[6];
    // Byte 7: Don't care
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Configuration Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_ConfigurationParameters(CAN_Queue_Frame_t frame) {
    // TODO: Implement
}

/**
 * @brief Parses EMUS BMS 29-bit Contactor Control CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_ContactorControl(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Contactor state
    CVC_data[BMS_CONTACTOR_STATE].data = frame.data[0];
    // Byte 1: Don't care
    // Byte 2: Don't care
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Energy Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_EnergyParameters(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: Estimated Consumption (MSB)
    // Byte 1: Estimated Consumption (LSB)
    CVC_data[BMS_ESTIMATED_CONSUMPTION].data = (frame.data[0] << 8) | frame.data[1];
    // Byte 2: Estimated Energy (MSB)
    // Byte 3: Estimated Energy (LSB)
    CVC_data[BMS_ESTIMATED_ENERGY].data = (frame.data[2] << 8) | frame.data[3];
    // Byte 4: Estimated Distance Remaining (MSB)
    // Byte 5: Estimated Distance Remaining (LSB)
    CVC_data[BMS_ESTIMATED_DISTANCE_REMAINING].data = (frame.data[4] << 8) | frame.data[5];
    // Byte 6: Distance Traveled (MSB)
    // Byte 7: Distance Traveled (LSB)
    CVC_data[BMS_DISTANCE_TRAVELED].data = (frame.data[6] << 8) | frame.data[7];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses EMUS BMS 29-bit Events CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_Events(CAN_Queue_Frame_t frame) {
    // TODO: Implement
}

/**
 * @brief Parses VDM GPS Latitude and Longitude 29-bit CAN message (0x0000A0000).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSLatitudeLongitude(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0-3: GPS Latitude
    CVC_data[VDM_GPS_LATITUDE].data = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
    // Byte 4-7: GPS Longitude
    CVC_data[VDM_GPS_LONGITUDE].data = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses VDM GPS data including speed, altitude, true course, satellites in use, and GPS validity (0x0000A0001).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSData(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0,1: GPS Speed
    CVC_data[VDM_GPS_SPEED].data = (frame.data[0] << 8) | frame.data[1];
    // Byte 2, 3: GPS Altitude
    CVC_data[VDM_GPS_ALTITUDE].data = (frame.data[2] << 8) | frame.data[3];
    // Byte 4, 5: GPS True Course
    CVC_data[VDM_GPS_TRUE_COURSE].data = (frame.data[4] << 8) | frame.data[5];
    // Byte 6: Satellites in use
    CVC_data[VDM_GPS_SATELLITES_IN_USE].data = frame.data[6];
    // Byte 7: GPS Validity
    CVC_data[VDM_GPS_DATA_VALID].data = frame.data[7];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses VDM GPS date and time information (0x0000A0002).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSDateTime(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0: GPS Validity
    CVC_data[VDM_GPS_DATE_TIME_VALID].data = frame.data[0];
    // Byte 1: UTC Year
    CVC_data[VDM_UTC_YEAR].data = frame.data[1];
    // Byte 2: UTC Month
    CVC_data[VDM_UTC_MONTH].data = frame.data[2];
    // Byte 3: UTC Day
    CVC_data[VDM_UTC_DAY].data = frame.data[3];
    // Byte 5: UTC Hour
    CVC_data[VDM_UTC_HOUR].data = frame.data[5];
    // Byte 6: UTC Minute
    CVC_data[VDM_UTC_MINUTE].data = frame.data[6];
    // Byte 7: UTC Second
    CVC_data[VDM_UTC_SECOND].data = frame.data[7];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses acceleration data for the X, Y, and Z axes (0x0000A0003).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing bytes of acceleration data.
 * @retval None
 */
void CAN_Parse_VDM_AccelerationData(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0, 1: X-Axis Acceleration
    CVC_data[VDM_ACCELERATION_X].data = (frame.data[0] << 8) | frame.data[1];
    // Byte 2, 3: Y-Axis Acceleration
    CVC_data[VDM_ACCELERATION_Y].data = (frame.data[2] << 8) | frame.data[3];
    // Byte 4, 5: Z-Axis Acceleration
    CVC_data[VDM_ACCELERATION_Z].data = (frame.data[4] << 8) | frame.data[5];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses yaw rate data for the X, Y, and Z axes (0x0000A0004).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing bytes of yaw rate data.
 * @retval None
 */
void CAN_Parse_VDM_YawRateData(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0, 1: X-Axis Yaw Rate
    CVC_data[VDM_YAW_RATE_X].data = (frame.data[0] << 8) | frame.data[1];
    // Byte 2, 3: Y-Axis Yaw Rate
    CVC_data[VDM_YAW_RATE_Y].data = (frame.data[2] << 8) | frame.data[3];
    // Byte 4, 5: Z-Axis Yaw Rate
    CVC_data[VDM_YAW_RATE_Z].data = (frame.data[4] << 8) | frame.data[5];
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parse Sensor Board CAN messages.
 * @param frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */
void CAN_Parse_SensorBoard(CAN_Queue_Frame_t frame) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    // Byte 0, 1: Right Wheel RPM
    CVC_data[SENSOR_RIGHT_WHEELSPEED].data = frame.data[0] | (frame.data[1] << 8);
    // Byte 2, 3: Left Wheel RPM
    CVC_data[SENSOR_LEFT_WHEELSPEED].data = frame.data[2] | (frame.data[3] << 8);
    // Byte 4: Brake Pressure
    CVC_data[SENSOR_BRAKESWITCH].data = frame.data[4];
    // Byte 5: Steering Angle
    CVC_data[SENSOR_STEERING_ANGLE].data = frame.data[5];
    // Byte 6, 7: Throttle Position
    CVC_data[SENSOR_THROTTLE_ADC].data = frame.data[6] | (frame.data[7] << 8);
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Temperatures #1 CAN message. (0)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_Inverter_Temp1(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: Power Module Phase A Temperature
        CVC_data[INVERTER1_POWER_MODULE_A_TEMP].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Power Module Phase B Temperature
        CVC_data[INVERTER1_POWER_MODULE_B_TEMP].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Power Module Phase  C Temperature
        CVC_data[INVERTER1_POWER_MODULE_C_TEMP].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Gate Driver Board Temperature
        CVC_data[INVERTER1_GATE_DRIVER_BOARD_TEMP].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: Power Module Phase A Temperature
        CVC_data[INVERTER2_POWER_MODULE_A_TEMP].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Power Module Phase B Temperature
        CVC_data[INVERTER2_POWER_MODULE_B_TEMP].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Power Module Phase  C Temperature
        CVC_data[INVERTER2_POWER_MODULE_C_TEMP].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Gate Driver Board Temperature
        CVC_data[INVERTER2_GATE_DRIVER_BOARD_TEMP].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Temperatures #2 CAN message. (1)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp2(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: Temperature of Control Board
        CVC_data[INVERTER1_CONTROL_BOARD_TEMP].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Temperature read from RTD input #1
        CVC_data[INVERTER1_RTD_INPUT_1_TEMP].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Temperature read from RTD Input #2
        CVC_data[INVERTER1_RTD_INPUT_2_TEMP].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Stall Burst Model Temperature
        CVC_data[INVERTER1_STALL_BURST_MODEL_TEMP].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: Temperature of Control Board
        CVC_data[INVERTER1_CONTROL_BOARD_TEMP].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Temperature read from RTD input #1
        CVC_data[INVERTER1_RTD_INPUT_1_TEMP].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Temperature read from RTD Input #2
        CVC_data[INVERTER1_RTD_INPUT_2_TEMP].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Stall Burst Model Temperature
        CVC_data[INVERTER1_STALL_BURST_MODEL_TEMP].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Temperatures #3 & Torque Shudder CAN message. (2)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp3TorqueShudder(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: Coolant Temperature
        CVC_data[INVERTER1_COOLANT_TEMP].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Estimated hot spot temperature internal to inverter.
        CVC_data[INVERTER1_HOT_SPOT_TEMP].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Filtered temperature value from the motor temperature sensor
        CVC_data[INVERTER1_MOTOR_TEMP].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: A value of torque used in shudder compensation.
        CVC_data[INVERTER1_TORQUE_SHUDDER].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: Coolant Temperature
        CVC_data[INVERTER2_COOLANT_TEMP].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Estimated hot spot temperature internal to inverter.
        CVC_data[INVERTER2_HOT_SPOT_TEMP].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Filtered temperature value from the motor temperature sensor
        CVC_data[INVERTER2_MOTOR_TEMP].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: A value of torque used in shudder compensation.
        CVC_data[INVERTER2_TORQUE_SHUDDER].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Analog Input Status CAN message. (0x0A3)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

// 01234567 89012345 67890123 45678901 23456789 01234567 89012345 67890123
void CAN_Parse_Inverter_AnalogInputStatus(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    uint64_t full = 0;
    // Combine all 8 bytes into a single 64-bit integer
    for (uint8_t i = 0; i < 8; i++) {
        full |= (uint64_t)frame.data[i] << (i * 8);
    }
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Bits 0-9: Analog Input #1
        CVC_data[INVERTER1_ANALOG_INPUT_1].data = (full >> 0) & 0x3FF;
        // Bits 10-19: Analog Input #2
        CVC_data[INVERTER1_ANALOG_INPUT_2].data = (full >> 10) & 0x3FF;
        // Bits 20-29: Analog Input #3
        CVC_data[INVERTER1_ANALOG_INPUT_3].data = (full >> 20) & 0x3FF;
        // Bits 32-41: Analog Input #4
        CVC_data[INVERTER1_ANALOG_INPUT_4].data = (full >> 32) & 0x3FF;
        // Bits 42-51: Analog Input #5
        CVC_data[INVERTER1_ANALOG_INPUT_5].data = (full >> 42) & 0x3FF;
        // Bits 52-61: Analog Input #6
        CVC_data[INVERTER1_ANALOG_INPUT_6].data = (full >> 52) & 0x3FF;
    } else {
        // Bits 0-9: Analog Input #1
        CVC_data[INVERTER2_ANALOG_INPUT_1].data = (full >> 0) & 0x3FF;
        // Bits 10-19: Analog Input #2
        CVC_data[INVERTER2_ANALOG_INPUT_2].data = (full >> 10) & 0x3FF;
        // Bits 20-29: Analog Input #3
        CVC_data[INVERTER2_ANALOG_INPUT_3].data = (full >> 20) & 0x3FF;
        // Bits 32-41: Analog Input #4
        CVC_data[INVERTER2_ANALOG_INPUT_4].data = (full >> 32) & 0x3FF;
        // Bits 42-51: Analog Input #5
        CVC_data[INVERTER2_ANALOG_INPUT_5].data = (full >> 42) & 0x3FF;
        // Bits 52-61: Analog Input #6
        CVC_data[INVERTER2_ANALOG_INPUT_6].data = (full >> 52) & 0x3FF;
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Digital Input Status CAN message. (4)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_DigitalInputStatus(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0: Status of Digital Input #1, Forward switch
        CVC_data[INVERTER1_FORWARD_SWITCH].data = frame.data[1];
        // Byte 1: Status of Digital Input #2, Reverse switch
        CVC_data[INVERTER1_REVERSE_SWITCH].data = frame.data[0];
        // Byte 2: Status of Digital Input #3, Reverse switch
        CVC_data[INVERTER1_BRAKE_SWITCH].data = frame.data[3];
        // Byte 3: Status of Digital Input #4, Regen Disable switch
        CVC_data[INVERTER1_REGEN_DISABLE_SWITCH].data = frame.data[2];
        // Byte 4: Status of Digital Input #5, Ignition switch
        CVC_data[INVERTER1_IGNITION_SWITCH].data = frame.data[5];
        // Byte 5: Status of Digital Input #6, Start switch
        CVC_data[INVERTER1_START_SWITCH].data = frame.data[4];
        // Byte 6: Status of Digital Input #7, Valet Mode
        CVC_data[INVERTER1_VALET_MODE].data = frame.data[7];
    } else {
        // Byte 0: Status of Digital Input #1, Forward switch
        CVC_data[INVERTER2_FORWARD_SWITCH].data = frame.data[1];
        // Byte 1: Status of Digital Input #2, Reverse switch
        CVC_data[INVERTER2_REVERSE_SWITCH].data = frame.data[0];
        // Byte 2: Status of Digital Input #3, Reverse switch
        CVC_data[INVERTER2_BRAKE_SWITCH].data = frame.data[3];
        // Byte 3: Status of Digital Input #4, Regen Disable switch
        CVC_data[INVERTER2_REGEN_DISABLE_SWITCH].data = frame.data[2];
        // Byte 4: Status of Digital Input #5, Ignition switch
        CVC_data[INVERTER2_IGNITION_SWITCH].data = frame.data[5];
        // Byte 5: Status of Digital Input #6, Start switch
        CVC_data[INVERTER2_START_SWITCH].data = frame.data[4];
        // Byte 6: Status of Digital Input #7, Valet Mode
        CVC_data[INVERTER2_VALET_MODE].data = frame.data[7];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Motor Position Parameters CAN message. (5)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_MotorPositionParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: Motor Angle
        CVC_data[INVERTER1_MOTOR_ANGLE].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Motor Speed
        CVC_data[INVERTER1_MOTOR_SPEED].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Electrical Output Frequency
        CVC_data[INVERTER1_ELECTRICAL_OUTPUT_FREQUENCY].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Delta Resolver Filtered
        CVC_data[INVERTER1_DELTA_RESOLVER_FILTERED].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: Motor Angle
        CVC_data[INVERTER2_MOTOR_ANGLE].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Motor Speed
        CVC_data[INVERTER2_MOTOR_SPEED].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Electrical Output Frequency
        CVC_data[INVERTER2_ELECTRICAL_OUTPUT_FREQUENCY].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Delta Resolver Filtered
        CVC_data[INVERTER2_DELTA_RESOLVER_FILTERED].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Current Parameters CAN message. (6)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_CurrentParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: Phase A Current
        CVC_data[INVERTER1_PHASE_A_CURRENT].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Phase B Current
        CVC_data[INVERTER1_PHASE_B_CURRENT].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Phase C Current
        CVC_data[INVERTER1_PHASE_C_CURRENT].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER1_DC_BUS_CURRENT].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: Phase A Current
        CVC_data[INVERTER2_PHASE_A_CURRENT].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: Phase B Current
        CVC_data[INVERTER2_PHASE_B_CURRENT].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Phase C Current
        CVC_data[INVERTER2_PHASE_C_CURRENT].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER2_DC_BUS_CURRENT].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Voltage Parameters CAN message. (7)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_VoltageParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: The actual measured value of the DC bus voltage
        CVC_data[INVERTER1_DC_BUS_VOLTAGE].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: The calculated value of the output voltage, in peak line-neutral volts
        CVC_data[INVERTER1_OUTPUT_VOLTAGE].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Measured value of the voltage between Phase A and Phase B (VAB) when the inverter is disabled.
        // Vd voltage when the inverter is enabled.
        CVC_data[INVERTER1_VAB_VD_VOLTAGE].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER1_VAB_VQ_VOLTAGE].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: The actual measured value of the DC bus voltage
        CVC_data[INVERTER2_DC_BUS_VOLTAGE].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: The calculated value of the output voltage, in peak line-neutral volts
        CVC_data[INVERTER2_OUTPUT_VOLTAGE].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Measured value of the voltage between Phase A and Phase B (VAB) when the inverter is disabled.
        // Vd voltage when the inverter is enabled.
        CVC_data[INVERTER2_VAB_VD_VOLTAGE].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER2_VAB_VQ_VOLTAGE].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Flux Parameters CAN message. (8)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_FluxParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: The commanded flux
        CVC_data[INVERTER1_FLUX_COMMAND].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: The estimated flux
        CVC_data[INVERTER1_FLUX_FEEDBACK].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: D-axis current feedback
        CVC_data[INVERTER1_ID_CURRENT].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Q-axis current feedback
        CVC_data[INVERTER1_IQ_CURRENT].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: The commanded flux
        CVC_data[INVERTER2_FLUX_COMMAND].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: The estimated flux
        CVC_data[INVERTER2_FLUX_FEEDBACK].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: D-axis current feedback
        CVC_data[INVERTER2_ID_CURRENT].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Q-axis current feedback
        CVC_data[INVERTER2_IQ_CURRENT].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Internal Voltage Paramaters CAN message. (9)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_InternalVoltageParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: 1.5 Reference Voltage
        CVC_data[INVERTER1_1_5_REFERENCE_VOLTAGE].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: 2.5 Reference Voltage
        CVC_data[INVERTER1_2_5_REFERENCE_VOLTAGE].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: 5.0 Reference Voltage
        CVC_data[INVERTER1_5_0_REFERENCE_VOLTAGE].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: 12.0 Reference Voltage
        CVC_data[INVERTER1_12_0_REFERENCE_VOLTAGE].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: 1.5 Reference Voltage
        CVC_data[INVERTER2_1_5_REFERENCE_VOLTAGE].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: 2.5 Reference Voltage
        CVC_data[INVERTER2_2_5_REFERENCE_VOLTAGE].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: 5.0 Reference Voltage
        CVC_data[INVERTER2_5_0_REFERENCE_VOLTAGE].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: 12.0 Reference Voltage
        CVC_data[INVERTER2_12_0_REFERENCE_VOLTAGE].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Internal States CAN message. (10)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_InternalStateParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0: VSM State
        CVC_data[INVERTER1_VSM_STATE].data = frame.data[1];
        // Byte 1: PWM Frequency
        CVC_data[INVERTER1_PWM_FREQUENCY].data = frame.data[0];
        // Byte 2: Inverter State
        CVC_data[INVERTER1_STATE].data = frame.data[3];
        // Byte 3: Relay state
        // Bit 0: Relay 1 status
        // Bit 1: Relay 2 status
        // Bit 2: Relay 3 status
        // Bit 3: Relay 4 status
        // Bit 4: Relay 5 status
        // Bit 5: Relay 6 status
        CVC_data[INVERTER1_RELAY_1_STATUS].data = (frame.data[2] >> 0) & 0x01;
        CVC_data[INVERTER1_RELAY_2_STATUS].data = (frame.data[2] >> 1) & 0x01;
        CVC_data[INVERTER1_RELAY_3_STATUS].data = (frame.data[2] >> 2) & 0x01;
        CVC_data[INVERTER1_RELAY_4_STATUS].data = (frame.data[2] >> 3) & 0x01;
        CVC_data[INVERTER1_RELAY_5_STATUS].data = (frame.data[2] >> 4) & 0x01;
        CVC_data[INVERTER1_RELAY_6_STATUS].data = (frame.data[2] >> 5) & 0x01;
        // Byte 4
        // Bit 0: Inverter Run Mode (0 = Torque Mode, 1 = Speed Mode)
        // Bit 1: Self Sensing Assist Enable
        // Bit 5-7: Inverter Active Discharge State (000 (0) = Discharge Disabled, 001 (1) = Discharge Enabled, waiting, 010 (2) = Performing Speed Check, 011 (3) = Discharge Actively occurring, 100 (4) = Discharge Completed)
        CVC_data[INVERTER1_RUN_MODE].data = (frame.data[5] >> 0) & 0x01;
        CVC_data[INVERTER1_SELF_SENSING_ASSIST].data = (frame.data[5] >> 1) & 0x01;
        CVC_data[INVERTER1_ACTIVE_DISCHARGE_STATE].data = (frame.data[5] >> 5) & 0x07;
        // Byte 5
        // Bit 0: Inverter Command Mode
        // Bit 4-7: Rolling Counter Value
        CVC_data[INVERTER1_COMMAND_MODE].data = (frame.data[4] >> 0) & 0x01;
        CVC_data[INVERTER1_ROLLING_COUNTER_VALUE].data = (frame.data[4] >> 4) & 0x0F;
        // Byte 6
        // Bit 0: Inverter Enable State (0 = Inverter is disabled, 1 = Inverter is enabled)
        // Bit 1: Burst Model Mode (0 = Stall, 1 = High Speed)
        // Bit 6: Start Mode Active (0 = start signal has not been activated, 1 = start signal has been activated)
        // Bit 7: Inverter Enable Lockout (0 = Inverter can be enabled, 1 = Inverter cannot be enabled)
        CVC_data[INVERTER1_ENABLE_STATE].data = (frame.data[7] >> 0) & 0x01;
        CVC_data[INVERTER1_BURST_MODEL_MODE].data = (frame.data[7] >> 0) & 0x01;
        CVC_data[INVERTER1_START_MODE_ACTIVE].data = (frame.data[7] >> 6) & 0x01;
        CVC_data[INVERTER1_ENABLE_LOCKOUT].data = (frame.data[7] >> 7) & 0x01;
        // Byte 7
        // Bit 0: Direction Command (1 = Forward ;0 = Reverse, if inverter is enabled Stopped, if inverter is disabled)
        // Bit 1: BMS Active (0 = BMS Message is not being received, 1 = BMS Message is being received)
        // Bit 2: BMS Limiting Torque ( 0 = Torque is not being limited by the BMS. 1 = Torque is being limited by the BMS.)
        // Bit 3: Limit Max Speed (0 = no torque limiting is occurring. 1 = torque limiting is occurring due to the motor speed exceeding the maximum motor speed)
        // Bit 4: Limit Hot Spot (0 = Inverter hot spot temperature is below the limit. 1 = Inverter is limiting current due to regulate the maximum hot spot temperature.)
        // Bit 5: Low Speed Limiting (0 = low speed current limiting is not occurring. 1 = low speed current limiting is applied.)
        // Bit 6: Coolant Temperature Limiting (0 = Max motor current not limited due to coolant temperature.  1 = Max motor current limited due to coolan temperature.)
        // Bit 7: Limit Stall Burst Model (0 = Not limiting. 1 = Limiting due to stall burst model.)
        CVC_data[INVERTER1_DIRECTION_COMMAND].data = (frame.data[6] >> 0) & 0x01;
        CVC_data[INVERTER1_BMS_ACTIVE].data = (frame.data[6] >> 1) & 0x01;
        CVC_data[INVERTER1_BMS_LIMITING_TORQUE].data = (frame.data[6] >> 2) & 0x01;
        CVC_data[INVERTER1_LIMIT_MAX_SPEED].data = (frame.data[6] >> 3) & 0x01;
        CVC_data[INVERTER1_LIMIT_HOT_SPOT].data = (frame.data[6] >> 4) & 0x01;
        CVC_data[INVERTER1_LOW_SPEED_LIMITING].data = (frame.data[6] >> 5) & 0x01;
        CVC_data[INVERTER1_COOLANT_TEMP_LIMITING].data = (frame.data[6] >> 6) & 0x01;
        CVC_data[INVERTER1_LIMIT_STALL_BURST_MODEL].data = (frame.data[6] >> 7) & 0x01;
    } else {
        // Byte 0: VSM State
        CVC_data[INVERTER2_VSM_STATE].data = frame.data[1];
        // Byte 1: PWM Frequency
        CVC_data[INVERTER2_PWM_FREQUENCY].data = frame.data[0];
        // Byte 2: Inverter State
        CVC_data[INVERTER2_STATE].data = frame.data[3];
        // Byte 3: Relay state
        // Bit 0: Relay 1 status
        // Bit 1: Relay 2 status
        // Bit 2: Relay 3 status
        // Bit 3: Relay 4 status
        // Bit 4: Relay 5 status
        // Bit 5: Relay 6 status
        CVC_data[INVERTER2_RELAY_1_STATUS].data = (frame.data[2] >> 0) & 0x01;
        CVC_data[INVERTER2_RELAY_2_STATUS].data = (frame.data[2] >> 1) & 0x01;
        CVC_data[INVERTER2_RELAY_3_STATUS].data = (frame.data[2] >> 2) & 0x01;
        CVC_data[INVERTER2_RELAY_4_STATUS].data = (frame.data[2] >> 3) & 0x01;
        CVC_data[INVERTER2_RELAY_5_STATUS].data = (frame.data[2] >> 4) & 0x01;
        CVC_data[INVERTER2_RELAY_6_STATUS].data = (frame.data[2] >> 5) & 0x01;
        // Byte 4
        // Bit 0: Inverter Run Mode (0 = Torque Mode, 1 = Speed Mode)
        // Bit 1: Self Sensing Assist Enable
        // Bit 5-7: Inverter Active Discharge State (000 (0) = Discharge Disabled, 001 (1) = Discharge Enabled, waiting, 010 (2) = Performing Speed Check, 011 (3) = Discharge Actively occurring, 100 (4) = Discharge Completed)
        CVC_data[INVERTER2_RUN_MODE].data = (frame.data[5] >> 0) & 0x01;
        CVC_data[INVERTER2_SELF_SENSING_ASSIST].data = (frame.data[5] >> 1) & 0x01;
        CVC_data[INVERTER2_ACTIVE_DISCHARGE_STATE].data = (frame.data[5] >> 5) & 0x07;
        // Byte 5
        // Bit 0: Inverter Command Mode
        // Bit 4-7: Rolling Counter Value
        CVC_data[INVERTER2_COMMAND_MODE].data = (frame.data[4] >> 0) & 0x01;
        CVC_data[INVERTER2_ROLLING_COUNTER_VALUE].data = (frame.data[4] >> 4) & 0x0F;
        // Byte 6
        // Bit 0: Inverter Enable State (0 = Inverter is disabled, 1 = Inverter is enabled)
        // Bit 1: Burst Model Mode (0 = Stall, 1 = High Speed)
        // Bit 6: Start Mode Active (0 = start signal has not been activated, 1 = start signal has been activated)
        // Bit 7: Inverter Enable Lockout (0 = Inverter can be enabled, 1 = Inverter cannot be enabled)
        CVC_data[INVERTER2_ENABLE_STATE].data = (frame.data[7] >> 0) & 0x01;
        CVC_data[INVERTER2_BURST_MODEL_MODE].data = (frame.data[7] >> 0) & 0x01;
        CVC_data[INVERTER2_START_MODE_ACTIVE].data = (frame.data[7] >> 6) & 0x01;
        CVC_data[INVERTER2_ENABLE_LOCKOUT].data = (frame.data[7] >> 7) & 0x01;
        // Byte 7
        // Bit 0: Direction Command (1 = Forward ;0 = Reverse, if inverter is enabled Stopped, if inverter is disabled)
        // Bit 1: BMS Active (0 = BMS Message is not being received, 1 = BMS Message is being received)
        // Bit 2: BMS Limiting Torque ( 0 = Torque is not being limited by the BMS. 1 = Torque is being limited by the BMS.)
        // Bit 3: Limit Max Speed (0 = no torque limiting is occurring. 1 = torque limiting is occurring due to the motor speed exceeding the maximum motor speed)
        // Bit 4: Limit Hot Spot (0 = Inverter hot spot temperature is below the limit. 1 = Inverter is limiting current due to regulate the maximum hot spot temperature.)
        // Bit 5: Low Speed Limiting (0 = low speed current limiting is not occurring. 1 = low speed current limiting is applied.)
        // Bit 6: Coolant Temperature Limiting (0 = Max motor current not limited due to coolant temperature.  1 = Max motor current limited due to coolan temperature.)
        // Bit 7: Limit Stall Burst Model (0 = Not limiting. 1 = Limiting due to stall burst model.)
        CVC_data[INVERTER2_DIRECTION_COMMAND].data = (frame.data[6] >> 0) & 0x01;
        CVC_data[INVERTER2_BMS_ACTIVE].data = (frame.data[6] >> 1) & 0x01;
        CVC_data[INVERTER2_BMS_LIMITING_TORQUE].data = (frame.data[6] >> 2) & 0x01;
        CVC_data[INVERTER2_LIMIT_MAX_SPEED].data = (frame.data[6] >> 3) & 0x01;
        CVC_data[INVERTER2_LIMIT_HOT_SPOT].data = (frame.data[6] >> 4) & 0x01;
        CVC_data[INVERTER2_LOW_SPEED_LIMITING].data = (frame.data[6] >> 5) & 0x01;
        CVC_data[INVERTER2_COOLANT_TEMP_LIMITING].data = (frame.data[6] >> 6) & 0x01;
        CVC_data[INVERTER2_LIMIT_STALL_BURST_MODEL].data = (frame.data[6] >> 7) & 0x01;
    }
    xSemaphoreGive(CVC_DataMutex);
}

/**
 * @brief Parses Inverter 29-bit Fault Codes CAN message. (11)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_FaultCodes(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: POST Fault Lo
        CVC_data[INVERTER1_POST_FAULT_LO].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: POST Fault Hi
        CVC_data[INVERTER1_POST_FAULT_HI].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Run Fault Lo
        CVC_data[INVERTER1_RUN_FAULT_LO].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Run Fault Hi
        CVC_data[INVERTER1_RUN_FAULT_HI].data = (frame.data[7] << 8) | frame.data[6];
    } else {
        // Byte 0, 1: POST Fault Lo
        CVC_data[INVERTER2_POST_FAULT_LO].data = (frame.data[1] << 8) | frame.data[0];
        // Byte 2, 3: POST Fault Hi
        CVC_data[INVERTER2_POST_FAULT_HI].data = (frame.data[3] << 8) | frame.data[2];
        // Byte 4, 5: Run Fault Lo
        CVC_data[INVERTER2_RUN_FAULT_LO].data = (frame.data[5] << 8) | frame.data[4];
        // Byte 6, 7: Run Fault Hi
        CVC_data[INVERTER2_RUN_FAULT_HI].data = (frame.data[7] << 8) | frame.data[6];
    }
    xSemaphoreGive(CVC_DataMutex);
}
/**
 * @brief Parses Inverter 29-bit High Speed CAN message. (0x0B0)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_HighSpeedParameters(CAN_Queue_Frame_t frame, bool isFirstInverter) {
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    if (isFirstInverter) {
        // Byte 0, 1: Torque Command
        CVC_data[INVERTER1_TORQUE_COMMAND_HI].data = (frame.data[1] << 8) | frame.data[0];
        CVC_data[INVERTER1_TORQUE_FEEDBACK_HI].data = (frame.data[3] << 8) | frame.data[2];
        CVC_data[INVERTER1_MOTOR_SPEED_HI].data = (frame.data[5] << 8) | frame.data[4];
        CVC_data[INVERTER1_DC_BUS_VOLTAGE_HI].data = (frame.data[7] << 8) | frame.data[6];
        // Byte 0, 1: Torque Command
        CVC_data[INVERTER2_TORQUE_COMMAND_HI].data = (frame.data[1] << 8) | frame.data[0];
        CVC_data[INVERTER2_TORQUE_FEEDBACK_HI].data = (frame.data[3] << 8) | frame.data[2];
        CVC_data[INVERTER2_MOTOR_SPEED_HI].data = (frame.data[5] << 8) | frame.data[4];
        CVC_data[INVERTER2_DC_BUS_VOLTAGE_HI].data = (frame.data[7] << 8) | frame.data[6];
        xSemaphoreGive(CVC_DataMutex);
    }
}

// ===================== CAN Broadcast Task Functions =====================
/**
 * @brief Updates dashboard with information about AMS and IMD relay state, as well as current drive state
 * @param None
 * @retval None
 */
void CAN_Broadcast_Dashboard_Critical() {
    uint8_t data[8];
    // Byte 0: AMS Relay State
    data[0] = Relay_AMSState();
    // Byte 1: IMD Relay State
    data[1] = Relay_IMDState();
    // Byte 2: Drive State
    data[2] = CVC_GetData(CVC_DRIVE_MODE).data;
    // Byte 3: Reserved
    data[3] = 0;
    // Byte 4: Reserved
    data[4] = 0;
    // Byte 5: Reserved
    data[5] = 0;
    // Byte 6: Reserved
    data[6] = 0;
    // Byte 7: Reserved
    data[7] = 0;

    // Queue frame for transmission
    CAN_Transmit(CAN_BROADCAST_BASE_11, data, sizeof(data) / sizeof(data[0]), CAN_BROADCAST_USE_EXT);
}