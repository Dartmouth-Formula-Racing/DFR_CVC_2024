/*
 * cvc_can.c
 *
 *  Created on: Sep 30, 2023
 *      Author: Andrei Gerashchenko
 */

#include <FreeRTOS.h>
#include <cvc_can.h>
#include <cvc_data.h>
#include <main.h>
#include <queue.h>
#include <stdbool.h>
#include <stm32f7xx_hal_can.h>
#include <stm32f7xx_nucleo_144.h>

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

    // Blink LED1 to indicate CAN message received
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

    // Select parsing function based on CAN ID
    // TODO: Find a cleaner way to do this
    if (rx_frame.Rx_header.IDE == CAN_ID_EXT) { // 29-bit CAN messages
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


// ========================= CAN Test Functions =========================
/**
 * @brief Generates a random 32-bit integer between 0 and max.
 * @param max 
 * @return uint32_t 
 */
uint32_t rand(uint32_t max) {
    static uint64_t seed = 1;
    uint64_t a = 16807;
    uint64_t m = 2147483647;
    seed = (a * seed) % m;
    return (uint32_t)(seed % max);
}

/**
 * @brief Sends random CAN messages for testing purposes.
 * @param None
 * @retval None 
 */
void CAN_TestSend() {
    static uint32_t last = 0;
    if ((xTaskGetTickCount() - last) < CAN_TEST_SEND_INTERVAL_MS / portTICK_PERIOD_MS) {
        return;
    }
    
    CAN_Queue_Frame_t tx_frame;
    #if CAN_TEST_USE_EXT
    tx_frame.Tx_header.StdId = 0x0000;
    tx_frame.Tx_header.ExtId = rand(CAN_TEST_MAX_ID_EXT);
    tx_frame.Tx_header.IDE = CAN_ID_EXT;
    #else
    tx_frame.Tx_header.StdId = rand(CAN_TEST_MAX_ID_STD);
    tx_frame.Tx_header.ExtId = 0x0000;
    tx_frame.Tx_header.IDE = CAN_ID_STD;
    #endif
    tx_frame.Tx_header.RTR = CAN_RTR_DATA;
    tx_frame.Tx_header.DLC = 8;
    for (int i = 0; i < 8; i++) {
        tx_frame.data[i] = rand(CAN_TEST_MAX_DATA);
    }
    xQueueSendToBack(CAN_TxQueue, &tx_frame, 0);
    last = xTaskGetTickCount();
}

// ========================= CAN Parsing Functions =========================

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
    CVC_data[BMS_IGNITION].type = BOOLEAN;
    CVC_data[BMS_CHARGER_MAINS].data = (frame.data[0] >> 1) & 0x01;
    CVC_data[BMS_CHARGER_MAINS].type = BOOLEAN;
    CVC_data[BMS_FAST_CHARGE].data = (frame.data[0] >> 2) & 0x01;
    CVC_data[BMS_FAST_CHARGE].type = BOOLEAN;
    CVC_data[BMS_LEAKAGE_DETECTED].data = (frame.data[0] >> 3) & 0x01;
    CVC_data[BMS_LEAKAGE_DETECTED].type = BOOLEAN;
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
    CVC_data[BMS_CHARGER_ENABLED].type = BOOLEAN;
    CVC_data[BMS_HEATER_ENABLED].data = (frame.data[1] >> 1) & 0x01;
    CVC_data[BMS_HEATER_ENABLED].type = BOOLEAN;
    CVC_data[BMS_BATTERY_CONTACTOR].data = (frame.data[1] >> 2) & 0x01;
    CVC_data[BMS_BATTERY_CONTACTOR].type = BOOLEAN;
    CVC_data[BMS_BATTERY_FAN].data = (frame.data[1] >> 3) & 0x01;
    CVC_data[BMS_BATTERY_FAN].type = BOOLEAN;
    CVC_data[BMS_POWER_REDUCTION].data = (frame.data[1] >> 4) & 0x01;
    CVC_data[BMS_POWER_REDUCTION].type = BOOLEAN;
    CVC_data[BMS_CHARGING_INTERLOCK].data = (frame.data[1] >> 5) & 0x01;
    CVC_data[BMS_CHARGING_INTERLOCK].type = BOOLEAN;
    CVC_data[BMS_DCDC_ENABLED].data = (frame.data[1] >> 6) & 0x01;
    CVC_data[BMS_DCDC_ENABLED].type = BOOLEAN;
    CVC_data[BMS_CONTACTOR_PRECHARGE].data = (frame.data[1] >> 7) & 0x01;
    CVC_data[BMS_CONTACTOR_PRECHARGE].type = BOOLEAN;
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
    CVC_data[BMS_CHARGING_STATE].type = UINT;
    // Byte 4: Charging stage duration (MSB)
    // Byte 5: Charging stage duration (LSB)
    CVC_data[BMS_CHARGING_DURATION].data = (frame.data[4] << 8) | frame.data[5];
    CVC_data[BMS_CHARGING_DURATION].type = UINT;
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
    CVC_data[BMS_LAST_CHARGING_ERROR].type = UINT;
    // Byte 7: Number of live cells (LSB)
    CVC_data[BMS_LIVE_CELL_COUNT].data = (frame.data[2] << 8) | frame.data[7];
    CVC_data[BMS_LIVE_CELL_COUNT].type = UINT;
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
    CVC_data[BMS_UNDERVOLTAGE].type = BOOLEAN;
    CVC_data[BMS_OVERVOLTAGE].data = (frame.data[0] >> 1) & 0x01;
    CVC_data[BMS_OVERVOLTAGE].type = BOOLEAN;
    CVC_data[BMS_DISCHARGE_OVERCURRENT].data = (frame.data[0] >> 2) & 0x01;
    CVC_data[BMS_DISCHARGE_OVERCURRENT].type = BOOLEAN;
    CVC_data[BMS_CHARGE_OVERCURRENT].data = (frame.data[0] >> 3) & 0x01;
    CVC_data[BMS_CHARGE_OVERCURRENT].type = BOOLEAN;
    CVC_data[BMS_CELL_MODULE_OVERHEAT].data = (frame.data[0] >> 4) & 0x01;
    CVC_data[BMS_CELL_MODULE_OVERHEAT].type = BOOLEAN;
    CVC_data[BMS_LEAKAGE].data = (frame.data[0] >> 5) & 0x01;
    CVC_data[BMS_LEAKAGE].type = BOOLEAN;
    CVC_data[BMS_NO_CELL_COMMUNICATION].data = (frame.data[0] >> 6) & 0x01;
    CVC_data[BMS_NO_CELL_COMMUNICATION].type = BOOLEAN;
    // Byte 1: Warning (power reduction) flags
    // Bit 0: Low cell voltage
    // Bit 1: High discharge current
    // Bit 2: High cell module temperature
    CVC_data[BMS_WARN_LOW_CELL_VOLTAGE].data = (frame.data[1] >> 0) & 0x01;
    CVC_data[BMS_WARN_LOW_CELL_VOLTAGE].type = BOOLEAN;
    CVC_data[BMS_WARN_HIGH_DISCHARGE_CURRENT].data = (frame.data[1] >> 1) & 0x01;
    CVC_data[BMS_WARN_HIGH_DISCHARGE_CURRENT].type = BOOLEAN;
    CVC_data[BMS_WARN_HIGH_CELL_MODULE_TEMP].data = (frame.data[1] >> 2) & 0x01;
    CVC_data[BMS_WARN_HIGH_CELL_MODULE_TEMP].type = BOOLEAN;
    // Byte 3: Protection flags (MSB)
    // Bit 3: Cell overheat
    // Bit 4: No current sensor
    // Bit 5: Pack undervoltage
    CVC_data[BMS_CELL_OVERHEAT].data = (frame.data[3] >> 3) & 0x01;
    CVC_data[BMS_CELL_OVERHEAT].type = BOOLEAN;
    CVC_data[BMS_NO_CURRENT_SENSOR].data = (frame.data[3] >> 4) & 0x01;
    CVC_data[BMS_NO_CURRENT_SENSOR].type = BOOLEAN;
    CVC_data[BMS_PACK_UNDERVOLTAGE].data = (frame.data[3] >> 5) & 0x01;
    CVC_data[BMS_PACK_UNDERVOLTAGE].type = BOOLEAN;
    // Byte 4: Battery status flags
    // Bit 0: Cell voltages valid
    // Bit 1: Cell module temperatures valid
    // Bit 2: Cell balancing rates valid
    // Bit 3: Cell balancing thresholds valid
    // Bit 4: Charging finished
    // Bit 5: Cell temperatures valid
    CVC_data[BMS_CELL_VOLTAGE_VALID].data = (frame.data[4] >> 0) & 0x01;
    CVC_data[BMS_CELL_VOLTAGE_VALID].type = BOOLEAN;
    CVC_data[BMS_CELL_MODULE_TEMP_VALID].data = (frame.data[4] >> 1) & 0x01;
    CVC_data[BMS_CELL_MODULE_TEMP_VALID].type = BOOLEAN;
    CVC_data[BMS_BALANCE_RATE_VALID].data = (frame.data[4] >> 2) & 0x01;
    CVC_data[BMS_BALANCE_RATE_VALID].type = BOOLEAN;
    CVC_data[BMS_LIVE_CELL_COUNT_VALID].data = (frame.data[4] >> 3) & 0x01;
    CVC_data[BMS_LIVE_CELL_COUNT_VALID].type = BOOLEAN;
    CVC_data[BMS_CHARGING_FINISHED].data = (frame.data[4] >> 4) & 0x01;
    CVC_data[BMS_CHARGING_FINISHED].type = BOOLEAN;
    CVC_data[BMS_CELL_TEMP_VALID].data = (frame.data[4] >> 5) & 0x01;
    CVC_data[BMS_CELL_TEMP_VALID].type = BOOLEAN;
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
    CVC_data[BMS_MIN_CELL_VOLTAGE].data = frame.data[0];
    CVC_data[BMS_MIN_CELL_VOLTAGE].type = UINT;
    // Byte 1: Max cell voltage
    CVC_data[BMS_MAX_CELL_VOLTAGE].data = frame.data[1];
    CVC_data[BMS_MAX_CELL_VOLTAGE].type = UINT;
    // Byte 2: Average cell voltage
    CVC_data[BMS_AVG_CELL_VOLTAGE].data = frame.data[2];
    CVC_data[BMS_AVG_CELL_VOLTAGE].type = UINT;
    // Byte 3: Total voltage (2nd byte)
    CVC_data[BMS_TOTAL_VOLTAGE].data = frame.data[3];
    CVC_data[BMS_TOTAL_VOLTAGE].type = UINT;
    // Byte 4: Total voltage (LSB)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[4] << 8) | CVC_data[BMS_TOTAL_VOLTAGE].data;
    CVC_data[BMS_TOTAL_VOLTAGE].type = UINT;
    // Byte 5: Total voltage (MSB)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[5] << 16) | CVC_data[BMS_TOTAL_VOLTAGE].data;
    CVC_data[BMS_TOTAL_VOLTAGE].type = UINT;
    // Byte 6: Total voltage (3rd byte)
    CVC_data[BMS_TOTAL_VOLTAGE].data = (frame.data[6] << 24) | CVC_data[BMS_TOTAL_VOLTAGE].data;
    CVC_data[BMS_TOTAL_VOLTAGE].type = UINT;
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
    CVC_data[BMS_MIN_CELL_MODULE_TEMP].type = UINT;
    // Byte 1: Max cell module temperature
    CVC_data[BMS_MAX_CELL_MODULE_TEMP].data = frame.data[1];
    CVC_data[BMS_MAX_CELL_MODULE_TEMP].type = UINT;
    // Byte 2: Average cell module temperature
    CVC_data[BMS_AVG_CELL_MODULE_TEMP].data = frame.data[2];
    CVC_data[BMS_AVG_CELL_MODULE_TEMP].type = UINT;
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
    CVC_data[BMS_MIN_CELL_TEMP].type = UINT;
    // Byte 1: Max cell temperature
    CVC_data[BMS_MAX_CELL_TEMP].data = frame.data[1];
    CVC_data[BMS_MAX_CELL_TEMP].type = UINT;
    // Byte 2: Average cell temperature
    CVC_data[BMS_AVG_CELL_TEMP].data = frame.data[2];
    CVC_data[BMS_AVG_CELL_TEMP].type = UINT;
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
    CVC_data[BMS_MIN_BALANCE_RATE].type = UINT;
    // Byte 1: Max cell balancing rate
    CVC_data[BMS_MAX_BALANCE_RATE].data = frame.data[1];
    CVC_data[BMS_MAX_BALANCE_RATE].type = UINT;
    // Byte 2: Average cell balancing rate
    CVC_data[BMS_AVG_BALANCE_RATE].data = frame.data[2];
    CVC_data[BMS_AVG_BALANCE_RATE].type = UINT;
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
    CVC_data[BMS_CURRENT].type = INT;
    // Byte 2: Estimated charge (MSB)
    // Byte 3: Estimated charge (LSB)
    CVC_data[BMS_ESTIMATED_CHARGE].data = (frame.data[2] << 8) | frame.data[3];
    CVC_data[BMS_ESTIMATED_CHARGE].type = INT;
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Estimated state of charge
    CVC_data[BMS_ESTIMATED_SOC].data = frame.data[6];
    CVC_data[BMS_ESTIMATED_SOC].type = UINT;
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
    CVC_data[BMS_CONTACTOR_STATE].type = UINT;
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
    CVC_data[BMS_ESTIMATED_CONSUMPTION].type = INT;
    // Byte 2: Estimated Energy (MSB)
    // Byte 3: Estimated Energy (LSB)
    CVC_data[BMS_ESTIMATED_ENERGY].data = (frame.data[2] << 8) | frame.data[3];
    CVC_data[BMS_ESTIMATED_ENERGY].type = INT;
    // Byte 4: Estimated Distance Remaining (MSB)
    // Byte 5: Estimated Distance Remaining (LSB)
    CVC_data[BMS_ESTIMATED_DISTANCE_REMAINING].data = (frame.data[4] << 8) | frame.data[5];
    CVC_data[BMS_ESTIMATED_DISTANCE_REMAINING].type = INT;
    // Byte 6: Distance Traveled (MSB)
    // Byte 7: Distance Traveled (LSB)
    CVC_data[BMS_DISTANCE_TRAVELED].data = (frame.data[6] << 8) | frame.data[7];
    CVC_data[BMS_DISTANCE_TRAVELED].type = INT;
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