/*
 * cvc_can.c
 *
 *  Created on: Sep 30, 2023
 *      Author: Andrei Gerashchenko
 */

#include <FreeRTOS.h>
#include <cvc_can.h>
#include <cvc_data.h>
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
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {
        return;
    }

    CAN_Queue_Frame_t rx_frame;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_frame.Rx_header, rx_frame.data);
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
    HAL_CAN_AddTxMessage(&hcan1, &tx_frame.Tx_header, tx_frame.data, &mailbox);
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

    // TODO: Parse message using CAN parser functions
    BSP_LED_Toggle(LED_GREEN);  // Blink green LED on reception of CAN message
}

void CAN_InterpretTask() {
    // Interval for CAN interpretation
    const TickType_t interval = CAN_PROCESSING_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    if ((xTaskGetTickCount() - last) >= interval) {
        CANInterpretRx();
        last = xTaskGetTickCount();
    }
}

// ===================== CAN Test Task Functions =====================
uint16_t rand(uint16_t max) {
    static uint32_t seed = 1;
    uint32_t a = 16807;
    uint32_t m = 2147483647;
    seed = (a * seed) % m;
    return seed % max;
}

void CAN_TestSend() {
    // Interval for CAN send task
    const TickType_t interval = CAN_TEST_SEND_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    // Data to send
    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
        data[i] = rand(0xff);
    }
    // CAN ID to send
    uint32_t id = rand(0x7ff);

    if ((xTaskGetTickCount() - last) >= interval) {
        CAN_Transmit(id, data, sizeof(data), false);
        BSP_LED_Toggle(LED_BLUE);  // Blink blue LED on CAN message send
        last = xTaskGetTickCount();
    }
}