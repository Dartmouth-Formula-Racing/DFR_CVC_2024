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
    HAL_CAN_Start(&hcan1);
}

// ===================== CAN Communication Task Functions =====================

/**
 * @brief Internal function to receive CAN messages and place them in Rx queue.
 * @param None
 * @retval None
 */
void CANRxToQueue() {
    uint32_t rx0_count = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
    uint32_t rx1_count = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1);

    // Return early if there are no messages in either FIFO
    if (rx0_count + rx1_count == 0) {
        return;
    }

    CAN_Queue_Frame_t rx_frame;

    // Check if there is a message in Rx FIFO 0
    if (rx0_count > 0) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_frame.Rx_header, rx_frame.data);
        // Put in Rx queue
        xQueueSendToBack(CAN_RxQueue, &rx_frame, 0);
    }

    // Check if there is a message in Rx FIFO 1
    if (rx1_count > 0) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rx_frame.Rx_header, rx_frame.data);
        // Put in Rx queue
        xQueueSendToBack(CAN_RxQueue, &rx_frame, 0);
    }
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

    // Find a free mailbox
    uint32_t mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

    // Return early if there are no free mailboxes
    if (mailbox == 0) {
        return;
    }

    CAN_Queue_Frame_t tx_frame;

    // Get message from Tx queue
    xQueueReceive(CAN_TxQueue, &tx_frame, 0);
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_frame.Tx_header, tx_frame.data, &mailbox) != HAL_OK) {
        // Put message back in queue if transmission failed
        xQueueSendToBack(CAN_TxQueue, &tx_frame, 0);
    }
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
    if (rx_frame.Rx_header.StdId == 1) {
        BSP_LED_Toggle(LED_BLUE); // Blink blue LED on reception of CAN message with ID 1
    }
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

void CAN_TestSend() {
    // Interval for CAN send task
    const TickType_t interval = CAN_TEST_SEND_MS / portTICK_PERIOD_MS;
    static TickType_t last = 0;

    // Data to send
    uint8_t data[8] = {0};
    // CAN ID to send
    uint32_t id = 1;

    if ((xTaskGetTickCount() - last) >= interval) {
        CAN_Transmit(id, data, sizeof(data), false);
        last = xTaskGetTickCount();
    }
}