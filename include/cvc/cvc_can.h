/*
 * cvc_can.h
 *
 *  Created on: Sep 30, 2023
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_CAN_H_
#define INC_CVC_CAN_H_

#include <stdbool.h>
#include <stm32f7xx_hal_can.h>
#include <FreeRTOS.h>
#include <queue.h>

#define CAN_QUEUE_LENGTH 10 // Length of CAN Tx queue
#define CAN_PERIPHERAL_MS 1 // 1 ms, 1000 Hz
#define CAN_PROCESSING_MS 1 // 1 ms, 1000 Hz
#define CAN_TEST_SEND_MS 500 // 500 ms, 2 Hz

// CAN Tx and Rx queues
extern QueueHandle_t CAN_TxQueue;
extern QueueHandle_t CAN_RxQueue;

/* Struct to hold messages used in CAN message queues */
typedef struct
{
	union
	{
		CAN_TxHeaderTypeDef	Tx_header;
		CAN_RxHeaderTypeDef	Rx_header;
	};
	uint8_t data[8];
} CAN_Queue_Frame_t;

/**
 * @brief Configures CAN peripheral.
 * @param None
 * @retval None 
 */
void CAN_Configure(void);

/**
 * @brief Function for interacting with CAN peripheral.
 * @param None
 * @retval None 
 */
void CAN_CommunicationTask(void);

/**
 * @brief Function for queuing CAN messages to be transmitted.
 * Handles both standard and extended CAN messages.
 * @param uint32_t id: CAN message ID (11-bit or 29-bit)
 * @param uint8_t data[8]: Array of 8 bytes of data to be transmitted
 * @param uint8_t len: Length of data array (0-8)
 * @param bool isExt: True if extended CAN message, false if standard
 * @retval None
 */
void CAN_Transmit(uint32_t id, uint8_t data[8], uint8_t len, bool isExt);

/**
 * @brief Function for interpreting CAN messages.
 * @param None
 * @retval None 
 */
void CAN_InterpretTask(void);

/**
 * @brief Function for testing CAN communication.
 * @param None
 * @retval None 
 */
void CAN_TestSend(void);

#endif /* INC_CVC_CAN_H_ */