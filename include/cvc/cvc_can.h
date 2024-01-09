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

#define CAN_EMUS_USE_EXT 1 // 1 if using extended IDs, 0 if using standard IDs
#define CAN_EMUS_BASE_29 0x19B5 // Base ID for EMUS BMS 29-bit IDs
#define CAN_EMUS_BASE_11 0x00 // Base ID for EMUS BMS 11-bit IDs

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

// ========================= CAN Parsing Functions =========================

// TODO: Implement 11-bit CAN message parsing functions
// Only 29-bit CAN message parsing functions are currently implemented
// ========== EMUS BMS Parsing Functions ==========

/**
 * @brief Parses EMUS BMS 29-bit Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_OverallParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Diagnostic Codes CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_DiagnosticCodes(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Battery Voltage Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_BatteryVoltageOverallParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Cell Module Temperature Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellModuleTemperatureOverallParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Cell Temperature Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellTemperatureOverallParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Cell Balancing Rate Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellBalancingRateOverallParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit State of Charge Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_StateOfChargeParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Configuration Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_ConfigurationParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Contactor Control CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_ContactorControl(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Energy Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_EnergyParameters(CAN_Queue_Frame_t frame);

/**
 * @brief Parses EMUS BMS 29-bit Events CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_Events(CAN_Queue_Frame_t frame);
#endif /* INC_CVC_CAN_H_ */