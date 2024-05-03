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

#define CAN_BROADCAST_MS 200 // 200 ms, 5 Hz

#define CAN_EMUS_USE_EXT 1 // 1 if using extended IDs, 0 if using standard IDs
#define CAN_EMUS_BASE_29 0x19B5 // Base ID for EMUS BMS 29-bit IDs
#define CAN_EMUS_BASE_11 0x00 // Base ID for EMUS BMS 11-bit IDs

#define CAN_VDM_USE_EXT 1 // 1 if using extended IDs, 0 if using standard IDs
#define CAN_VDM_BASE_29 0x0000A // Base ID for VDM 29-bit IDs

#define CAN_INVERTER_USE_EXT 0 // 1 if using extended IDs, 0 if using standard IDs
#define CAN_INVERTER_BASE_ID1 0x0D0 // Base ID for Inverter1 29-bit IDs
#define CAN_INVERTER_BASE_ID2 0x0A0 // Base ID for Inverter2 29-bit IDs

#define CAN_SENSORBOARD_BASE_11 0x65D
#define CAN_SENSORBOARD_USE_EXT 0

#define CAN_DASHBOARD_BASE_11 0x750
#define CAN_DASHBOARD_USE_EXT 0

#define CAN_BROADCAST_BASE_11 0x770
#define CAN_BROADCAST_USE_EXT 0

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
 * @brief Function for broadcasting CAN messages from the CVC.
 * @param None
 * @retval None 
 */
void CAN_BroadcastTask(void);

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

// ========== Dashboard Parsing Functions ==========
/**
 * @brief Parses Dashboard 11-bit CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_Dashboard(CAN_Queue_Frame_t frame);

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

/**
 * @brief Parses VDM GPS Latitude and Longitude 29-bit CAN message (0x0000A0000).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSLatitudeLongitude(CAN_Queue_Frame_t frame);

/**
 * @brief Parses VDM GPS data including speed, altitude, true course, satellites in use, and GPS validity (0x0000A0001).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSData(CAN_Queue_Frame_t frame);

/**
 * @brief Parses VDM GPS date and time information (0x0000A0002).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSDateTime(CAN_Queue_Frame_t frame);

/**
 * @brief Parses acceleration data for the X, Y, and Z axes (0x0000A0003).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing bytes of acceleration data.
 * @retval None
 */
void CAN_Parse_VDM_AccelerationData(CAN_Queue_Frame_t frame);

/**
 * @brief Parses yaw rate data for the X, Y, and Z axes (0x0000A0004).
 * @param CAN_Queue_Frame_t frame: A CAN frame containing bytes of yaw rate data.
 * @retval None
 */
void CAN_Parse_VDM_YawRateData(CAN_Queue_Frame_t frame);

/**
 * @brief Parse Sensor Board CAN messages.
 * @param frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */
void CAN_Parse_SensorBoard(CAN_Queue_Frame_t frame);

/**
 * @brief Parses Inverter 29-bit Temperatures #1 CAN message. (0x0A0)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp1(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Temperatures #2 CAN message. (0x0A1)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp2(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Temperatures #3 & Torque Shudder CAN message. (0x0A2)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp3TorqueShudder(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Analog Input Status CAN message. (0x0A3)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_AnalogInputStatus(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Digital Input Status CAN message. (0x0A4)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_DigitalInputStatus(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Motor Position Parameters CAN message. (0x0A5)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_MotorPositionParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Current Parameters CAN message. (0x0A6)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_CurrentParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Voltage Parameters CAN message. (0x0A7)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_VoltageParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Flux Parameters CAN message. (0x0A8)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_FluxParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Internal Voltage Paramaters CAN message. (0x0A9)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_InternalVoltageParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Internal States CAN message. (0x0AA)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_InternalStateParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit Fault Codes CAN message. (0x0AB)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_FaultCodes(CAN_Queue_Frame_t frame,	bool isFirstInverter);

/**
 * @brief Parses Inverter 29-bit High Speed CAN message. (0x0B0)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_HighSpeedParameters(CAN_Queue_Frame_t frame, bool isFirstInverter);


// ========================= CAN Broadcast Functions =========================
/**
 * @brief Updates dashboard with information about AMS and IMD relay state, as well as current drive state
 * @param None
 * @retval None
 */
void CAN_Broadcast_Dashboard_Critical(void);

#endif /* INC_CVC_CAN_H_ */