/*
 * cvc_plc.h
 *
 *  Created on: Sep 27, 2023
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_PLC_H_
#define INC_CVC_PLC_H_

#include <stm32f7xx.h>

// Bitfield struct for PLC inputs (8 bits, each bit represents a single input)
typedef struct {
    uint8_t IN1 : 1;
    uint8_t IN2 : 1;
    uint8_t IN3 : 1;
    uint8_t IN4 : 1;
    uint8_t IN5 : 1;
    uint8_t IN6 : 1;
    uint8_t IN7 : 1;
    uint8_t IN8 : 1;
} PLC_INPUT_t;

// Bitfield struct for PLC outputs (8 bits, each bit represents a single output)
typedef struct {
    uint8_t OUT1 : 1;
    uint8_t OUT2 : 1;
    uint8_t OUT3 : 1;
    uint8_t OUT4 : 1;
    uint8_t OUT5 : 1;
    uint8_t OUT6 : 1;
    uint8_t OUT7 : 1;
    uint8_t OUT8 : 1;
} PLC_OUTPUT_t;

/**
 * @brief Blinks PLC output LEDs in a certain order to demonstrate communication.
 * This function should be called in the Control task.
 * @param argument: Not used
 * @retval None
 */
void PLC_BlinkTask(void);

/**
 * @brief Function imlementing the PLC_CommunicationTask thread.
 * This function handles communication between the PLC expansion board and the CVC.
 * This function should be called in the Communication task.
 * @param argument: Not used
 * @retval None
 */
void PLC_CommunicationTask(void);

/**
 * @brief Initializes the PLC expansion board.
 * @param None
 * @retval None
 */
void PLC_Configure(void);

#endif /* INC_CVC_PLC_H_ */