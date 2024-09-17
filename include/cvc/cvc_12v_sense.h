/*
 * cvc_12v_sense.h
 *
 *  Created on: Apr 7, 2024
 *      Author: Andrei Gerashchenko
 */

#ifndef INC_CVC_12V_SENSE_H_
#define INC_CVC_12V_SENSE_H_

#include <stm32f767xx.h>

// Enum for identifying 12V sense lines
typedef enum {
    PrechargeBtn = 0,
    LVChargeState,
    AIR_12V,
    AIR_Power,
    DCDCConverter,
    CockpitBRB,
    BOT,
} CVC_12V_id_t;

/**
 * @brief Function for reading state of 12V sense line
 * @param id - 12V sense line to check
 * @return 1 if 12V sense line is high, 0 if 12V sense line is low 
 */
uint8_t CVC_12V_Read(CVC_12V_id_t id);

#endif /* INC_CVC_12V_SENSE_H_ */