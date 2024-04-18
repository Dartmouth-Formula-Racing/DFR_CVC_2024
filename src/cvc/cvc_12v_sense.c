/*
 * cvc_12v_sense.c
 *
 *  Created on: Apr 7, 2024
 *      Author: Andrei Gerashchenko
 */

#include <cvc_12v_sense.h>
#include <stm32f7xx_hal_gpio.h>
#include <main.h>

uint8_t CVC_12V_Read(CVC_12V_id_t id) {
    switch (id) {
        case PrechargeBtn:
            return !HAL_GPIO_ReadPin(SENSE_12V_7_GPIO_Port, SENSE_12V_7_Pin);
        case LVChargeState:
            return !HAL_GPIO_ReadPin(SENSE_12V_6_GPIO_Port, SENSE_12V_6_Pin);
        case AIR_12V:
            return !HAL_GPIO_ReadPin(SENSE_12V_5_GPIO_Port, SENSE_12V_5_Pin);
        case AIR_Power:
            return !HAL_GPIO_ReadPin(SENSE_12V_4_GPIO_Port, SENSE_12V_4_Pin);
        case DCDCConverter:
            return !HAL_GPIO_ReadPin(SENSE_12V_3_GPIO_Port, SENSE_12V_3_Pin);
        case CockpitBRB:
            return !HAL_GPIO_ReadPin(SENSE_12V_2_GPIO_Port, SENSE_12V_2_Pin);
        case BOT:
            return !HAL_GPIO_ReadPin(SENSE_12V_1_GPIO_Port, SENSE_12V_1_Pin);
        default:
            return 0;
    }
}