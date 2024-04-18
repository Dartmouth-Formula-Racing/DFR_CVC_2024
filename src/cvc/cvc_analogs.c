/*
 * cvc_analogs.c
 *
 *  Created on: Apr 16, 2024
 *      Author: Andrei Gerashchenko
 */

#include <cvc_analogs.h>
#include <stm32f767xx.h>
#include <stm32f7xx_hal_adc.h>
#include <portmacro.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

uint32_t CVC_Analog_Read(CVC_analog_id_t id) {
    switch (id) {
        case LVVoltage:
            HAL_ADC_PollForConversion(&hadc1, portMAX_DELAY);
            return HAL_ADC_GetValue(&hadc1);
        case Throttle1:
            HAL_ADC_PollForConversion(&hadc2, portMAX_DELAY);
            return HAL_ADC_GetValue(&hadc2);
        default:
            return 0;
    }
}