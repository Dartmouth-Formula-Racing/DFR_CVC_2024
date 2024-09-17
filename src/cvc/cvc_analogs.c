#include <stm32f7xx_hal_adc.h>
#include <cvc_data.h>
#include <main.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void Analogs_Configure() {
    uint32_t adc_value = 0;
    CVC_SetData(LV_VOLTAGE, &adc_value, sizeof(adc_value));
    CVC_SetData(CVC_THROTTLE_ADC, &adc_value, sizeof(adc_value));
}

void Analogs_ReadVoltage() {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        CVC_SetData(LV_VOLTAGE, &adc_value, sizeof(adc_value));
    }
}

void Analogs_ReadThrottle() {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc2);
        CVC_SetData(CVC_THROTTLE_ADC, &adc_value, sizeof(adc_value));
    }
}
