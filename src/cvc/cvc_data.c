#include <cvc_data.h>
#include <stdio.h>

volatile CVC_data_t CVC_data[NUM_VALUES] = {0};

SemaphoreHandle_t CVC_DataMutex;

void CVC_DataInit(void) {
    CVC_DataMutex = xSemaphoreCreateMutex();
}

void CVC_SetData(CVC_data_id_t id, void *data, uint8_t size) {
    uint64_t temp = 0;
    if (size <= sizeof(uint8_t)) {
        temp = *(uint8_t *)data;
    } else if (size <= sizeof(uint16_t)) {
        temp = *(uint16_t *)data;
    } else if (size <= sizeof(uint32_t)) {
        temp = *(uint32_t *)data;
    } else if (size <= sizeof(uint64_t)) {
        temp = *(uint64_t *)data;
    }
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    CVC_data[id].data = temp;
    xSemaphoreGive(CVC_DataMutex);
}

CVC_data_t CVC_GetData(CVC_data_id_t id) {
    CVC_data_t temp;
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    temp = CVC_data[id];
    xSemaphoreGive(CVC_DataMutex);
    return temp;
}