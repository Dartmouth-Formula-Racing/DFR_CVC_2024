#include <cvc_data.h>
#include <stdio.h>

volatile CVC_data_t CVC_data[NUM_VALUES] = {0};

SemaphoreHandle_t CVC_DataMutex;

void CVC_DataInit(void) {
    CVC_DataMutex = xSemaphoreCreateMutex();
}

float CVC_DataToFloat(CVC_data_id_t id) {
    CVC_data_t temp = CVC_GetData(id);

    if (temp.type == INT_10) {
        return ((float)((int32_t)temp.data)) / 10;
    }
    if (temp.type == INT_100) {
        return ((float)((int32_t)temp.data)) / 100;
    }
    if (temp.type == INT_1000) {
        return ((float)((int32_t)temp.data)) / 1000;
    }
    if (temp.type == FLOAT) {
        return (float)temp.data;
    }
    return (float)0;
}

uint64_t CVC_DataToUint(CVC_data_id_t id) {
    CVC_data_t temp = CVC_GetData(id);
    return (uint64_t)temp.data;
}

int32_t CVC_DataToInt(CVC_data_id_t id) {
    CVC_data_t temp = CVC_GetData(id);
    return (int32_t)temp.data;
}

void CVC_SetData(CVC_data_id_t id, void *data, uint8_t size, type_t type) {
    uint64_t temp = 0;
    if (size <= 8) {
        temp = *(uint8_t *)data;
    }
    if (size <= 16) {
        temp = *(uint16_t *)data;
    }
    if (size <= 32) {
        temp = *(uint32_t *)data;
    }
    if (size <= 64) {
        temp = *(uint64_t *)data;
    }
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    CVC_data[id].data = temp;
    CVC_data[id].type = type;
    xSemaphoreGive(CVC_DataMutex);
}

CVC_data_t CVC_GetData(CVC_data_id_t id) {
    CVC_data_t temp;
    xSemaphoreTake(CVC_DataMutex, portMAX_DELAY);
    temp = CVC_data[id];
    xSemaphoreGive(CVC_DataMutex);
    return temp;
}