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
    return 0;
}

uint64_t CVC_DataToUint(CVC_data_id_t id) {
    CVC_data_t temp = CVC_GetData(id);
    return (uint64_t)temp.data;
}

int32_t CVC_DataToInt(CVC_data_id_t id) {
    CVC_data_t temp = CVC_GetData(id);
    return (int32_t)temp.data;
}

void CVC_SetData(CVC_data_id_t id, void *data, uint8_t len, type_t type) {
    uint64_t temp = 0;
    if (len == 8) {
        temp = *(uint8_t *)data;
    }
    if (len == 16) {
        temp = *(uint16_t *)data;
    }
    if (len == 32) {
        temp = *(uint32_t *)data;
    }
    if (len == 64) {
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

void CVC_TestSetGetData(void) {
    // Received current reading from inverter, range is -3276.8 to 3276.7 amps encoded as a signed integer
    static int32_t current = -32768;
    float current_float = 0;
    CVC_SetData(CVC_RANDOM, &current, sizeof(current) * 8, INT_10);
    // Read CVC_data[CVC_RANDOM] and convert to float
    current_float = CVC_DataToFloat(CVC_RANDOM);
    printf("Current: %f\n", current_float);
    current += 100;
    if (current > 32767) {
        current = -32768;
    }
}