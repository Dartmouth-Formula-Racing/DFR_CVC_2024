#include <cvc_data.h>

volatile uint32_t CVC_data[NUM_VALUES] = {0};

SemaphoreHandle_t PLC_InputMutex;

void CVC_DataInit(void) {
    PLC_InputMutex = xSemaphoreCreateMutex();
}