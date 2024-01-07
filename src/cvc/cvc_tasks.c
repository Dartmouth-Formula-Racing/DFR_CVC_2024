/*
 * cvc_tasks.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Andrei Gerashchenko
 */

#include <cvc_tasks.h>
#include <cvc_plc.h>
#include <cvc_can.h>
#include <cvc_data.h>
#include "cmsis_os.h"
#include "portmacro.h"
#include "task.h"
#include "stm32f7xx_nucleo_144.h"

void Communication(void *argument) {
	for (;;) {
		PLC_CommunicationTask();
        CAN_CommunicationTask();
		taskYIELD();
	}
}

void CommunicationProcessing(void *argument) {
    for (;;) {
        CAN_InterpretTask();
        taskYIELD();
    }
}

void Control(void *argument) {
	for (;;) {
		// PLC_BlinkTask();
		CAN_TestSend();
		taskYIELD();
	}
}