/*
 * cvc_tasks.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Andrei Gerashchenko
 */

#include <cvc_tasks.h>
#include <cvc_can.h>
#include <cvc_data.h>
#include <cvc_relay.h>
#include <cmsis_os.h>
#include <portmacro.h>
#include <task.h>

void Communication(void *argument) {
	for (;;) {
        // CAN_CommunicationTask();
		Relay_SendTask();
	}
}

void CommunicationProcessing(void *argument) {
    for (;;) {
        CAN_InterpretTask();
    }
}

void Control(void *argument) {
	for (;;) {
		Relay_TestTask();
	}
}