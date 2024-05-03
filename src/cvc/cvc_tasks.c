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
#include <cvc_analogs.h>
#include <cvc_control.h>
#include "cmsis_os.h"
#include "portmacro.h"
#include "task.h"

void Communication(void *argument) {
	static uint32_t lastWakeTime = 0;
	for (;;) {
		volatile uint32_t currentWakeTime = lastWakeTime;
		currentWakeTime++;
		lastWakeTime = currentWakeTime;
        CAN_CommunicationTask();
		Relay_SendTask();
		CAN_BroadcastTask();
		Control_TorqueCommand();
	}
}

void Data(void *argument) {
	static uint32_t lastWakeTime = 0;
    for (;;) {
		volatile uint32_t currentWakeTime = lastWakeTime;
		currentWakeTime++;
		lastWakeTime = currentWakeTime;
        CAN_InterpretTask();
		Analogs_ReadThrottle();
		Analogs_ReadVoltage();
    }
}

void Control(void *argument) {
	static uint32_t lastWakeTime = 0;
	for (;;) {
		volatile uint32_t currentWakeTime = lastWakeTime;
		currentWakeTime++;
		lastWakeTime = currentWakeTime;
		Control_ControlTask();
		// Control_ErrorAlertTask();
	}
}