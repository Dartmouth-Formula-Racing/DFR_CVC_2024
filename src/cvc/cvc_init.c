/*
 * cvc_init.c
 *
 *  Created on: Sep 15, 2023
 *      Author: Andrei Gerashchenko
 */

#include <cvc_data.h>
#include <cvc_init.h>
#include <cvc_can.h>
#include <cvc_plc.h>
#include <stm32f7xx_nucleo_144.h>

// TODO: Put this into a separate file (cvc_gpio.c)?
void CVC_LEDInit(void) {
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_BLUE);
	BSP_LED_Init(LED_RED);
	BSP_LED_Off(LED_GREEN);
	BSP_LED_Off(LED_BLUE);
	BSP_LED_Off(LED_RED);
}

void CVC_Init(void) {
	CVC_DataInit();
	CVC_LEDInit();
	CAN_Configure();
	PLC_Configure();
}