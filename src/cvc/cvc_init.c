/*
 * cvc_init.c
 *
 *  Created on: Sep 15, 2023
 *      Author: Andrei Gerashchenko
 */

#include <cvc_data.h>
#include <cvc_init.h>
#include <cvc_can.h>

void CVC_Init(void) {
	CVC_DataInit();
	CAN_Configure();
}