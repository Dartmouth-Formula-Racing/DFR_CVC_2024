/**
  ******************************************************************************
  * @file    vni8200xp.h
  * @author  CL
  * @version V1.0.0
  * @date    25_February-2016
  * @brief   PLC_VNI8200XP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VNI8200XP_H
#define __VNI8200XP_H

/* Includes ------------------------------------------------------------------*/
#include "relay.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup VNI8200XP
 * @{
 */

/** @defgroup VNI8200XP_Imported_Functions VNI8200XP_Imported_Functions
 * @{
 */

/* Relay IO functions */
extern RELAY_StatusTypeDef VNI8200XP_IO_Init(void);
extern void VNI8200XP_IO_Deinit(void);
extern void VNI8200XP_IO_Config(void);
extern void VNI8200XP_ResetIt(void);
extern void VNI8200XP_EnOut(void);
extern uint8_t* VNI8200XP_SendOutputData(uint8_t);
extern uint8_t VNI8200XP_FaultStatus(uint8_t* status);
extern uint8_t VNI8200XP_FbOkStatus(uint8_t* status);
extern uint8_t VNI8200XP_TempWarningStatus(uint8_t* status);
extern uint8_t VNI8200XP_PcFailStatus(uint8_t* status);
extern uint8_t VNI8200XP_PowerGoodStatus(uint8_t* status);
extern uint8_t VNI8200XP_CommErrorStatus(uint8_t* status);
/**
 * @}
 */

/** @addtogroup VNI8200XP_Exported_Variables VNI8200XP_Exported_Variables
 * @{
 */
/* Relay structure */
extern RELAY_DrvTypeDef VNI8200XPDrv;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
