/**
  ******************************************************************************
  * @file    clt01_38s.h
  * @author  CL
  * @version V1.0.0
  * @date    25-April-2016
  * @brief   PLC_CLT01-38SQ7
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
#ifndef __CLT01_38S_H
#define __CLT01_38S_H

/* Includes ------------------------------------------------------------------*/
#include "current_limiter.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup CLT01_38SQ7
 * @{
 */

/** @defgroup CLT0138_SQ7_Imported_Functions CLT0138_SQ7_Imported_Functions
 * @{
 */
/* Current Limiter IO functions */
extern CURRENT_LIMITER_StatusTypeDef CURRENT_LIMITER_IO_Init(void);
extern void CLT01_38S_IO_Deinit(void);
extern void CLT01_38S_IO_Config(void);
extern uint8_t* CLT01_38S_GetInpData(void);
extern uint8_t CLT01_38S_CommErrorStatus(uint8_t* status);
extern uint8_t CLT01_38S_UvaStatus(uint8_t*);
extern uint8_t CLT01_38S_OtaStatus(uint8_t*);
/**
 * @}
 */

/** @addtogroup CLT01_38S_Exported_Variables CLT01_38S_Exported_Variables
 * @{
 */
/* Current Limter structure */
extern CURRENT_LIMITER_DrvTypeDef CLT0138SQ7Drv;

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
