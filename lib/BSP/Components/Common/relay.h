/**
 ******************************************************************************
 * @file    realy.h
 * @author  CL
 * @version V1.1.0
 * @date    25_April-2016
 * @brief   This header file contains the functions prototypes for the
 *          plc driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup RELAY
  * @{
  */

/** @defgroup RELAY_Exported_Types
  * @{
  */

/**
 * @brief  RELAY init structure definition
 */
typedef struct
{
  uint8_t outputs_state;
} RELAY_InitTypeDef;    
    
/**
 * @brief  RELAY status enumerator definition
 */
typedef enum
{
  RELAY_OK = 0,
  RELAY_ERROR = 1,
  RELAY_TIMEOUT = 2,
  RELAY_NOT_IMPLEMENTED = 3
} RELAY_StatusTypeDef;
    
/**
 * @brief  SSRELAY driver structure definition
 */
  
typedef struct
{
  RELAY_StatusTypeDef (*Init)(RELAY_InitTypeDef*);
  void (*Deinit)(void);
  void (*ResetRelay)(void);
  void (*EnableOutputs)(void);
  uint8_t* (*SetOutputs)(uint8_t);
  uint8_t (*GetFaultStatus)(uint8_t*);
  uint8_t (*GetFbOkStatus)(uint8_t*);
  uint8_t (*GetTempWarningStatus)(uint8_t*);
  uint8_t (*GetPcFailStatus)(uint8_t*);
  uint8_t (*GetPowerGoodStatus)(uint8_t*);
  uint8_t (*GetCommErrorStatus)(uint8_t*);  
} RELAY_DrvTypeDef;


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

#ifdef	 __cplusplus
}
#endif

#endif /* __RELAY_H */
/**
  * @} // end   plc Exported Function
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
