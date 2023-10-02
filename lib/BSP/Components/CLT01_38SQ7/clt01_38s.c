/**
  ******************************************************************************
  * @file    clt01_38s.c
  * @author  CL
  * @version V1.1.0
  * @date    25-April-2016
  * @brief   This file provides a set of function to manage CLT01-38SQ7
  ==============================================================================    
 
           
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

/* Includes ------------------------------------------------------------------*/
#include "clt01_38s.h"
    
/** @addtogroup Drivers     Drivers
  * @{
  * @brief Demo Driver Layer
  */    
  
/** @addtogroup BSP     BSP
  * @{
  */    
    
/** @addtogroup Components     Components
  * @{
  */

/** @defgroup CLT01_38S    CLT01_38S
  * @{ 
  */
static CURRENT_LIMITER_StatusTypeDef CLT01_38S_Init(CURRENT_LIMITER_InitTypeDef *CLT01_38S_Init);
static void CLT01_38S_Deinit(void);
static uint8_t CLT01_38S_GetOtaStatus(uint8_t*);
static uint8_t CLT01_38S_GetCommErrorStatus(uint8_t*);
static uint8_t CLT01_38S_GetUvaStatus(uint8_t*);
static uint8_t* CLT01_38S_GetData(void);


/** @defgroup CLT01_38S_Private_variables     CLT01 38S Private variables
  * @{
  * @brief Digital Input Private variables
  */    

CURRENT_LIMITER_DrvTypeDef CLT0138SQ7Drv =
{
  CLT01_38S_Init,
  CLT01_38S_Deinit,
  CLT01_38S_GetData,
  CLT01_38S_GetOtaStatus,
  CLT01_38S_GetUvaStatus,
  CLT01_38S_GetCommErrorStatus,
};
/**
  * @}
  */



/** @defgroup CLT01_38S_Private_Functions     CLT01 38S Private Functions
  *  @{
    * @brief Digital Input exported Function 
  */  

/**
 * @brief  Set CLT01 Initialization
 * @param  CLT01_Init configuration setting for the CLT01
 * @retval CURRENT_LIMITER_OK in case of success, an error code otherwise
 */
static CURRENT_LIMITER_StatusTypeDef CLT01_38S_Init(CURRENT_LIMITER_InitTypeDef *CLT01_38S_Init)
{
  /* Configure the low level interface ---------------------------------------*/
  if(CURRENT_LIMITER_IO_Init() != CURRENT_LIMITER_OK)
  {
    return CURRENT_LIMITER_ERROR;
  }
  
  CLT01_38S_IO_Config();

  return CURRENT_LIMITER_OK;  
  
}

/**
 * @brief  Deinitialize CLT01_38S IO
 * @param  None
 * @retval None
 */
static void CLT01_38S_Deinit(void)
{
 /* To be implemented */ 
}


/**
  * @brief      CLT Over_Temprature_Alarm
  * @param      None
  * @retval     Overtemperature bit, 1 in case of alarm
**/
static uint8_t CLT01_38S_GetOtaStatus(uint8_t* status)
{
  return (CLT01_38S_OtaStatus(status));
}


/**
  * @brief      CLT Parity_Check_bits
  * @param      None
  * @retval     Parity bits for diagnosing inconsistency in data transmission 
**/
static uint8_t CLT01_38S_GetCommErrorStatus(uint8_t* status)
{
  return (CLT01_38S_CommErrorStatus(status));
}


/**
  * @brief      CLT Under_Voltage_Alarm_bit
  * @param      None
  * @retval     Under voltage alarm bit, 1 in case of alarm
**/
static uint8_t CLT01_38S_GetUvaStatus(uint8_t* status)
{
  return (CLT01_38S_UvaStatus(status));
}


/**
  * @brief      INPUT status
  * @param      None
  * @retval     Channels status corresponding to 8 inputs           
**/
static uint8_t* CLT01_38S_GetData(void)
{
  return (CLT01_38S_GetInpData());
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
