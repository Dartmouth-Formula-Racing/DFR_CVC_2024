/**
  ******************************************************************************
  * @file    vni8200xp.c
  * @author  CL
  * @version V1.1.0
  * @date    25-April-2016
  * @brief   This file provides a set of functions needed to manage VNI8200XP
  * This file provides firmware functions for how  to manage I/O from VNI8200XP 
  ==============================================================================    
 
           
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

/* Includes ------------------------------------------------------------------*/
#include "vni8200xp.h"
    
/** @addtogroup Drivers
  * @{
  * @brief Demo Driver Layer
  */    
  
/** @addtogroup BSP
  * @{
  */    
    
/** @addtogroup Components
  * @{
  */

/** @defgroup VNI8200XP
  * @{ 
  */   
static RELAY_StatusTypeDef VNI8200XP_Init(RELAY_InitTypeDef *VNI8200XP_Init);  
static void VNI8200XP_Deinit(void);
static void VNI8200XP_Reset(void);
static void VNI8200XP_EnableOutputs(void);
static uint8_t* VNI8200XP_SetOutput(uint8_t);
static uint8_t VNI8200XP_GetChFaultStatus(uint8_t*);
static uint8_t VNI8200XP_GetFbOkStatus(uint8_t*);
static uint8_t VNI8200XP_GetTempWarningStatus(uint8_t*);
static uint8_t VNI8200XP_GetPcFailStatus(uint8_t*);
static uint8_t VNI8200XP_GetPowerGoodStatus(uint8_t*);
static uint8_t VNI8200XP_GetCommErrorStatus(uint8_t*);


/** @defgroup VNI8200XP_Private_variables     VNI8200XP  Private variables
  * @{
  */    

RELAY_DrvTypeDef VNI8200XPDrv =
{
  VNI8200XP_Init,
  VNI8200XP_Deinit,
  VNI8200XP_Reset,
  VNI8200XP_EnableOutputs,
  VNI8200XP_SetOutput,
  VNI8200XP_GetChFaultStatus,
  VNI8200XP_GetFbOkStatus,
  VNI8200XP_GetTempWarningStatus,
  VNI8200XP_GetPcFailStatus,
  VNI8200XP_GetPowerGoodStatus,
  VNI8200XP_GetCommErrorStatus
};

/**
  * @}
  */

/** @defgroup VNI8200XP_Private_Functions       VNI8200XP_Private_Functions
  * @{
  */

/**
 * @brief  Set VNI8200XP Initialization
 * @param  VNI8200XP_Init configuration setting for the VNI8200XP
 * @retval RELAY_OK in case of success, an error code otherwise
 */
static RELAY_StatusTypeDef VNI8200XP_Init(RELAY_InitTypeDef *VNI8200XP_Init)
{
  /* Configure the low level interface ---------------------------------------*/
  if(VNI8200XP_IO_Init()!= RELAY_OK)
  {
    return RELAY_ERROR;
  }

  VNI8200XP_IO_Config();
  
  return RELAY_OK;  
}  


/**
 * @brief  Deinitialize VNI8200XP interface
 * @param  None
 * @retval None
 */
static void VNI8200XP_Deinit(void)
{
  VNI8200XP_IO_Deinit();
}


/**
  * @brief      Reset VNI8200XP
  * @param      None 
  * @retval     None           
**/
static void VNI8200XP_Reset(void)
{
  VNI8200XP_ResetIt();
}

/**
  * @brief      Enable outputs of VNI8200XP
  * @param      None 
  * @retval     None           
**/
static void VNI8200XP_EnableOutputs(void)
{
  VNI8200XP_EnOut();
}

/**
  * @brief      Set outputs of VNI8200XP
  * @param      Output Channels data
  * @retval     None
**/
static uint8_t* VNI8200XP_SetOutput(uint8_t data)
{
  return(VNI8200XP_SendOutputData(data));
}

/**
  * @brief      Get output fault status
  * @param      Pointer to incoming status
  * @retval     Output channel fault status      
**/
static uint8_t VNI8200XP_GetChFaultStatus(uint8_t* status)
{
  return VNI8200XP_FaultStatus(status);
}

/**
  * @brief      Get status of DC-DC feedback
  * @param      Pointer to incoming status
  * @retval     Feedback ok status         
**/
static uint8_t VNI8200XP_GetFbOkStatus(uint8_t* status)
{
  return VNI8200XP_FbOkStatus(status);
}

/**
  * @brief      GetVNI8200XP temperature warning status
  * @param      Pointer to incoming status
  * @retval     Temperature warning status, 1 if over temperature
**/
static uint8_t VNI8200XP_GetTempWarningStatus(uint8_t* status)
{  
  return VNI8200XP_TempWarningStatus(status);
}


/**
  * @brief      Check VNI8200XP parity check status
  * @param      Pointer to incoming status
  * @retval     Parity check flag
**/
static uint8_t VNI8200XP_GetPcFailStatus(uint8_t* status)
{  
  return VNI8200XP_PcFailStatus(status);
}


/**
  * @brief      Get VNI8200XP power good status
  * @param      Pointer to incoming status
  * @retval     Power good bit, 1 in case of power good
**/
static uint8_t VNI8200XP_GetPowerGoodStatus(uint8_t* status)
{
  return VNI8200XP_PowerGoodStatus(status);
}


/**
  * @brief      Check communication error
  * @param      Pointer to incoming status
  * @retval     Communication error
**/
static uint8_t VNI8200XP_GetCommErrorStatus(uint8_t* status)
{
  return VNI8200XP_CommErrorStatus(status);
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
