/**
  ******************************************************************************
  * @file    x_nucleo_plc01a1.c
  * @author  CL
  * @version V1.1.0
  * @date    25-April-2016
  * @brief   X-NUCLEO-PLC01A1_application.
  *
  * This file provides firmware functions which are Application Oriented
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
#include "x_nucleo_plc01a1.h"

    
/** @addtogroup BSP     BSP
  * @{
  * @brief 
  */ 


/** @addtogroup X_NUCLEO_PLC01A1     X_NUCLEO_PLC01A1  
  * @brief   
  *@{
  */

    
/** @defgroup X_NUCLEO_PLC01A1_Private_variables   X_NUCLEO_PLC01A1_Private variables
  * @brief  
  * @{
  */    
static RELAY_DrvTypeDef *RelayDrv = NULL;
static CURRENT_LIMITER_DrvTypeDef *CurrentLimiterDrv = NULL;
static SPI_HandleTypeDef SPI_EXPBD_Handle;
TIM_HandleTypeDef TIM_EXPBD_Handle;

/**
  * @}
  */

/* Link function for Relay */
RELAY_StatusTypeDef VNI8200XP_IO_Init(void);
void VNI8200XP_IO_Deinit(void);

/* Link function for current limiter */
CURRENT_LIMITER_StatusTypeDef CLT01_38S_IO_Init(void);
void CLT01_38S_IO_Deinit(void);


RELAY_StatusTypeDef RELAY_IO_Init(void);
void RELAY_IO_Deinit(void);
CURRENT_LIMITER_StatusTypeDef CURRENT_LIMITER_IO_Init(void);
void CURRENT_LIMITER_IO_Deinit(void);
static HAL_StatusTypeDef SPI_EXPBD_Init(void);
static void SPI_EXPBD_MspInit(void);
static HAL_StatusTypeDef TIM_EXPBD_Init(uint16_t, uint8_t);
static void TIM_EXPBD_MspInit(void);

/** @defgroup X_NUCLEO_PLC01A1_Exported_Functions X_NUCLEO_PLC01A1_Exported_Functions
 * @{
 */
/**
  * @brief      Mirrors input data  
  * @param      Input channels data
  * @retval     Output channels data
**/
uint8_t BSP_Signal_Mirror(uint8_t input_state)
{
  return(input_state);
}


/**
  * @brief      Freeze selected output for a given time
  * @param      Input channels state
  * @param      Output channels to be freezed
  * @param      Duration of freeze in miliseconds
  * @retval     Output chanels data
**/

uint8_t BSP_Output_Freeze(uint8_t input_state, uint8_t freeze_channels, uint32_t* msec)
{
  if (*msec != 0)
  {
    return (input_state&freeze_channels);
  }
  else
  {
    return 0x00;
  }
}


/**
  * @brief      Regroup output buffer according to out_Array  
  * @param      Regroup array
  * @retval     Output channels data
**/
uint8_t BSP_Output_Regroup(uint8_t Out_Array)
{
  return(Out_Array);
}


/**
  * @brief      Sum all the inputs at high state 
  * @param      Input channels data
  * @retval     Value corresponding to the sum of inputs at high
**/

uint8_t BSP_Inputs_Sum(uint8_t input_state)
{
  static uint8_t inputChannelsOn = 0;
  static uint8_t count = 0;
  
  for(count = 0; count<8; count++)
  {
    if(input_state & 0x01)
      inputChannelsOn++;
    
    input_state = input_state >> 1;
  }
  
  return inputChannelsOn;
}


/**
  * @brief      Keep the selected channel high 
  * @param      Output to keep high
  * @retval     Output value 
**/
uint8_t BSP_Output_ON(uint8_t Out_Array)
{
  return(Out_Array); 
}


/**
  * @brief      Keep the selected channel low 
  * @param      Output to keep low
  * @retval     Output value   
**/
uint8_t BSP_Output_OFF(uint8_t Out_Array)
{
  return(Out_Array);
}


/**
  * @brief      AND Inputs for selected output channels
  * @param      Input channels state
  * @param      Outputs to be AND with inputs
  * @retval     Result of AND operation   
**/
uint8_t BSP_Inputs_AND(uint8_t input_state, uint8_t Out_Channel)
{
  return (input_state&Out_Channel);
}


/**
  * @brief      OR Inputs for selected output channels
  * @param      Input channels state
  * @param      Outputs to be OR with inputs
  * @retval     Result of OR operation
**/
uint8_t BSP_Inputs_OR(uint8_t input_state, uint8_t Out_Channel)
{
  return (input_state|Out_Channel); 
}


/**
  * @brief        NOT Inputs
  * @param        Input channels state
  * @retval       Result of NOT operation
**/
uint8_t BSP_Inputs_NOT(uint8_t input_state)
{
  return (~input_state); 
}


/**
  * @brief      XOR inputs for selected output channels
  * @param      Input channels state
  * @param      Outputs to be XOR with inputs
  * @retval     Result of XOR operation  
**/
uint8_t BSP_Inputs_XOR(uint8_t input_state, uint8_t Out_Channel)
{
  return (input_state ^ Out_Channel);
}


/**
  * @brief      Reset relay  
  * @param      None
  * @retval     None
**/
void BSP_RELAY_Reset(void)
{
  RelayDrv->ResetRelay();
}

/**
  * @brief      Enable relay outputs 
  * @param      None
  * @retval     None
**/
void BSP_RELAY_EN_Out(void)
{
  RelayDrv->EnableOutputs();
}

uint8_t* BSP_RELAY_SetOutputs(uint8_t* output_data)
{
  return(RelayDrv->SetOutputs(*output_data));   
}

uint8_t* BSP_CURRENT_LIMITER_Read(void)
{
  return(CurrentLimiterDrv->InputData()); 
}


/**
  * @brief      Initialization of Relay
  * @param      None
  * @retval     None
**/
RELAY_StatusTypeDef BSP_Relay_Init(void)
{
  RELAY_InitTypeDef InitStructure;
  
  RelayDrv = &VNI8200XPDrv;
  InitStructure.outputs_state = ALL_OUTPUTS_OFF;
  
  if (RelayDrv->Init == NULL)
  {
    RelayDrv = NULL;
    return RELAY_ERROR;
  }
  
  if (RelayDrv->Init(&InitStructure) != RELAY_OK)
  {
    RelayDrv = NULL;
    return RELAY_ERROR;
  }
  
  return RELAY_OK;
  
}

/**
  * @brief      Initialization Timers for output cycling mode
  * @param      None
  * @retval     None
**/
void BSP_OuputCycling_Init(uint16_t freq, uint8_t duty)
{
  TIM_EXPBD_Init(freq,duty);
}


/**
  * @brief      Inditialization of input current limiter
  * @param      None
  * @retval     Current Limiter Status
**/
CURRENT_LIMITER_StatusTypeDef BSP_CurrentLimiter_Init(void)
{
  CURRENT_LIMITER_InitTypeDef InitStructure;
  
  CurrentLimiterDrv = &CLT0138SQ7Drv;  

  if ( CurrentLimiterDrv->Init == NULL)
  {
    CurrentLimiterDrv = NULL;
    return CURRENT_LIMITER_ERROR;
  }
  
  if (CurrentLimiterDrv->Init(&InitStructure) != CURRENT_LIMITER_OK)
  {
    CurrentLimiterDrv = NULL;
    return CURRENT_LIMITER_ERROR;
  }  
  
   return CURRENT_LIMITER_OK;
}
 

/**
  * @brief      Get status of relay
  * @param      Pointer to incoming status from relay
  * @retval     RELAY_OK if status is ok otherwise RELAY_ERROR
**/
RELAY_StatusTypeDef BSP_GetRelayStatus(uint8_t* status)
{
  if (RelayDrv->GetFaultStatus(status))
    return RELAY_ERROR;
  if (RelayDrv->GetFbOkStatus(status))
    return RELAY_ERROR;
  if (RelayDrv->GetTempWarningStatus(status))
    return RELAY_ERROR;
  if (RelayDrv->GetPcFailStatus(status))
    return RELAY_ERROR;
  if (RelayDrv->GetPowerGoodStatus(status))
    return RELAY_ERROR;
  if (RelayDrv->GetCommErrorStatus(status))
    return RELAY_ERROR;
  
  return RELAY_OK;
}

/**
  * @brief      Get status of current limiter
  * @param      Pointer to incoming status from current limiter
  * @retval     CURRENT_LIMITER_OK if status is ok otherwise CURRENT_LIMITER_ERROR
**/
CURRENT_LIMITER_StatusTypeDef BSP_GetCurrentLimiterStatus(uint8_t* status)
{
  if (CurrentLimiterDrv->GetOverTempAlarmStatus(status))
    return CURRENT_LIMITER_ERROR;
  if (CurrentLimiterDrv->GetUnderVoltAlarmStatus(status))
    return CURRENT_LIMITER_ERROR;
  if (CurrentLimiterDrv->GetCommErrorStatus(status))
    return CURRENT_LIMITER_ERROR;
  
  return CURRENT_LIMITER_OK;
}


/********************************* LINK RELAY *********************************/
/**
 * @brief  Configures VNI8200XP 
 * @param  None
 * @retval RELAY_OK in case of success, an error code otherwise
 */
RELAY_StatusTypeDef VNI8200XP_IO_Init(void)
{
  return RELAY_IO_Init();
}

/**
 * @brief  Reset VNI8200XP
 * @param  None
 * @retval None
 */
void VNI8200XP_IO_Deinit(void)
{
 /* To be implemented */  
}

/**
 * @brief  Configures VNI8200XP gpios for NUCLEO boards
 * @param  None
 * @retval None
 */
void VNI8200XP_IO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  VNI8200XP_CS_CLK_ENABLE();
  VNI8200XP_OUT_EN_CLK_ENABLE();

  GPIO_InitStruct.Pin = VNI8200XP_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VNI8200XP_CS_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = VNI8200XP_OUT_EN_PIN;
  HAL_GPIO_Init(VNI8200XP_OUT_EN_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Set reset GPIO high to disable VNI8200XP
 * @param  None
 * @retval None
 */
void VNI8200XP_ResetIt(void)
{
  HAL_GPIO_WritePin(VNI8200XP_RESET_PORT,VNI8200XP_RESET_PIN,GPIO_PIN_SET);  
}


/**
 * @brief  Enable outputs of VNI8200XP
 * @param  None
 * @retval None
 */
void VNI8200XP_EnOut(void)
{
  HAL_GPIO_WritePin(VNI8200XP_ENABLE_PORT,VNI8200XP_ENABLE_PIN,GPIO_PIN_SET);  
}


/**
 * @brief  Calculate output parity bits and send data
 * @param  None
 * @retval None
 */
uint8_t* VNI8200XP_SendOutputData(uint8_t udata)
{
  static uint8_t P0, P1, P2, nP0, parityData, txData[2], rxData[2];
  
  P0 = udata^(udata>>1);
  P0 = P0 ^(P0 >> 2);
  P0 = P0 ^(P0 >> 4);
  P0 = P0 & 0x01;
  
  P1 = udata^(udata>>2);
  P1 = P1 ^ (P1 >> 4);
  
  P2 = P1 & 0x01;
  
  P1 = P1 & 0x02;
  P1 = P1 >> 1;
  
  nP0 = (~P0) & 0x01;
  
  parityData = (P2<<3)|(P1<<2)|(P0<<1)|nP0;
  
  *txData = parityData; 
  *(txData+1) = udata;
  
  // why is this hardcoded
    HAL_GPIO_WritePin(VNI8200XP_CS_PORT, VNI8200XP_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPI_EXPBD_Handle,txData,rxData,1,0x100);
    HAL_GPIO_WritePin(VNI8200XP_CS_PORT, VNI8200XP_CS_PIN, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//  HAL_SPI_TransmitReceive(&SPI_EXPBD_Handle,txData,rxData,1,0x100);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  
  return rxData;
}


/**
 * @brief  Calculate output parity bits and send data
 * @param  None
 * @retval None
 */
uint8_t VNI8200XP_FaultStatus(uint8_t* status)
{
  return (*(status+1) != 0);
}


/**
 * @brief  Returns status of feedback
 * @param  VNI8200XP incoming status
 * @retval Status of feedback
 */
uint8_t VNI8200XP_FbOkStatus(uint8_t* status)
{
  return (((*status)&0x80) == 0);
}

/**
 * @brief  Returns status of Temperature warning
 * @param  VNI8200XP incoming status
 * @retval Status of Temperature Warning
 */
uint8_t VNI8200XP_TempWarningStatus(uint8_t* status)
{
  return (((*status)&0x40) != 0);
}


/**
 * @brief  Returns status of Outgoing Parity check
 * @param  VNI8200XP incoming status
 * @retval Status of parity check
 */
uint8_t VNI8200XP_PcFailStatus(uint8_t* status)
{
  return (((*status)&0x20) != 0);
}


/**
 * @brief  Returns status of power to VNI8200XP
 * @param  VNI8200XP incoming status
 * @retval Status of power good
 */
uint8_t VNI8200XP_PowerGoodStatus(uint8_t* status)
{
  return (((*status)&0x10) != 0);
}


/**
 * @brief  Returns status of incoming parity 
 * @param  VNI8200XP incoming status
 * @retval Result of incoming parity check
 */
uint8_t VNI8200XP_CommErrorStatus(uint8_t* status)
{
  static uint16_t P0, P1, P2, nP0, tempStatusData, parityCalcData;
  
  tempStatusData = ((*status))+((*(status+1))*256);
  tempStatusData = tempStatusData & 0xFFF0;
  
  P0 = tempStatusData^(tempStatusData>>1);
  P0 = P0 ^(P0 >> 2);
  P0 = P0 ^(P0 >> 4);
  P0 = P0 & 0x0100;
  P0 = P0 >> 8;
    
  P1 = (tempStatusData)^(tempStatusData>>2);
  P1 = P1 ^ (P1 >> 4);
  P1 = P1 ^ (P1 >> 8);
  
  P2 = P1 & 0x01;
  
  P1 = P1 & 0x02;
  P1 = P1 >> 1;
  
  nP0 = (~P0) & 0x01;
  
  parityCalcData = (P2<<3)|(P1<<2)|(P0<<1)|nP0;

  return (((uint8_t)parityCalcData) != ((*status)&0x0F));
}


/**************************** LINK CURRENT_LIMITER ****************************/
/**
 * @brief  Configures CLT01-38SQ7 
 * @param  None
 * @retval CURRENT_LIMITER_OK in case of success, an error code otherwise
 */
CURRENT_LIMITER_StatusTypeDef CLT01_38S_IO_Init(void)
{
  return CURRENT_LIMITER_IO_Init();  
}


/**
 * @brief  Reset CLT01-38SQ7 
 * @param  None
 * @retval None
 */
void CLT01_38S_IO_Deinit(void)
{
 /* To be implemented */  
}


/**
 * @brief  Configures CLT01-38SQ7 gpios
 * @param  None
 * @retval None
 */
void CLT01_38S_IO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  CLT01_38S_CS_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = CLT01_38S_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW ;
  HAL_GPIO_Init(CLT01_38S_CS_PORT, &GPIO_InitStruct);
}


/**
 * @brief  Read input channels state 
 * @param  None
 * @retval Pointer to input data from CLT01-38SQ7
 */
uint8_t* CLT01_38S_GetInpData(void)
{
  static uint8_t txData[2],rxData[2];
  

// Why was this hardcoded???
  HAL_GPIO_WritePin(CLT01_38S_CS_PORT, CLT01_38S_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&SPI_EXPBD_Handle, txData, rxData, 1, 0x100);
  HAL_GPIO_WritePin(CLT01_38S_CS_PORT, CLT01_38S_CS_PIN, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//  HAL_SPI_TransmitReceive(&SPI_EXPBD_Handle, txData, rxData, 1, 0x100);
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  return rxData;
}


/**
 * @brief  Returns status of incoming parity 
 * @param  Pointer to CLT01-38SQ7 incoming status
 * @retval Result of incoming parity check status
 */
uint8_t CLT01_38S_CommErrorStatus(uint8_t* status)
{
  static uint8_t PC1, PC2, PC3, PC4, parityCalcData;
  
  PC1 = (*(status+1))^((*(status+1))>>1);
  PC1 = PC1 ^ (PC1 >> 2);
  
  PC2 = PC1 & 0x10;
  PC2 = PC2 >> 4;
  
  PC3 = PC1 & 0x01;

  PC4 = PC1 & 0x04;
  PC4 = PC4 >> 2;

  PC1 = PC1 ^ (PC1 >> 4);
  PC1 = PC1 & 0x01;
  
  PC1 = ~PC1; PC1 = PC1 & 0x01;
  PC2 = ~PC2; PC2 = PC2 & 0x01;
  PC3 = ~PC3; PC3 = PC3 & 0x01;
  PC4 = ~PC4; PC4 = PC4 & 0x01;
  
  parityCalcData = (PC1<<5)|(PC2<<4)|(PC3<<3)|(PC4<<2);

  return (((uint8_t)parityCalcData) != ((*status)&0x3C));
}


/**
 * @brief  Get status of under voltage alarm 
 * @param  Pointer to CLT01-38SQ7 incoming status
 * @retval Under voltage alarm status
 */
uint8_t CLT01_38S_UvaStatus(uint8_t* status)
{
  return (((*status)&0x80) == 0);
}


/**
 * @brief  Get status of over temperature alarm 
 * @param  Pointer to CLT01-38SQ7 incoming status
 * @retval Over Temperature alarm status
 */
uint8_t CLT01_38S_OtaStatus(uint8_t* status)
{
  return (((*status)&0x40) == 0);
}

/**
  * @} 
  */

/** @defgroup X_NUCLEO_PLC01A1_Private_Functions X_NUCLEO_PLC01A1_Private_Functions
 * @{
 */
/**
 * @brief  Configures output relay
 * @param  None
 * @retval RELAY_OK in case of success, an error code otherwise
 */
RELAY_StatusTypeDef RELAY_IO_Init(void)
{
  if (SPI_EXPBD_Init() != HAL_OK)
  {
    return RELAY_ERROR;
  }
  
  return RELAY_OK;
}


/**
 * @brief  Deinitialize Relay 
 * @param  None
 * @retval None
 */
void RELAY_IO_Deinit(void)
{
 /* To be implemented */  
}


/**
 * @brief  Configures input Current Limiter
 * @param  None
 * @retval CURRENT_LIMITER_OK in case of success, an error code otherwise
 */
CURRENT_LIMITER_StatusTypeDef CURRENT_LIMITER_IO_Init(void)
{
  if (SPI_EXPBD_Init() != HAL_OK)
  {
    return CURRENT_LIMITER_ERROR;
  }
  
  return CURRENT_LIMITER_OK;
}


/**
 * @brief  Deinitialize Current Limiter 
 * @param  None
 * @retval None
 */
void CURRENT_LIMITER_IO_Deinit(void)
{
 /* To be implemented */ 
}


/**
 * @brief  Configures SPI interface
 * @param  None
 * @retval HAL status
 */
static HAL_StatusTypeDef SPI_EXPBD_Init(void)
{
  HAL_StatusTypeDef ret_val;
  
  if (HAL_SPI_GetState(&SPI_EXPBD_Handle) == HAL_SPI_STATE_RESET)
  {
    SPI_EXPBD_Handle.Instance = SPI1;
    SPI_EXPBD_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI_EXPBD_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI_EXPBD_Handle.Init.DataSize = SPI_DATASIZE_16BIT;
    SPI_EXPBD_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_EXPBD_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_EXPBD_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_EXPBD_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    SPI_EXPBD_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_EXPBD_Handle.Init.TIMode = SPI_TIMODE_DISABLED;
    SPI_EXPBD_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
#ifdef STM32F030x8
    SPI_EXPBD_Handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
#elif (STM32F401xE|STM32F334x8|STM32F103xB)
    SPI_EXPBD_Handle.Init.CRCPolynomial = 10;
#endif

    SPI_EXPBD_MspInit();
    ret_val = HAL_SPI_Init(&SPI_EXPBD_Handle);
    if (ret_val != HAL_OK)      return ret_val;
  }
  
  if (HAL_SPI_GetState(&SPI_EXPBD_Handle) == HAL_SPI_STATE_READY)
  {
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
 * @brief  SPI MSP Initialization
 * @param  None
 * @retval None
 */
static void SPI_EXPBD_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /* Peripheral clock enable */
  X_NUCLEO_SPI_EXPBD_CLK_ENABLE();

  GPIO_InitStruct.Pin = X_NUCLEO_SPI_EXPBD_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW ;
#if defined (STM32F030x8) | defined (STM32F334x8) | defined (STM32F401xE)
  GPIO_InitStruct.Alternate = X_NUCLEO_SPI_MISO_ALTERNATE;
#endif
  HAL_GPIO_Init(X_NUCLEO_SPI_EXPBD_MISO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = X_NUCLEO_SPI_EXPBD_MOSI_PIN;
#ifdef STM32F030x8
  GPIO_InitStruct.Alternate = X_NUCLEO_SPI_MOSI_ALTERNATE;
#endif
  HAL_GPIO_Init(X_NUCLEO_SPI_EXPBD_MOSI_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = X_NUCLEO_SPI_EXPBD_SCK_PIN;
#ifdef STM32F030x8
  GPIO_InitStruct.Alternate = X_NUCLEO_SPI_SCK_ALTERNATE;
#endif
  HAL_GPIO_Init(X_NUCLEO_SPI_EXPBD_SCK_PORT, &GPIO_InitStruct);

#ifdef STM32F103xB
  __HAL_AFIO_REMAP_SPI1_DISABLE();
#endif
}


/**
 * @brief  Configures TIM3
 * @param  None
 * @retval HAL status
 */
static HAL_StatusTypeDef TIM_EXPBD_Init(uint16_t freq, uint8_t duty)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  uint16_t timPeriod = 0;
  uint16_t timPulse = 0;
  
  timPeriod = ((48000000/(X_NUCLEO_TIM_PRESCALER+1))/freq)-1;
  timPulse = ((((48000000/(X_NUCLEO_TIM_PRESCALER+1))/freq)-2)*duty)/100;
  
  TIM_EXPBD_Handle.Instance = TIM3;
  TIM_EXPBD_Handle.Init.Prescaler = X_NUCLEO_TIM_PRESCALER;
  TIM_EXPBD_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_EXPBD_Handle.Init.Period = timPeriod;
  TIM_EXPBD_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
  TIM_EXPBD_MspInit();
  
  HAL_TIM_Base_Init(&TIM_EXPBD_Handle);
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&TIM_EXPBD_Handle, &sClockSourceConfig);

  HAL_TIM_OC_Init(&TIM_EXPBD_Handle);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TIM_EXPBD_Handle, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = timPulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&TIM_EXPBD_Handle, &sConfigOC, TIM_CHANNEL_1);  
    
  if (HAL_TIM_OC_Start_IT(&TIM_EXPBD_Handle,TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_Base_Start_IT(&TIM_EXPBD_Handle) != HAL_OK)
  {
    return HAL_ERROR;
  } 
  
  return HAL_OK;
}

/**
 * @brief  TIM MSP Initialization
 * @param  None
 * @retval None
 */
static void TIM_EXPBD_MspInit(void)
{
   __TIM3_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
