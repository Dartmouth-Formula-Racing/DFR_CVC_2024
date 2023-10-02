/**
  ******************************************************************************
  * @file    X_NUCLEO_PLC01A1.h
  * @author  CL
  * @version V1.1.0
  * @date    25-April-2016
  * @brief   X-NUCLEO-PLC01A1_Application
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
/** @defgroup X-NUCLEO-PLC01A1_Exported_variables     X-NUCLEO-PLC01A1 Exported variables
  * @{
  * @brief Exported variables 
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_PLC01A1_H
#define __X_NUCLEO_PLC01A1_H
    
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal_def.h"

#include "relay.h"
#include "current_limiter.h"

#include "vni8200xp.h"
#include "clt01_38s.h"

#define ALL_OUTPUTS_OFF         0x00
#define RESET_FLAG              0
    
/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_PLC01A1 X_NUCLEO_PLC01A1
 * @{
 */

/** @addtogroup X_NUCLEO_PLC01A1_IO IO
 * @{
 */

/** @addtogroup X_NUCLEO_PLC01A1_IO_Public_Constants Public constants
 * @{
 */
//#define X_NUCLEO_SPI_EXPBD_CLK_ENABLE() __SPI1_CLK_ENABLE()
#define X_NUCLEO_SPI_EXPBD_CLK_ENABLE() NUCLEO_SPIx_CLK_ENABLE()
#define X_NUCLEO_SPI_EXPBD_MISO_PORT    GPIOA
#define X_NUCLEO_SPI_EXPBD_MISO_PIN     GPIO_PIN_6
//#define X_NUCLEO_SPI_EXPBD_MISO_PIN			GPIO_AF6_SPI1
#ifdef STM32F030x8
#define X_NUCLEO_SPI_MISO_ALTERNATE     GPIO_AF0_SPI1
#elif (STM32F334x8|STM32F401xE)
#define X_NUCLEO_SPI_MISO_ALTERNATE     GPIO_AF5_SPI1
#endif

#define X_NUCLEO_SPI_EXPBD_MOSI_PORT    GPIOA
#define X_NUCLEO_SPI_EXPBD_MOSI_PIN     GPIO_PIN_7
#if defined(STM32F030x8)
#define X_NUCLEO_SPI_MOSI_ALTERNATE     GPIO_AF0_SPI1
#elif defined(STM32F334x8) | defined(STM32F401xE)
#define X_NUCLEO_SPI_MOSI_ALTERNATE     GPIO_AF5_SPI1
#endif


#if defined (STM32F030x8) | defined (STM32F334x8) | defined (STM32F401xE)
#define X_NUCLEO_SPI_EXPBD_SCK_PORT    GPIOB
#elif   defined (STM32F103xB) | defined (STM32F767xx)
#define X_NUCLEO_SPI_EXPBD_SCK_PORT    GPIOA
#endif
#if defined (STM32F030x8) | defined (STM32F334x8) | defined (STM32F401xE)
#define X_NUCLEO_SPI_EXPBD_SCK_PIN      GPIO_PIN_3
#elif   defined (STM32F103xB) | defined (STM32F767xx)
#define X_NUCLEO_SPI_EXPBD_SCK_PIN      GPIO_PIN_5
#endif

#ifdef STM32F030x8
#define X_NUCLEO_SPI_SCK_ALTERNATE     GPIO_AF0_SPI1
#elif (STM32F334x8|STM32F401xE)
#define X_NUCLEO_SPI_SCK_ALTERNATE     GPIO_AF5_SPI1
#endif

#define X_NUCLEO_TIM_PRESCALER          749
  
//#define VNI8200XP_CS_CLK_ENABLE()       __GPIOC_CLK_ENABLE();
#define VNI8200XP_CS_CLK_ENABLE()					__HAL_RCC_GPIOD_CLK_ENABLE();
//#define VNI8200XP_CS_PIN                GPIO_PIN_7
//#define VNI8200XP_CS_PORT               GPIOC
#define VNI8200XP_CS_PORT               GPIOD
#define VNI8200XP_CS_PIN                GPIO_PIN_14

//#define VNI8200XP_OUT_EN_CLK_ENABLE()   __GPIOB_CLK_ENABLE();
#define VNI8200XP_OUT_EN_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE();
//#define VNI8200XP_OUT_EN_PIN            GPIO_PIN_10
//#define VNI8200XP_OUT_EN_PORT           GPIOB
#define VNI8200XP_OUT_EN_PORT           GPIOE
#define VNI8200XP_OUT_EN_PIN						GPIO_PIN_9

//#define CLT01_38S_CS_CLK_ENABLE()       __GPIOB_CLK_ENABLE();
#define CLT01_38S_CS_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE();
//#define CLT01_38S_CS_PIN                GPIO_PIN_6
//#define CLT01_38S_CS_PORT               GPIOB
#define CLT01_38S_CS_PORT               GPIOD
#define CLT01_38S_CS_PIN                GPIO_PIN_15

#define VNI8200XP_RESET_PORT            GPIOC   
#define VNI8200XP_RESET_PIN             GPIO_PIN_7 
   
#define VNI8200XP_ENABLE_PORT           GPIOE
#define VNI8200XP_ENABLE_PIN            GPIO_PIN_9
   
   
uint8_t BSP_Signal_Mirror(uint8_t);
uint8_t BSP_Output_Freeze(uint8_t, uint8_t, uint32_t*);
uint8_t BSP_Output_Regroup(uint8_t);
uint8_t BSP_Inputs_Sum(uint8_t);
uint8_t BSP_Output_ON(uint8_t);
uint8_t BSP_Output_OFF(uint8_t);
uint8_t BSP_Inputs_AND(uint8_t, uint8_t);
uint8_t BSP_Inputs_OR(uint8_t, uint8_t);
uint8_t BSP_Inputs_NOT(uint8_t);
uint8_t BSP_Inputs_XOR(uint8_t, uint8_t);
RELAY_StatusTypeDef BSP_GetRelayStatus(uint8_t*);
CURRENT_LIMITER_StatusTypeDef BSP_GetCurrentLimiterStatus(uint8_t*);
void BSP_OuputCycling_Init(uint16_t, uint8_t);

void BSP_RELAY_Reset(void);
void BSP_RELAY_EN_Out(void);
uint8_t* BSP_RELAY_SetOutputs(uint8_t*);
uint8_t* BSP_CURRENT_LIMITER_Read(void);

RELAY_StatusTypeDef BSP_Relay_Init(void);
CURRENT_LIMITER_StatusTypeDef BSP_CurrentLimiter_Init(void);

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
