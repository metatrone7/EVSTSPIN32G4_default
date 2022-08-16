/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "spin32g4_parameters.h"
#include "User_periodic.h"  
#include "User_uart.h"
#include "User_debug.h"
#include "User_predefine.h"
#include "User_SystemControl.h"
    
#include "stspin32gx_register_config.h"
  
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DBG_ALARM_LED_Pin LL_GPIO_PIN_13
#define DBG_ALARM_LED_GPIO_Port GPIOC
#define GD_WAKE_Pin LL_GPIO_PIN_7
#define GD_WAKE_GPIO_Port GPIOE
#define M1_PWM_UL_Pin LL_GPIO_PIN_8
#define M1_PWM_UL_GPIO_Port GPIOE
#define M1_PWM_UH_Pin LL_GPIO_PIN_9
#define M1_PWM_UH_GPIO_Port GPIOE
#define M1_PWM_VL_Pin LL_GPIO_PIN_10
#define M1_PWM_VL_GPIO_Port GPIOE
#define M1_PWM_VH_Pin LL_GPIO_PIN_11
#define M1_PWM_VH_GPIO_Port GPIOE
#define M1_PWM_WL_Pin LL_GPIO_PIN_12
#define M1_PWM_WL_GPIO_Port GPIOE
#define M1_PWM_WH_Pin LL_GPIO_PIN_13
#define M1_PWM_WH_GPIO_Port GPIOE
#define Start_Stop_Pin LL_GPIO_PIN_10
#define Start_Stop_GPIO_Port GPIOB
#define Start_Stop_EXTI_IRQn EXTI15_10_IRQn
#define GD_SCL_Pin LL_GPIO_PIN_8
#define GD_SCL_GPIO_Port GPIOC
#define GD_SDA_Pin LL_GPIO_PIN_9
#define GD_SDA_GPIO_Port GPIOC
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UART2_DBG_TX_Pin LL_GPIO_PIN_3
#define UART2_DBG_TX_GPIO_Port GPIOB
#define UART2_DBG_RX_Pin LL_GPIO_PIN_4
#define UART2_DBG_RX_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */
																																	 
/**
  ******************************************************************************
  * @file    stm32g4xx_ll_iwdg.h
  * @author  MCD Application Team
  * @brief   Header file of IWDG LL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
					
#define WATCHDOG_ENABLE // Comment out if you want to diable watchdog
#ifdef WATCHDOG_ENABLE	

	extern void SystemClock_Config(void);
	extern void STSPIN32G4_STBY_Recovery_for_CLK(void);
	
	//-------------------------------------------------------------
	/* IWDG_InitTypeDef - Watchdog Struct */
	//-------------------------------------------------------------
	typedef struct
	{
		uint16_t Reload;
		uint16_t Window;
		uint16_t Enable;
		uint16_t Prescaler;
		uint16_t ResetFlag;
		uint32_t Timeout_msec;
	} IWDG_UserTypeDef;
		
	//-------------------------------------------------------------
	/* WWDG_InitTypeDef - Watchdog Struct */
	//-------------------------------------------------------------
	typedef struct
	{
		uint16_t Reload;
		uint16_t Window;
		uint16_t Enable;
		uint16_t Prescaler;
		uint16_t ResetFlag;
		uint32_t Timeout_msec;
	} WWDG_UserTypeDef;

	#ifndef STM32G4xx_LL_IWDG_H
	#define STM32G4xx_LL_IWDG_H

	#ifdef __cplusplus
	extern "C" {
	#endif

	/* Includes ------------------------------------------------------------------*/
	#include "stm32g4xx.h"

	/** @addtogroup STM32G4xx_LL_Driver
		* @{
		*/

	#if defined(IWDG)

	/** @defgroup IWDG_LL IWDG
		* @{
		*/

	/* Private types -------------------------------------------------------------*/
	/* Private variables ---------------------------------------------------------*/

	/* Private constants ---------------------------------------------------------*/
	/** @defgroup IWDG_LL_Private_Constants IWDG Private Constants
		* @{
		*/
	#define LL_IWDG_KEY_RELOAD                 0x0000AAAAU               /*!< IWDG Reload Counter Enable   */
	#define LL_IWDG_KEY_ENABLE                 0x0000CCCCU               /*!< IWDG Peripheral Enable       */
	#define LL_IWDG_KEY_WR_ACCESS_ENABLE       0x00005555U               /*!< IWDG KR Write Access Enable  */
	#define LL_IWDG_KEY_WR_ACCESS_DISABLE      0x00000000U               /*!< IWDG KR Write Access Disable */
	/**
		* @}
		*/

	/* Private macros ------------------------------------------------------------*/

	/* Exported types ------------------------------------------------------------*/
	/* Exported constants --------------------------------------------------------*/
	/** @defgroup IWDG_LL_Exported_Constants IWDG Exported Constants
		* @{
		*/

	/** @defgroup IWDG_LL_EC_GET_FLAG Get Flags Defines
		* @brief    Flags defines which can be used with LL_IWDG_ReadReg function
		* @{
		*/
	#define LL_IWDG_SR_PVU                     IWDG_SR_PVU                           /*!< Watchdog prescaler value update */
	#define LL_IWDG_SR_RVU                     IWDG_SR_RVU                           /*!< Watchdog counter reload value update */
	#define LL_IWDG_SR_WVU                     IWDG_SR_WVU                           /*!< Watchdog counter window value update */
	/**
		* @}
		*/

	/** @defgroup IWDG_LL_EC_PRESCALER  Prescaler Divider
		* @{
		*/
	#define LL_IWDG_PRESCALER_4                0x00000000U                           /*!< Divider by 4   */
	#define LL_IWDG_PRESCALER_8                (IWDG_PR_PR_0)                        /*!< Divider by 8   */
	#define LL_IWDG_PRESCALER_16               (IWDG_PR_PR_1)                        /*!< Divider by 16  */
	#define LL_IWDG_PRESCALER_32               (IWDG_PR_PR_1 | IWDG_PR_PR_0)         /*!< Divider by 32  */
	#define LL_IWDG_PRESCALER_64               (IWDG_PR_PR_2)                        /*!< Divider by 64  */
	#define LL_IWDG_PRESCALER_128              (IWDG_PR_PR_2 | IWDG_PR_PR_0)         /*!< Divider by 128 */
	#define LL_IWDG_PRESCALER_256              (IWDG_PR_PR_2 | IWDG_PR_PR_1)         /*!< Divider by 256 */
	/**
		* @}
		*/

	/**
		* @}
		*/

	/* Exported macro ------------------------------------------------------------*/
	/** @defgroup IWDG_LL_Exported_Macros IWDG Exported Macros
		* @{
		*/

	/** @defgroup IWDG_LL_EM_WRITE_READ Common Write and read registers Macros
		* @{
		*/

	/**
		* @brief  Write a value in IWDG register
		* @param  __INSTANCE__ IWDG Instance
		* @param  __REG__ Register to be written
		* @param  __VALUE__ Value to be written in the register
		* @retval None
		*/
	#define LL_IWDG_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

	/**
		* @brief  Read a value in IWDG register
		* @param  __INSTANCE__ IWDG Instance
		* @param  __REG__ Register to be read
		* @retval Register value
		*/
	#define LL_IWDG_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
	/**
		* @}
		*/

	/**
		* @}
		*/


	/* Exported functions --------------------------------------------------------*/
	/** @defgroup IWDG_LL_Exported_Functions IWDG Exported Functions
		* @{
		*/
	/** @defgroup IWDG_LL_EF_Configuration Configuration
		* @{
		*/

	/**
		* @brief  Start the Independent Watchdog
		* @note   Except if the hardware watchdog option is selected
		* @rmtoll KR           KEY           LL_IWDG_Enable
		* @param  IWDGx IWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_Enable(IWDG_TypeDef *IWDGx)
	{
		WRITE_REG(IWDGx->KR, LL_IWDG_KEY_ENABLE);
	}

	/**
		* @brief  Reloads IWDG counter with value defined in the reload register
		* @rmtoll KR           KEY           LL_IWDG_ReloadCounter
		* @param  IWDGx IWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_ReloadCounter(IWDG_TypeDef *IWDGx)
	{
		WRITE_REG(IWDGx->KR, LL_IWDG_KEY_RELOAD);
	}

	/**
		* @brief  Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers
		* @rmtoll KR           KEY           LL_IWDG_EnableWriteAccess
		* @param  IWDGx IWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_EnableWriteAccess(IWDG_TypeDef *IWDGx)
	{
		WRITE_REG(IWDGx->KR, LL_IWDG_KEY_WR_ACCESS_ENABLE);
	}

	/**
		* @brief  Disable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers
		* @rmtoll KR           KEY           LL_IWDG_DisableWriteAccess
		* @param  IWDGx IWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_DisableWriteAccess(IWDG_TypeDef *IWDGx)
	{
		WRITE_REG(IWDGx->KR, LL_IWDG_KEY_WR_ACCESS_DISABLE);
	}

	/**
		* @brief  Select the prescaler of the IWDG
		* @rmtoll PR           PR            LL_IWDG_SetPrescaler
		* @param  IWDGx IWDG Instance
		* @param  Prescaler This parameter can be one of the following values:
		*         @arg @ref LL_IWDG_PRESCALER_4
		*         @arg @ref LL_IWDG_PRESCALER_8
		*         @arg @ref LL_IWDG_PRESCALER_16
		*         @arg @ref LL_IWDG_PRESCALER_32
		*         @arg @ref LL_IWDG_PRESCALER_64
		*         @arg @ref LL_IWDG_PRESCALER_128
		*         @arg @ref LL_IWDG_PRESCALER_256
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_SetPrescaler(IWDG_TypeDef *IWDGx, uint32_t Prescaler)
	{
		WRITE_REG(IWDGx->PR, IWDG_PR_PR & Prescaler);
	}

	/**
		* @brief  Get the selected prescaler of the IWDG
		* @rmtoll PR           PR            LL_IWDG_GetPrescaler
		* @param  IWDGx IWDG Instance
		* @retval Returned value can be one of the following values:
		*         @arg @ref LL_IWDG_PRESCALER_4
		*         @arg @ref LL_IWDG_PRESCALER_8
		*         @arg @ref LL_IWDG_PRESCALER_16
		*         @arg @ref LL_IWDG_PRESCALER_32
		*         @arg @ref LL_IWDG_PRESCALER_64
		*         @arg @ref LL_IWDG_PRESCALER_128
		*         @arg @ref LL_IWDG_PRESCALER_256
		*/
	__STATIC_INLINE uint32_t LL_IWDG_GetPrescaler(IWDG_TypeDef *IWDGx)
	{
		return (READ_REG(IWDGx->PR));
	}

	/**
		* @brief  Specify the IWDG down-counter reload value
		* @rmtoll RLR          RL            LL_IWDG_SetReloadCounter
		* @param  IWDGx IWDG Instance
		* @param  Counter Value between Min_Data=0 and Max_Data=0x0FFF
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_SetReloadCounter(IWDG_TypeDef *IWDGx, uint32_t Counter)
	{
		WRITE_REG(IWDGx->RLR, IWDG_RLR_RL & Counter);
	}

	/**
		* @brief  Get the specified IWDG down-counter reload value
		* @rmtoll RLR          RL            LL_IWDG_GetReloadCounter
		* @param  IWDGx IWDG Instance
		* @retval Value between Min_Data=0 and Max_Data=0x0FFF
		*/
	__STATIC_INLINE uint32_t LL_IWDG_GetReloadCounter(IWDG_TypeDef *IWDGx)
	{
		return (READ_REG(IWDGx->RLR));
	}

	/**
		* @brief  Specify high limit of the window value to be compared to the down-counter.
		* @rmtoll WINR         WIN           LL_IWDG_SetWindow
		* @param  IWDGx IWDG Instance
		* @param  Window Value between Min_Data=0 and Max_Data=0x0FFF
		* @retval None
		*/
	__STATIC_INLINE void LL_IWDG_SetWindow(IWDG_TypeDef *IWDGx, uint32_t Window)
	{
		WRITE_REG(IWDGx->WINR, IWDG_WINR_WIN & Window);
	}

	/**
		* @brief  Get the high limit of the window value specified.
		* @rmtoll WINR         WIN           LL_IWDG_GetWindow
		* @param  IWDGx IWDG Instance
		* @retval Value between Min_Data=0 and Max_Data=0x0FFF
		*/
	__STATIC_INLINE uint32_t LL_IWDG_GetWindow(IWDG_TypeDef *IWDGx)
	{
		return (READ_REG(IWDGx->WINR));
	}

	/**
		* @}
		*/

	/** @defgroup IWDG_LL_EF_FLAG_Management FLAG_Management
		* @{
		*/

	/**
		* @brief  Check if flag Prescaler Value Update is set or not
		* @rmtoll SR           PVU           LL_IWDG_IsActiveFlag_PVU
		* @param  IWDGx IWDG Instance
		* @retval State of bit (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_PVU(IWDG_TypeDef *IWDGx)
	{
		return ((READ_BIT(IWDGx->SR, IWDG_SR_PVU) == (IWDG_SR_PVU)) ? 1UL : 0UL);
	}

	/**
		* @brief  Check if flag Reload Value Update is set or not
		* @rmtoll SR           RVU           LL_IWDG_IsActiveFlag_RVU
		* @param  IWDGx IWDG Instance
		* @retval State of bit (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_RVU(IWDG_TypeDef *IWDGx)
	{
		return ((READ_BIT(IWDGx->SR, IWDG_SR_RVU) == (IWDG_SR_RVU)) ? 1UL : 0UL);
	}

	/**
		* @brief  Check if flag Window Value Update is set or not
		* @rmtoll SR           WVU           LL_IWDG_IsActiveFlag_WVU
		* @param  IWDGx IWDG Instance
		* @retval State of bit (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_WVU(IWDG_TypeDef *IWDGx)
	{
		return ((READ_BIT(IWDGx->SR, IWDG_SR_WVU) == (IWDG_SR_WVU)) ? 1UL : 0UL);
	}

	/**
		* @brief  Check if all flags Prescaler, Reload & Window Value Update are reset or not
		* @rmtoll SR           PVU           LL_IWDG_IsReady\n
		*         SR           RVU           LL_IWDG_IsReady\n
		*         SR           WVU           LL_IWDG_IsReady
		* @param  IWDGx IWDG Instance
		* @retval State of bits (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_IWDG_IsReady(IWDG_TypeDef *IWDGx)
	{
		return ((READ_BIT(IWDGx->SR, IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU) == 0U) ? 1UL : 0UL);
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

	#endif /* IWDG */

	/**
		* @}
		*/

	#ifdef __cplusplus
	}
	#endif

	#endif /* STM32G4xx_LL_IWDG_H */
	/**
		******************************************************************************
		* @file    stm32g4xx_ll_wwdg.h
		* @author  MCD Application Team
		* @brief   Header file of WWDG LL module.
		******************************************************************************
		* @attention
		*
		* Copyright (c) 2019 STMicroelectronics.
		* All rights reserved.
		*
		* This software is licensed under terms that can be found in the LICENSE file
		* in the root directory of this software component.
		* If no LICENSE file comes with this software, it is provided AS-IS.
		*
		******************************************************************************
		*/

	/* Define to prevent recursive inclusion -------------------------------------*/
	#ifndef STM32G4xx_LL_WWDG_H
	#define STM32G4xx_LL_WWDG_H

	#ifdef __cplusplus
	extern "C" {
	#endif

	/* Includes ------------------------------------------------------------------*/
	#include "stm32g4xx.h"

	/** @addtogroup STM32G4xx_LL_Driver
		* @{
		*/

	#if defined (WWDG)

	/** @defgroup WWDG_LL WWDG
		* @{
		*/

	/* Private types -------------------------------------------------------------*/
	/* Private variables ---------------------------------------------------------*/
	/* Private constants ---------------------------------------------------------*/
	/* Private macros ------------------------------------------------------------*/
	/* Exported types ------------------------------------------------------------*/
	/* Exported constants --------------------------------------------------------*/
	/** @defgroup WWDG_LL_Exported_Constants WWDG Exported Constants
		* @{
		*/

	/** @defgroup WWDG_LL_EC_IT IT Defines
		* @brief    IT defines which can be used with LL_WWDG_ReadReg and  LL_WWDG_WriteReg functions
		* @{
		*/
	#define LL_WWDG_CFR_EWI                     WWDG_CFR_EWI
	/**
		* @}
		*/

	/** @defgroup WWDG_LL_EC_PRESCALER  PRESCALER
		* @{
		*/
	#define LL_WWDG_PRESCALER_1                 0x00000000u                                               /*!< WWDG counter clock = (PCLK1/4096)/1 */
	#define LL_WWDG_PRESCALER_2                 WWDG_CFR_WDGTB_0                                          /*!< WWDG counter clock = (PCLK1/4096)/2 */
	#define LL_WWDG_PRESCALER_4                 WWDG_CFR_WDGTB_1                                          /*!< WWDG counter clock = (PCLK1/4096)/4 */
	#define LL_WWDG_PRESCALER_8                 (WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1)                     /*!< WWDG counter clock = (PCLK1/4096)/8 */
	#define LL_WWDG_PRESCALER_16                WWDG_CFR_WDGTB_2                                          /*!< WWDG counter clock = (PCLK1/4096)/16 */
	#define LL_WWDG_PRESCALER_32                (WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_0)                     /*!< WWDG counter clock = (PCLK1/4096)/32 */
	#define LL_WWDG_PRESCALER_64                (WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_1)                     /*!< WWDG counter clock = (PCLK1/4096)/64 */
	#define LL_WWDG_PRESCALER_128               (WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_1 | WWDG_CFR_WDGTB_0)  /*!< WWDG counter clock = (PCLK1/4096)/128 */
	/**
		* @}
		*/

	/**
		* @}
		*/

	/* Exported macro ------------------------------------------------------------*/
	/** @defgroup WWDG_LL_Exported_Macros WWDG Exported Macros
		* @{
		*/
	/** @defgroup WWDG_LL_EM_WRITE_READ Common Write and read registers macros
		* @{
		*/
	/**
		* @brief  Write a value in WWDG register
		* @param  __INSTANCE__ WWDG Instance
		* @param  __REG__ Register to be written
		* @param  __VALUE__ Value to be written in the register
		* @retval None
		*/
	#define LL_WWDG_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

	/**
		* @brief  Read a value in WWDG register
		* @param  __INSTANCE__ WWDG Instance
		* @param  __REG__ Register to be read
		* @retval Register value
		*/
	#define LL_WWDG_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
	/**
		* @}
		*/

	/**
		* @}
		*/

	/* Exported functions --------------------------------------------------------*/
	/** @defgroup WWDG_LL_Exported_Functions WWDG Exported Functions
		* @{
		*/

	/** @defgroup WWDG_LL_EF_Configuration Configuration
		* @{
		*/
	/**
		* @brief  Enable Window Watchdog. The watchdog is always disabled after a reset.
		* @note   It is enabled by setting the WDGA bit in the WWDG_CR register,
		*         then it cannot be disabled again except by a reset.
		*         This bit is set by software and only cleared by hardware after a reset.
		*         When WDGA = 1, the watchdog can generate a reset.
		* @rmtoll CR           WDGA          LL_WWDG_Enable
		* @param  WWDGx WWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_WWDG_Enable(WWDG_TypeDef *WWDGx)
	{
		SET_BIT(WWDGx->CR, WWDG_CR_WDGA);
	}

	/**
		* @brief  Checks if Window Watchdog is enabled
		* @rmtoll CR           WDGA          LL_WWDG_IsEnabled
		* @param  WWDGx WWDG Instance
		* @retval State of bit (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_WWDG_IsEnabled(WWDG_TypeDef *WWDGx)
	{
		return ((READ_BIT(WWDGx->CR, WWDG_CR_WDGA) == (WWDG_CR_WDGA)) ? 1UL : 0UL);
	}

	/**
		* @brief  Set the Watchdog counter value to provided value (7-bits T[6:0])
		* @note   When writing to the WWDG_CR register, always write 1 in the MSB b6 to avoid generating an immediate reset
		*         This counter is decremented every (4096 x 2expWDGTB) PCLK cycles
		*         A reset is produced when it rolls over from 0x40 to 0x3F (bit T6 becomes cleared)
		*         Setting the counter lower then 0x40 causes an immediate reset (if WWDG enabled)
		* @rmtoll CR           T             LL_WWDG_SetCounter
		* @param  WWDGx WWDG Instance
		* @param  Counter 0..0x7F (7 bit counter value)
		* @retval None
		*/
	__STATIC_INLINE void LL_WWDG_SetCounter(WWDG_TypeDef *WWDGx, uint32_t Counter)
	{
		MODIFY_REG(WWDGx->CR, WWDG_CR_T, Counter);
	}

	/**
		* @brief  Return current Watchdog Counter Value (7 bits counter value)
		* @rmtoll CR           T             LL_WWDG_GetCounter
		* @param  WWDGx WWDG Instance
		* @retval 7 bit Watchdog Counter value
		*/
	__STATIC_INLINE uint32_t LL_WWDG_GetCounter(WWDG_TypeDef *WWDGx)
	{
		return (READ_BIT(WWDGx->CR, WWDG_CR_T));
	}

	/**
		* @brief  Set the time base of the prescaler (WDGTB).
		* @note   Prescaler is used to apply ratio on PCLK clock, so that Watchdog counter
		*         is decremented every (4096 x 2expWDGTB) PCLK cycles
		* @rmtoll CFR          WDGTB         LL_WWDG_SetPrescaler
		* @param  WWDGx WWDG Instance
		* @param  Prescaler This parameter can be one of the following values:
		*         @arg @ref LL_WWDG_PRESCALER_1
		*         @arg @ref LL_WWDG_PRESCALER_2
		*         @arg @ref LL_WWDG_PRESCALER_4
		*         @arg @ref LL_WWDG_PRESCALER_8
		*         @arg @ref LL_WWDG_PRESCALER_16
		*         @arg @ref LL_WWDG_PRESCALER_32
		*         @arg @ref LL_WWDG_PRESCALER_64
		*         @arg @ref LL_WWDG_PRESCALER_128
		* @retval None
		*/
	__STATIC_INLINE void LL_WWDG_SetPrescaler(WWDG_TypeDef *WWDGx, uint32_t Prescaler)
	{
		MODIFY_REG(WWDGx->CFR, WWDG_CFR_WDGTB, Prescaler);
	}

	/**
		* @brief  Return current Watchdog Prescaler Value
		* @rmtoll CFR          WDGTB         LL_WWDG_GetPrescaler
		* @param  WWDGx WWDG Instance
		* @retval Returned value can be one of the following values:
		*         @arg @ref LL_WWDG_PRESCALER_1
		*         @arg @ref LL_WWDG_PRESCALER_2
		*         @arg @ref LL_WWDG_PRESCALER_4
		*         @arg @ref LL_WWDG_PRESCALER_8
		*         @arg @ref LL_WWDG_PRESCALER_16
		*         @arg @ref LL_WWDG_PRESCALER_32
		*         @arg @ref LL_WWDG_PRESCALER_64
		*         @arg @ref LL_WWDG_PRESCALER_128
		*/
	__STATIC_INLINE uint32_t LL_WWDG_GetPrescaler(WWDG_TypeDef *WWDGx)
	{
		return (READ_BIT(WWDGx->CFR, WWDG_CFR_WDGTB));
	}

	/**
		* @brief  Set the Watchdog Window value to be compared to the downcounter (7-bits W[6:0]).
		* @note   This window value defines when write in the WWDG_CR register
		*         to program Watchdog counter is allowed.
		*         Watchdog counter value update must occur only when the counter value
		*         is lower than the Watchdog window register value.
		*         Otherwise, a MCU reset is generated if the 7-bit Watchdog counter value
		*         (in the control register) is refreshed before the downcounter has reached
		*         the watchdog window register value.
		*         Physically is possible to set the Window lower then 0x40 but it is not recommended.
		*         To generate an immediate reset, it is possible to set the Counter lower than 0x40.
		* @rmtoll CFR          W             LL_WWDG_SetWindow
		* @param  WWDGx WWDG Instance
		* @param  Window 0x00..0x7F (7 bit Window value)
		* @retval None
		*/
	__STATIC_INLINE void LL_WWDG_SetWindow(WWDG_TypeDef *WWDGx, uint32_t Window)
	{
		MODIFY_REG(WWDGx->CFR, WWDG_CFR_W, Window);
	}

	/**
		* @brief  Return current Watchdog Window Value (7 bits value)
		* @rmtoll CFR          W             LL_WWDG_GetWindow
		* @param  WWDGx WWDG Instance
		* @retval 7 bit Watchdog Window value
		*/
	__STATIC_INLINE uint32_t LL_WWDG_GetWindow(WWDG_TypeDef *WWDGx)
	{
		return (READ_BIT(WWDGx->CFR, WWDG_CFR_W));
	}

	/**
		* @}
		*/

	/** @defgroup WWDG_LL_EF_FLAG_Management FLAG_Management
		* @{
		*/
	/**
		* @brief  Indicates if the WWDG Early Wakeup Interrupt Flag is set or not.
		* @note   This bit is set by hardware when the counter has reached the value 0x40.
		*         It must be cleared by software by writing 0.
		*         A write of 1 has no effect. This bit is also set if the interrupt is not enabled.
		* @rmtoll SR           EWIF          LL_WWDG_IsActiveFlag_EWKUP
		* @param  WWDGx WWDG Instance
		* @retval State of bit (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_WWDG_IsActiveFlag_EWKUP(WWDG_TypeDef *WWDGx)
	{
		return ((READ_BIT(WWDGx->SR, WWDG_SR_EWIF) == (WWDG_SR_EWIF)) ? 1UL : 0UL);
	}

	/**
		* @brief  Clear WWDG Early Wakeup Interrupt Flag (EWIF)
		* @rmtoll SR           EWIF          LL_WWDG_ClearFlag_EWKUP
		* @param  WWDGx WWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_WWDG_ClearFlag_EWKUP(WWDG_TypeDef *WWDGx)
	{
		WRITE_REG(WWDGx->SR, ~WWDG_SR_EWIF);
	}

	/**
		* @}
		*/

	/** @defgroup WWDG_LL_EF_IT_Management IT_Management
		* @{
		*/
	/**
		* @brief  Enable the Early Wakeup Interrupt.
		* @note   When set, an interrupt occurs whenever the counter reaches value 0x40.
		*         This interrupt is only cleared by hardware after a reset
		* @rmtoll CFR          EWI           LL_WWDG_EnableIT_EWKUP
		* @param  WWDGx WWDG Instance
		* @retval None
		*/
	__STATIC_INLINE void LL_WWDG_EnableIT_EWKUP(WWDG_TypeDef *WWDGx)
	{
		SET_BIT(WWDGx->CFR, WWDG_CFR_EWI);
	}

	/**
		* @brief  Check if Early Wakeup Interrupt is enabled
		* @rmtoll CFR          EWI           LL_WWDG_IsEnabledIT_EWKUP
		* @param  WWDGx WWDG Instance
		* @retval State of bit (1 or 0).
		*/
	__STATIC_INLINE uint32_t LL_WWDG_IsEnabledIT_EWKUP(WWDG_TypeDef *WWDGx)
	{
		return ((READ_BIT(WWDGx->CFR, WWDG_CFR_EWI) == (WWDG_CFR_EWI)) ? 1UL : 0UL);
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

	#endif /* WWDG */

	/**
		* @}
		*/

	#ifdef __cplusplus
	}
	#endif

	#endif /* STM32G4xx_LL_WWDG_H */

#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
