/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    stm32g4xx_it.c
* @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "User_periodic.h"
#include "User_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
__IO uint32_t uwTick;
uint16_t uDMA1CH1_Count = 0;
uint16_t uDMA1CH4_Count = 0;
uint16_t uTIM1CH4_Count = 6;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  if(++uwTick >= 65535) uwTick = 0;
  
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  RunPeriodicTimerCounter();
  
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	uint16_t _for_i;
	if(LL_DMA_IsActiveFlag_HT1(DMA1))
	{
		GPA11_ON;
		LL_DMA_ClearFlag_HT1(DMA1);
	}
	
	if(LL_DMA_IsActiveFlag_TC1(DMA1))
	{
		
		GPA11_OFF;
		LL_DMA_ClearFlag_TC1(DMA1);

		uDMA1CH1_Count = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
		uDMA1CH4_Count = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_4);
		
		for(_for_i = 0; _for_i < guc_ADC1_READ_NUM; _for_i++)
		{
			gush_DMA_ADC1_MEM_mV[_for_i] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, gush_DMA_ADC1_MEM[_for_i], LL_ADC_RESOLUTION_12B);   
			gush_DMA_ADC1_MEM[_for_i] = 0;
		}

	}

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	uint16_t _for_i;
	
	if(LL_DMA_IsActiveFlag_HT2(DMA1))
	{
		GPA12_ON;
		LL_DMA_ClearFlag_HT2(DMA1);
	}
	
	if(LL_DMA_IsActiveFlag_TC2(DMA1))
	{
		LL_DMA_ClearFlag_TC2(DMA1);
		
		for(_for_i = 0; _for_i < ADC2_READ_NUM; _for_i++)
		{
			gush_DMA_ADC2_MEM_mV[_for_i] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, gush_DMA_ADC2_MEM[_for_i], LL_ADC_RESOLUTION_12B);   
			gush_DMA_ADC2_MEM[_for_i] = 0;
		}
		
		GPA12_OFF;
	}
  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
	if(LL_DMA_IsActiveFlag_HT4(DMA1))
	{
		LL_DMA_ClearFlag_HT4(DMA1);
	}
	
	if(LL_DMA_IsActiveFlag_TC4(DMA1))
	{
		LL_DMA_ClearFlag_TC4(DMA1);
	}

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	if(LL_ADC_IsActiveFlag_EOC(ADC2))
	{
		LL_ADC_ClearFlag_EOC(ADC2);
	  //gush_DMA_ADC2_MEM[0] = LL_ADC_REG_ReadConversionData12(ADC2);
	}
  
  /* USER CODE END ADC1_2_IRQn 0 */

  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	if(LL_ADC_IsActiveFlag_JEOC(ADC2))
	{
		GPA8_ON;
	  gush_INJ_ADC2_MEM = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
		gush_INJ_ADC2_MEM_mV = 
			__LL_ADC_CALC_DATA_TO_VOLTAGE(3300, 
																		gush_INJ_ADC2_MEM, 
																		LL_ADC_RESOLUTION_12B); 
		gush_INJ_ADC2_MEM = 0;
			//LL_ADC_ClearFlag_JEOC(ADC2);
		GPA8_OFF;	
	}
  
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  
	uint16_t _for_i;
	uint16_t uForICnt = 50;
	
	if(LL_TIM_IsActiveFlag_UPDATE(TIM1))
	{
		LL_TIM_ClearFlag_UPDATE(TIM1);
		
		if(LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP)
		{
			//while(LL_TIM_GetCounter(TIM1) < TIM1->CCR4){}
			//while(LL_TIM_GetCounter(TIM1) < gush_DMA_Trg1_MEM[0]){}
			//while(LL_TIM_GetCounter(TIM1) < gush_DMA_Trg1_MEM[1]){}
			_for_i = uForICnt; while(--_for_i){} 
		}
		else
		{
			_for_i = uForICnt; while(--_for_i){} 
			//while(LL_TIM_GetCounter(TIM1) > gush_DMA_Trg1_MEM[3]){}
			//_for_i = uForICnt; while(--_for_i){} 
			//while(LL_TIM_GetCounter(TIM1) > gush_DMA_Trg1_MEM[4]){}
		}
	}
	
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
  
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  if(LL_USART_IsActiveFlag_RXNE(USART2))
  {
    //    cUART2_RxData[uUART2_RxIdx] = LL_USART_ReceiveData8(USART2);
    
    //    if(++uUART2_RxIdx >= uUART2_BufferMax)
    //    {
    //      uUART2_RxIdx = 0;    
    //    }  
    
    stSci_Debug.bRX_flag = TRUE;
    stSci_Debug.cRX_Buffer[stSci_Debug.uRX_Buffer_index++] = LL_USART_ReceiveData8(USART2);
    stSci_Debug.cRX_Buffer[stSci_Debug.uRX_Buffer_index] = NULL;
    
    if(stSci_Debug.uRX_Buffer_index >= UART_RX_BUFFER_SIZE)		// Buffer Overflow
    {
      stSci_Debug.uRX_Buffer_index = 0;
      stSci_Debug.bRX_Buffer_over_flag = TRUE;
      stSci_Debug.cRX_Buffer[stSci_Debug.uRX_Buffer_index] = NULL;
    }
  }
  
  //  if(LL_USART_IsActiveFlag_TXE(USART2))
  //  {
  //    ;
  //  }
  
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */
  
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  
  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    /* USER CODE BEGIN LL_EXTI_LINE_10 */
    
    /* USER CODE END LL_EXTI_LINE_10 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14 */
		if(LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_14))
		{
			User_printf(SCI_DEBUG, "\r\n[!] RDY Rising ");
			if(bSystemInitializeFinishFlag == YES)
			{
				bReadyPinExternalInterruptFlag = YES;	
				bReadyPinInput = HIGH;
			}
		}
		else
		{
			User_printf(SCI_DEBUG, "\r\n[!] RDY Falling ");				
			if(bSystemInitializeFinishFlag == YES)
			{
				bReadyPinExternalInterruptFlag = YES;	
				bReadyPinInput = LOW;
			}
		}
		
    /* USER CODE END LL_EXTI_LINE_14 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    /* USER CODE BEGIN LL_EXTI_LINE_15 */
		if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n[!] nFault Triggered");
				
		if(bSystemInitializeFinishFlag == YES)
		{
			bFaultPinExternalInterruptFlag = YES;	
			bFaultPinInput = LOW;	
		}

    /* USER CODE END LL_EXTI_LINE_15 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM8))
	{	
		LL_TIM_ClearFlag_UPDATE(TIM8);
	}

  /* USER CODE END TIM8_UP_IRQn 0 */
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

  /* USER CODE END TIM8_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
	if(LL_TIM_IsActiveFlag_CC1(TIM8))
	{	
		LL_TIM_ClearFlag_CC1(TIM8);
	}
	
	if(LL_TIM_IsActiveFlag_CC4(TIM8))
	{	
		LL_TIM_ClearFlag_CC4(TIM8);
	}

  /* USER CODE END TIM8_CC_IRQn 0 */
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
