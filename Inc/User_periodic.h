/* ###################################################################
**  Filename		: User_periodic.h
**  Title		: Header file of User_periodic.c
**  Project		: test environment implementation for EVSPIN32G4 
**  Processor		: STM32G431VBx / STMicroelectronics
**  ToolChain		: IAR Embedded Workbench IDE - Arm 9.10.2
**  Compiler		: IAR C/C++ Compiler for ARM  9.10.2.313 
**  Date/Time		: 2022.02.03
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.1 / 2022.02.03 / Sungkyu Kim
begin
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_PERIODIC_H
#define __USER_PERIODIC_H "USER_PERIODIC"

#define	USER_PERIODIC_VERSION		"0.0.1"		// Reserved.Release.Debug

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32g4xx.h"
#include "User_predefine.h"
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
/* USER CODE BEGIN EFP */
extern void RunPeriodicTimerCounter(void);
extern void PeriodicFunction(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

// -- Debug Alarm LED Macro
#define LED2_ON         WRITE_REG(DBG_ALARM_LED_GPIO_Port->BSRR, DBG_ALARM_LED_Pin);
#define LED2_OFF        WRITE_REG(DBG_ALARM_LED_GPIO_Port->BRR, DBG_ALARM_LED_Pin);
#define LED2_TOGGLE     WRITE_REG(DBG_ALARM_LED_GPIO_Port->ODR, READ_REG(DBG_ALARM_LED_GPIO_Port->ODR) ^ DBG_ALARM_LED_Pin);

/* USER CODE END Private defines */

#endif /*__USER_PERIODIC_H*/
/*********************************************************************************
* End of File
*********************************************************************************/