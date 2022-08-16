/* ###################################################################
**  Filename		: user_FDCAN.h
**  Title		: Header file of user_FDCAN.c
**  Project		: test environment implementation for EVSPIN32G4 
**  Processor		: STM32G431VBx / STMicroelectronics
**  ToolChain		: IAR Embedded Workbench IDE - Arm 9.10.2
**  Compiler		: IAR C/C++ Compiler for ARM  9.10.2.313 
**  Date/Time		: 2022.02.24
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.1 / 2022.02.24 / skkim
begin
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_FDCAN_H
#define __USER_FDCAN_H	"USER_FDCAN"

#define	USER_FDCAN_VERSION		"0.0.1"		// Reserved.Release.Debug


/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>


/*********************************************************************************
* Defines
*********************************************************************************/
#define USER_FDCAN_VERSION	"0.0.1"		// Reserved.Release.Debug



/*********************************************************************************
* Extern Variables
*********************************************************************************/
extern FDCAN_HandleTypeDef hfdcan1;


/*********************************************************************************
* Extern Functions
*********************************************************************************/
extern void FDCAN_TestFunction(void);

#endif /* __USER_FDCAN_H */
/*********************************************************************************
* End of File
*********************************************************************************/

