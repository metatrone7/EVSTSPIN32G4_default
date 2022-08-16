/* ###################################################################
**  Filename		: User_SystemControl.c
**  Title		: System control functions to control EVSPIN32G4
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

/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> 

#include "stm32g4xx.h"

#include "User_SystemControl.h"
#include "User_predefine.h"
#include "User_uart.h"
#include "User_util.h"


/*********************************************************************************
* Variables
*********************************************************************************/
System_Ctrl_Flag_t	stSpinSysCtrl;		// System Control Flag
SystemError_Flag_t	stSystemErrorFlag;	// System Error flags


/*********************************************************************************
* Functions
*********************************************************************************/

//-------------------------------------------------------------
// Function name	: PrimarySequence
// Description		: System Mode check
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PrimarySequence(void)
{    
  // System Mode Selection
  if(stSystemErrorFlag.bEmergency_flag == NORMAL)
  {
    if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubIdle == SET)
    {
      //SecondarySequence_SystemIdle();
    }
    else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubMaintenance == SET)
    {
      //SecondarySequence_Maintenance();
    }
    else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubRunTest == SET)
    {
      //SecondarySequence_RunTest();
    }
    else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubDebug == SET)
    {
      //SecondarySequence_Debug();
    }
    else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control == SET)
    {
      //SecondarySequence_Debug();
    }
    else 
    {
      //SecondarySequence_Selection_Error();
    }
    
    if(stSystemErrorFlag.bEmergencyStop_flag)
    {
      stSystemErrorFlag.bEmergencyStop_flag = FALSE;
    }
  }
  else if(stSystemErrorFlag.bEmergency_flag == EMERGENCY)
  {
    //EmergencySequence();
    ;
  }
  
}

/*********************************************************************************
* End of File
*********************************************************************************/

