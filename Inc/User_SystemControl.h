/* ###################################################################
**  Filename		: User_SystemControl.h
**  Title		: Header file of User_SystemControl.c
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
#ifndef __USER_SYSTEMCONTROL_H
#define __USER_SYSTEMCONTROL_H	"USER_SYSTEMCONTROL"

#define	USER_SYSTEMCONTROL_VERSION	"0.0.1"	


/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "User_predefine.h"
//#include "stm32g4xx_ll_tim.h"


/*********************************************************************************
* Defines
*********************************************************************************/

// Motor Error
#define ERR_MOTOR_ACTIVE	0x1	// Activation(run) error
#define ERR_MOTOR_INACTIVE	0x2	// Inactivation(stop) error
#define ERR_MOTOR_RUN           0x3     // Running error

// System Control Mode definitions
#define MODE_IDLE		0x0001
#define MODE_ROTATING           0x0002
#define MODE_MAINTENANCE        0x0004
#define MODE_RUNTEST            0x0008
#define MODE_DEBUG              0x0010
#define MODE_PC_CONTROL	        0x0020

//-------------------------------------------------------------
/* System_Ctrl_Flag_t - System Control Mode Register */
struct SYS_MACHINE_BITS
{  
  uint16_t	ubIdle          :1; // Idle mode
  uint16_t	ubRotating      :1; // Rotating mode
  uint16_t	ubMaintenance   :1; // Maintenance mode
  uint16_t	ubRunTest       :1; // Runtest mode
  uint16_t	ubDebug         :1; // Debug mode
  uint16_t	ubPC_Control    :1; // PC Control mode
  uint16_t	rsvd            :10; // Reserved
};
union SYSTEM_PRIMARY_SEQ
{
  uint16_t	all;
  struct	SYS_MACHINE_BITS  bit;
};

//-------------------------------------------------------------
/* System_Ctrl_Flag_t - Rotating Sequence */
struct SPIN32G4_DEBUG_BITS
{
  uint16_t	ubInitializeFlag        :1;	// Rotating Mode Initialize
  uint16_t	ubRunStatus		:1;	// Rotating Status (Active/Inactive)
  uint16_t	rsvd			:14;// Reserved
};
union SPIN32G4_DEBUG
{
  uint16_t	all;
  struct	SPIN32G4_DEBUG_BITS	bit;
};

//-------------------------------------------------------------
/* System_Ctrl_Flag_t - Motor control */
struct MOTOR_CONTROL
{
  // Rotating Motor
  uint16_t	uRotatingPwmPrd;    	    // Target compare count
  bool		bMotorCommLineStatusFlag;      // 0: Off, 1: On
};

//-------------------------------------------------------------
/* System_Ctrl_Flag_t - Main Struct */
//-------------------------------------------------------------
typedef struct
{    
  union		SYSTEM_PRIMARY_SEQ	PRIMARY_SEQ;	// Primary Sequence Select  
  union		SPIN32G4_DEBUG		SPIN32G4_SEQ;	// Secondary Sequence - Rotating Mode  
  struct	MOTOR_CONTROL		MOTOR;
  uint8_t	ucOperatingMode;        	        // STSPIN32G4 Operating Mode
    
} System_Ctrl_Flag_t;

//-------------------------------------------------------------
/* SystemError_Flag_t - Motor Error Flag */
struct BLDCM_BITS	// bits description
{
  // 0 - Normal State
  // 1 - Motor Start Error
  // 2 - Motor Stop Error
  // 3 - Encoder error during drive
  uint16_t	ubBLDCM01	:2;	// 0:1   Rotating Motor		
  uint32_t	rsvd		:30;// 2:31 Reserved
};
union BLDCM_ERROR_REG
{
  uint32_t  all;
  struct  BLDCM_BITS   bit;
};

//-------------------------------------------------------------
/* SystemError_Flag_t - Main Struct */
//-------------------------------------------------------------
typedef struct
{
  bool	bErrorClear_flag;
  bool	bError_flag;  
  bool	bEmergency_flag;
  bool	bEmergencyStop_flag;
  
  union	BLDCM_ERROR_REG		BLDCM;           	// DC Motor Error
} SystemError_Flag_t;
//-------------------------------------------------------------


/*********************************************************************************
* Extern Variables
*********************************************************************************/
extern System_Ctrl_Flag_t	stSpinSysCtrl;		// System Control Flag
extern SystemError_Flag_t	stSystemErrorFlag;	// SystemError flags


/*********************************************************************************
* Extern Functions
*********************************************************************************/
extern void PrimarySequence(void);


#endif /* __USER_SYSTEMCONTROL_H */
/*********************************************************************************
* End of File
*********************************************************************************/
