/* ###################################################################
**  Filename		: User_debug.c
**  Title		: PC Command analysis and print to PC for debugging
**  Project		: test environment implementation for EVSPIN32G4 
**  Processor		: STM32G431VBx / STMicroelectronics
**  ToolChain		: IAR Embedded Workbench IDE - Arm 9.10.2
**  Compiler		: IAR C/C++ Compiler for ARM  9.10.2.313 
**  Date/Time		: 2022.02.03
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.1 / 2022.02.03 / skkim
begin
*/

/*********************************************************************************
* Includes
*********************************************************************************/
#include "main.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "spin32g4_parameters.h"

#include "User_predefine.h" 
#include "User_util.h"
#include "User_uart.h"
#include "User_debug.h"
#include "User_periodic.h" 
#include "User_SystemControl.h"

#include "stspin32gx_i2c.h"
#include "stspin32gx_register_config.h"



/*********************************************************************************
* Variables
*********************************************************************************/
bool	            bDebug_flag = FALSE;
Debug_Print_t       stDbgPrt;

uint8_t	ucEVSPIN32G4_Version_Major = 0;	// 0 or 1
uint8_t	ucEVSPIN32G4_Version_Minor = 1;	// 0 ~ 7

bool	bSystemInitializeFinishFlag;
bool	bReadyPinExternalInterruptFlag;
bool	bReadyPinInput;
bool	bFaultPinExternalInterruptFlag;
bool	bFaultPinInput;


/*********************************************************************************
* Function Declares
*********************************************************************************/
void Source_File_Version(void);
void DebugMenu(uint8_t _ucOption, uint8_t _ucPort);



/*********************************************************************************
* Functions
*********************************************************************************/
//-------------------------------------------------------------
// Function name	: Init_DebugFlag
// Description		: Device Debug Flag ÃÊ±âÈ­.
// Parameter		: ¾øÀ½.
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void Init_DebugFlag(void)
{
	stDbgPrt.bDebug	= SET;
}


//-------------------------------------------------------------
// Function name	: GetPcbVersion
// Description		: PCB ¹öÀü ÀÐ±â.
// Parameter		: ¾øÀ½.
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void GetPcbVersion(void)
{
	uint8_t	_ucPCB_Ver;
	
	_ucPCB_Ver = 1;
	
	ucEVSPIN32G4_Version_Major = (_ucPCB_Ver >> 3) & 0x01;
	ucEVSPIN32G4_Version_Minor = _ucPCB_Ver & 0x07;
}


//-------------------------------------------------------------
// Function name	: PISensor_Test
// Description		: GPIO PI Sensor Test.
// Parameter		: ¾øÀ½.
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void PISensor_Test(void)
{
	//if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\nPIC01 = %u", GET_DISPENSE_SHUTTER_HIGH);
	EMPTY_LINE(SCI_DEBUG);
}


//-------------------------------------------------------------
// Function name	: TestFunction
// Description		: Å×½ºÆ® ÇÔ¼ö.
// Parameter		: _sTemp = ¹®ÀÚ¿­
//					  _ucPort = Ãâ·ÂÆ÷Æ®
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void TestFunction(uint8_t _ucPort, char* _sTemp)
{
	if(strlen(_sTemp) == 2)
	{
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n Function %c\r\n", _sTemp[1]);
		switch(_sTemp[1])
		{
		case '1':
			break;
			
		case '2':
			break;
			
		default :
			if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[Err] Invalid TestFunction Number 1-10 !!!\r\n");
			break;
		}
	}
	else if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[Err] Invalid Count !!!\r\n");
}

//-------------------------------------------------------------
// Function name	: MotorTest
// Description		: ¸ðÅÍ Å×½ºÆ® ÇÔ¼ö.
// Parameter		: _sTemp = ¹®ÀÚ¿­
//					  _ucPort = Ãâ·ÂÆ÷Æ®
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void MotorTest(uint8_t _ucPort, char* _sTemp)
{
	//	char _string[10];
	
	if(strlen(_sTemp) == 2)
	{
		//if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n Function %c\r\n", _sTemp[1]);
		switch(_sTemp[1])
		{
		case '5':
			//                if(stDMA01.uRunStatus == INACTIVE)
			//                {
			//                    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  5 = DMA01   : AM Reverse Motor Run");
			////                    ReverseMotor_Run();
			//                }
			//                else if(stDMA01.uRunStatus == ACTIVE)
			//                {
			//                    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  5 = DMA01   : AM Reverse Motor Stop");
			////                    ReverseMotor_Stop();
			//                }
			break;
			
		case '0':
			break;
			
		default :
			if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n [Err] Invalid TestFunction Number 1-6 !!!\r\n");
			break;
		}
	}
	else if(strncmp(_sTemp, "m3 ", 3) == NULL)
	{
		switch(_sTemp[3])
		{
		case '1':
			//				stSpinSysCtrl.MOTOR.uRotatingPwmPrd = 850;
			//				stDMA01.uTargetRPM = 10;
			//				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = 10rpm (CMPA = 850)");
			break;
			
		case '2':
			//				stSpinSysCtrl.MOTOR.uRotatingPwmPrd = 1850;
			//				stDMA01.uTargetRPM = 30;
			//				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  2 = 30rpm (CMPA = 1850)");
			break;
			
		case '3':
			//				stSpinSysCtrl.MOTOR.uRotatingPwmPrd = 3350;
			//				stDMA01.uTargetRPM = 60;
			//				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  3 = 60rpm (CMPA = 3350)");
			break;
			
		case '4':
			//				stSpinSysCtrl.MOTOR.uRotatingPwmPrd = 4000;
			//				stDMA01.uTargetRPM = 75;
			//				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  4 = 75rpm (CMPA = 4000(MAX))");
			break;
			
		default :
			if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  Menu Error enter 1~4");
			break;
		}    
	}
	else if(strncmp(_sTemp, "m9 ", 3) == NULL)
	{
		;
	}
	else
	{
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n [Err] Invalid Count!!! Use M1~M6 !!!\r\n");
	}
	
	if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n");
}


//-------------------------------------------------------------
// Function name	: ModeChange
// Description		: ¼öµ¿ ¸ðµå º¯°æ ÇÔ¼ö.
// Parameter		: _sTemp = ¹®ÀÚ¿­
//					  _ucPort = Ãâ·ÂÆ÷Æ®
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void ModeChange(uint8_t _ucPort, char* _sTemp)
{
	if(strlen(_sTemp) == 5)
	{
		//if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n Function %c\r\n", _sTemp[1]);
		switch(_sTemp[4])
		{
		case '0':
			if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  0 = MODE_CLR");
			stSpinSysCtrl.PRIMARY_SEQ.all = CLR;
			break;
			
		case '1':
			if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = MODE_IDLE: SET");
			if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubIdle == CLR)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubIdle = SET;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = MODE_IDLE: SET");
			}
			else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubIdle == SET)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubIdle = CLR;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = MODE_IDLE: CLEAR");
			}
			break;
			
		case '2':
			if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubMaintenance == CLR)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubMaintenance = SET;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  2 = MODE_MAINTENANCE: SET");
			}
			else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubMaintenance == SET)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubMaintenance = CLR;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  2 = MODE_MAINTENANCE: CLEAR");
			}
			break;
			
		case '3':
			if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubRunTest == CLR)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubRunTest = SET;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  3 = MODE_RUNTEST: SET");
			}
			else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubRunTest == SET)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubRunTest = CLR;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  3 = MODE_RUNTEST: CLEAR");
			}
			break;
			
		case '4':
			if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubDebug == CLR)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubDebug = SET;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  4 = MODE_DEBUG: SET");
			}
			else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubDebug == SET)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubDebug = CLR;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  4 = MODE_DEBUG: CLEAR");
			}
			break;
			
		case '5':
			if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control == CLR)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control = SET;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  5 = MODE_PC_CONTROL: SET");
			}
			else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control == SET)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control = CLR;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  5 = MODE_PC_CONTROL: CLEAR");
			}
			break;
			
		case '6':
			if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control == CLR)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control = SET;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  6 = MODE_ROTATING: SET");
			}
			else if(stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control == SET)
			{
				stSpinSysCtrl.PRIMARY_SEQ.bit.ubPC_Control = CLR;
				if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  6 = MODE_ROTATING: CLEAR");
			}
			break;
			
		default :
			if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[Err] Invalid TestFunction Number 1-6 !!!\r\n");
			break;
		}
	}
	else
	{
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[Err] Invalid Count!!! Use MODE1~MODE6 !!!\r\n");
	}
	
	if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n");
}


//-------------------------------------------------------------
// Function name	: DebugCmdProcess
// Description		:
// Parameter		:
// Return		:
//-------------------------------------------------------------
void DebugCmdProcess(Uart_t* _pstUartData)
{
	bool	_bPass_flag = FALSE;
	
	switch(_pstUartData->uPort)
	{    
	case SCI_DEBUG:
		// system
		if(strcmp(_pstUartData->cRX_Buffer, "help?") == NULL)
		{
			DebugMenu(1, SCI_DEBUG);      
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "help!") == NULL)
		{
			DebugMenu(1, SCI_DEBUG);      
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "h") == NULL)
		{
			DebugMenu(1, SCI_DEBUG);      
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "dbg") == NULL)
		{
			if(bDebug_flag)
			{
				if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n Debuging Message View ON");
				bDebug_flag = FALSE;
				Print_flag = FALSE;
			}
			else
			{
				if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n Debuging Message View OFF");
				bDebug_flag = FALSE;
				Print_flag = TRUE;
			}
			
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "ver") == NULL)
		{
			Version();      
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "verf") == NULL)
		{
			Source_File_Version();      
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, " ") == NULL)
		{
			if(stSystemErrorFlag.bEmergency_flag == EMERGENCY)
			{
				stSystemErrorFlag.bEmergency_flag = NORMAL;
				//Init_System();
				if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n Emergency Off");
			}
			else
			{
				stSystemErrorFlag.bEmergency_flag = EMERGENCY;
				if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n Emergency On");
			}      
			_bPass_flag = TRUE;
		}
		
		
		// Mode Change
		else if(strncmp(_pstUartData->cRX_Buffer, "MODE", 4) == NULL)
		{
			ModeChange(SCI_DEBUG, _pstUartData->cRX_Buffer);      
			_bPass_flag = TRUE;
		}
		
		// Test function
		else if(strncmp(_pstUartData->cRX_Buffer, "t", 1) == NULL)
		{
			if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n Test Function run");
			TestFunction(SCI_DEBUG, _pstUartData->cRX_Buffer);      
			_bPass_flag = TRUE;
		}
		
		// Motor test
		else if(strncmp(_pstUartData->cRX_Buffer, "m", 1) == NULL)
		{
			MotorTest(SCI_DEBUG, _pstUartData->cRX_Buffer);      
			_bPass_flag = TRUE;
		}
		
		// ÀüÀå IR Sensor, PI Sensor, Rail Switch, HW connection Á¡°Ë
		else if(strcmp(_pstUartData->cRX_Buffer, "conn") == NULL)
		{      
			_bPass_flag = TRUE;
		}
		
		// ************************************************************************
		// ************ Number Test function 
		// ************************************************************************
		else if(strcmp(_pstUartData->cRX_Buffer, "0") == NULL)
		{  
			User_printf(SCI_DEBUG, "\r\n Number Test[0] : Register status check");
      DebugControlRegisterStatusPrint();      
      _bPass_flag = TRUE;
			
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "1") == NULL)
		{
			User_printf(SCI_DEBUG, "\r\n Number Test[1] : ADC Debug ");
      
  		if(stDbgPrt.bADC == CLR)
			{
				User_printf(SCI_DEBUG, " [Cmd : stDbgPrt.bADC flag set] ");
				stDbgPrt.bADC = SET;
			}
			else
			{
				User_printf(SCI_DEBUG, " [Cmd : stDbgPrt.bADC flag clear] ");
				stDbgPrt.bADC = CLR;
			}
      _bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "2") == NULL)
		{
			User_printf(SCI_DEBUG, "\r\n Number Test[2] : STSPIN32GX_enterStandby();  ");
			DebugControlRegisterStatusPrint();   
      STSPIN32GX_enterStandby();
			//STSPIN32GX_waitForStandby();
      _bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "3") == NULL)
		{
			User_printf(SCI_DEBUG, "\r\n Number Test[3] : STSPIN32GX_wakeup(100);  ");
      STSPIN32GX_wakeup(100);
			DebugControlRegisterStatusPrint();
      _bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "4") == NULL)
		{
			
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "5") == NULL)
		{
			
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "6") == NULL)
		{
			
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "7") == NULL)
		{
			
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "8") == NULL)
		{
			
			_bPass_flag = TRUE;
		}
		else if(strcmp(_pstUartData->cRX_Buffer, "9") == NULL)
		{
			_bPass_flag = TRUE;
		}
		break;
	}
	
	if(!_bPass_flag)
	{
		User_printf(_pstUartData->uPort, "\r\n[Err] Incorrect Command !!!\r\n");
	}
}


//-------------------------------------------------------------
// Function name	: DebugCmdParser
// Description		: µð¹ö±ë ¸í·É ºÐ½Ä±â.
// Parameter		: _pstUartData = Uart ±¸Á¶Ã¼ Æ÷ÀÎÅÍ
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void DebugCmdParser(Uart_t* _pstUartData)
{
	bool	_bCR_flag = FALSE;
	
	if(!_pstUartData->bRX_Buffer_over_flag)
	{
		// ÀÔ·ÂÅ° ÇÊÅÍ
		if((_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] >= '0'  &&
				_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] <= '9') ||
			 (_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] >= 'a'  &&
				_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] <= 'z') ||
				 (_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] >= 'A'  &&
					_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] <= 'Z') ||
					 _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == CAR_RET   ||
						 _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == '\b' ||
							 _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == ' '  ||
								 _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == '.'  ||
									 _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == '?'  ||
										 _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == '!')
		{
			if(_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == CAR_RET)
			{
				_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] = NULL;
				_bCR_flag = TRUE;
			}
			else
			{
				if(_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1] == '\b')	// BackSpace Ã³¸®¿ë
				{
					if(_pstUartData->uRX_Buffer_index > 1)
					{
						User_printf(_pstUartData->uPort, "\b \b");
						_pstUartData->cRX_Buffer[--_pstUartData->uRX_Buffer_index] = NULL;
						_pstUartData->cRX_Buffer[--_pstUartData->uRX_Buffer_index] = NULL;
					}
					else
					{
						_pstUartData->uRX_Buffer_index--;
					}
				}
				else
				{
					Send_char_print(_pstUartData->uPort, _pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index - 1]);
				}
			}
		}
		else if(_pstUartData->uRX_Buffer_index != 0)
		{
			_pstUartData->uRX_Buffer_index--;
		}
		
		if(_bCR_flag)
		{
			if(_pstUartData->uRX_Buffer_index >= 2)
			{
				DebugCmdProcess(_pstUartData);
			}
			_pstUartData->uRX_Buffer_index = 0;
			_pstUartData->cRX_Buffer[_pstUartData->uRX_Buffer_index] = NULL;
			MACHINE_NAME(_pstUartData->uPort);
		}
	}
	else
	{
		_pstUartData->bRX_Buffer_over_flag = FALSE;
		User_printf(_pstUartData->uPort, "\r\n[Err] DEBUG SCI BUffer Over !!!\r\n");
		MACHINE_NAME(_pstUartData->uPort);
	}
}


//-------------------------------------------------------------
// Function name	: Version
// Description		: FW ¹öÀü Ãâ·Â.
// Parameter		: ¾øÀ½.
// Return			: ¾øÀ½.
//-------------------------------------------------------------
void Version(void)
{	
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n=========================================");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n** EVSPIN32G4 Development Environments ** ");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n EVSPIN32G4 FW Version \t: %u.%u.%u.%u.%u.%u", FW_VERSION_MAJOR1, FW_VERSION_MAJOR2, FW_VERSION_MINOR1, FW_VERSION_MINOR2, FW_VERSION_TEST1, FW_VERSION_TEST2);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n Project FW Version    \t: %s", PROJECT_VERSION);
	//	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n DCM Protocol Version  : %c", PROTOCOL_T1_VERSION);
	//	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n EXT Protocol Version  : %c", PROTOCOL_DCM_VERSION);
	//	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n APP Protocol Version : %c", PROTOCOL_APP_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n PCB Version           \t: %u.%u", EVSPIN32G4_PCB_VERSION_MAJOR, EVSPIN32G4_PCB_VERSION_MINOR);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n=========================================");
	
	Source_File_Version();
}


//-------------------------------------------------------------
// Function name	: Source_File_Version
// Description		:
// Parameter		:
// Return		:
//-------------------------------------------------------------
void Source_File_Version(void)
{
	//GetPcbVersion();
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n-----------------------------------------");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n System EVSPIN32G4 FW Code Version");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n-----------------------------------------");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t: %s", __SPIN32G4_PARAMETERS_H,	SPIN32G4_PARAMETERS_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t: %s", __USER_PREDEFINE_H,	USER_PREDEFINE_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n-----------------------------------------");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t\t: %s", __USER_UTIL_H,		USER_UTIL_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t\t: %s", __USER_UART_H,		USER_UART_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t\t: %s", __USER_DEBUG_H,	USER_DEBUG_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t\t: %s", __USER_PERIODIC_H,	USER_PERIODIC_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n-----------------------------------------");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t\t: %s", __STSPIN32G4_I2C_H,	STSPIN32G4_I2C_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n %s \t: %s", __STSPIN32G4_REGISTER_CONFIG_H,	STSPIN32G4_REGISTER_CONFIG_VERSION);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n-----------------------------------------");
}


//-------------------------------------------------------------
// Function name	: DebugMenu
// Description		: 
// Parameter		: _uOption = 
//			: _ucPort =
// Return		:
//-------------------------------------------------------------
void DebugMenu(uint8_t _ucOption, uint8_t _ucPort)
{
	if(stDbgPrt.bDebug) 
	{
		User_printf(_ucPort, "\r\n======================HELP======================");
	}
	
	if(_ucOption == 1)
	{
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n-------------------- SYSTEM --------------------");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[help?]    : HELP Debug Command View");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[help!]    : HELP Motor Test Command View");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[reset]    : System Watchdog Reset");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[dbg]      : Debuging Message View ON/OFF");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[bfs]      : Boot flag Set/Clear");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[ver]      : Version Infomation");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[SPACE]    : Emergency ON/OFF");
		EMPTY_LINE(_ucPort);
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n---------------- TEST FUNCTION -----------------");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[t_] : Test Function, _ = Function no.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  0 = STSPIN32G4 Gate Driver register configuration check.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = ADC debugging print toggle.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  2 = STSPIN32GX_enterStandby");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  3 = STSPIN32GX_wakeup.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  4 = TIM16 Motor Brake test.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  5 = None.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  6 = None.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  7 = None.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  8 = None.");
    if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  9 = None.");
    EMPTY_LINE(_ucPort);
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n--------------- SYSTEM MODE CHANGE ---------------");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[MODE_] : Mode Change Function, _ = Mode no.");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  0 = MODE_CLEAR		");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = MODE_IDLE		");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  2 = MODE_MAINTENANCE");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  3 = MODE_RUNTEST    ");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  4 = MODE_DEBUG      ");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  5 = MODE_PC_CONTROL ");
		EMPTY_LINE(_ucPort);
	}
	else if(_ucOption == 2)
	{
		EMPTY_LINE(_ucPort);
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n---------------- MOTOR ----------------");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n[m_] : Test Function, _ = Function no.");
		if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n  1 = DMA01   : Rotating Motor Run/Stop");
		if(stDbgPrt.bDebug) EMPTY_LINE(_ucPort);
	}
	if(stDbgPrt.bDebug) User_printf(_ucPort, "\r\n================================================\r\n");
}


//-------------------------------------------------------------
// Function name	: DebugWelcomeMessage
// Description		: 
// Parameter		: 
// Return		: 
//-------------------------------------------------------------
void DebugWelcomeMessage(void)
{
	if(stDbgPrt.bDebug) 
	{
		EMPTY_LINE(SCI_DEBUG);
		EMPTY_LINE(SCI_DEBUG);
		EMPTY_LINE(SCI_DEBUG);
		EMPTY_LINE(SCI_DEBUG);
		
		User_printf(SCI_DEBUG, "\r\n----------------------------------------");
		User_printf(SCI_DEBUG, "\r\n Welcome to EVSPIN32G4!!!!!!            ");
		User_printf(SCI_DEBUG, "\r\n System boot complete....               ");  
		User_printf(SCI_DEBUG, "\r\n Staring firmware........               ");
		Version();
		
		EMPTY_LINE(SCI_DEBUG);
		User_printf(SCI_DEBUG, "\r\n------------------HELP------------------");
		User_printf(SCI_DEBUG, "\r\n If you want to see the menu, enter 'h' ");
		User_printf(SCI_DEBUG, "\r\n----------------------------------------"); //if(stDbgPrt.bDebug) 
		MACHINE_NAME(SCI_DEBUG);
	}
}


//-------------------------------------------------------------
// Function name	: DebugControlRegisterStatusPrint
// Description		: 
// Parameter		:
// Return		:
//-------------------------------------------------------------
void DebugControlRegisterStatusPrint(void)
{
	CheckAllGateDriverRegisters();
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n ----------- STSPIN32G4 Control Register condition -----------");
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x01] Power manager configuration: %x", 		stSpinReg.POWMNG.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - REG3V3   = %d (0: Internal 3.3V, 1: External 3.3V) ", 	stSpinReg.POWMNG.cValue.bit.ubREG3V3_DIS);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VCC      = %d (0: Internal VCC, 1: External VCC) ", 	stSpinReg.POWMNG.cValue.bit.ubVCC_DIS);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - STBY_REG = %d (0: disable, 1: Enable)", 		stSpinReg.POWMNG.cValue.bit.ubSTBY_REG_EN);     
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VCC_VAL  = %d (0: 8V, 1: 10V, 2: 12V, 3: 15V)", 	stSpinReg.POWMNG.cValue.bit.ubVCC_VAL);
	EMPTY_LINE(SCI_DEBUG);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x02] Driving logic configuration: %x", 		stSpinReg.LOGIC.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VDS_P_DEG = %d (0: 6us, 1: 4us, 2: 3us, 3: 2us)", 	stSpinReg.LOGIC.cValue.bit.ubVDS_P_DEG);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - DTMIN     = %d (0: Disable, 1: Enable(100~200ns)", 	stSpinReg.LOGIC.cValue.bit.ubDTMIN);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - ILOCK     = %d (0: Disable, 1: Enable)", 		stSpinReg.LOGIC.cValue.bit.ubILOCK);  
	EMPTY_LINE(SCI_DEBUG);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x07] READY output configuration: %x", 		stSpinReg.READY.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - STBY_RDY     = %d (0: Disable, 1:Enable)", 		stSpinReg.READY.cValue.bit.ubSTBY_RDY);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - THSD_RDY     = %d (0: Disable, 1:Enable)", 		stSpinReg.READY.cValue.bit.ubTHSD_RDY);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VCC_UVLO_RDY = %d (0: Disable, 1:Enable)", 		stSpinReg.READY.cValue.bit.ubVCC_UVLO_RDY);  
	EMPTY_LINE(SCI_DEBUG);   
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x08] nFAULT output configuration: %x", 		stSpinReg.FAULT.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - RESET_FLT    = %d (0: Disable, 1:Enable)", 		stSpinReg.FAULT.cValue.bit.ubRESET_FLT);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VDS_P_FLT    = %d (0: Disable, 1:Enable)", 		stSpinReg.FAULT.cValue.bit.ubVDS_P_FLT);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - THSD _FLT    = %d (0: Disable, 1:Enable)", 		stSpinReg.FAULT.cValue.bit.ubTHSD_FLT);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VCC_UVLO_FLT = %d (0: Disable, 1:Enable)", 		stSpinReg.FAULT.cValue.bit.ubVCC_UVLO_FLT);
	EMPTY_LINE(SCI_DEBUG);     
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x09] Command register: Fault clear: %x", 		stSpinReg.CLEAR.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - FAULT_CLEAR (clear command is 0xFF)");     
	EMPTY_LINE(SCI_DEBUG);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x0A] Standby: %x", 				stSpinReg.STBY.cValue.all);   
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - STBY = %d (0: normal, 1: Standby)", 			stSpinReg.STBY.cValue.bit.ubSTBY);  
	EMPTY_LINE(SCI_DEBUG);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x0B] Lock:  %x", 					stSpinReg.LOCK.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - NLOCK = %x : ", 					stSpinReg.LOCK.cValue.bit.ubNLOCK);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - LOCK = %x : ", 						stSpinReg.LOCK.cValue.bit.ubLOCK);
	EMPTY_LINE(SCI_DEBUG);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x0C] Command register: RESET: %x",			stSpinReg.RST.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - RESET (reset command is 0xFF) ");      
	EMPTY_LINE(SCI_DEBUG);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n [0x80] Reports the device status: %x", 		stSpinReg.STATUS.cValue.all);
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - LOCK      = %d (0: Unlocked, 1:Locked)", 		stSpinReg.STATUS.cValue.bit.ubLOCK);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - RESET     = %d (0: No reset, 1: Reset)", 		stSpinReg.STATUS.cValue.bit.ubRESET);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VDS_P     = %d (0: Not triggered, 1: Triggered)", 	stSpinReg.STATUS.cValue.bit.ubVDS_P);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - THSD      = %d (0: Not triggered, 1: Triggered)", 	stSpinReg.STATUS.cValue.bit.ubTHSD);  
	if(stDbgPrt.bDebug) User_printf(SCI_DEBUG, "\r\n    - VCC_UVLO  = %d (0: Above Threshold, 1: Under Threshold)",stSpinReg.STATUS.cValue.bit.ubVCC_UVLO);  
	EMPTY_LINE(SCI_DEBUG);
	EMPTY_LINE(SCI_DEBUG);
	MACHINE_NAME(SCI_DEBUG);
}

/** =========================================================================**/
/** ============================  for Test Function ======================== **/
/** =========================================================================**/

/* Timer trigger and ADC data acquisition variables */
uint8_t	guc_ADC1_READ_NUM = ADC1_READ_NUM;
uint16_t gush_DMA_Trg1_MEM[ADC1_READ_NUM] = {0,};
uint16_t gush_DMA_ADC1_MEM[ADC1_READ_NUM] = {0,};
uint16_t gush_DMA_ADC1_MEM_mV[ADC1_READ_NUM] = {0,};
uint16_t gush_DMA_ADC2_MEM[ADC2_READ_NUM] = {0,};
uint16_t gush_DMA_ADC2_MEM_mV[ADC2_READ_NUM] = {0,};
uint16_t gush_INJ_ADC2_MEM = 0;
uint16_t gush_INJ_ADC2_MEM_mV = 0;
uint8_t	guc_ADC2_RegConvCount = 0;
uint8_t	guc_ADC2_RegConvReadyFlag = NO;
uint16_t gush_PWM_period = 500;

uint16_t gush_VDDA_mVolt = 0;

float gf_Kcompen = 0;
uint32_t gul_Kcompen_shift16bit = 0;
uint16_t gush_VREFINT_CAL = 0;
uint16_t gush_VREFINT_CAL_mVolt = 0;
uint16_t gush_VREFINT_READ = 0;

uint8_t	guc_Boot_Flag = 0;   
uint16_t gush_First_TRG = 0;       
uint16_t gush_ADC_VbusData = 0;  

uint16_t guc_Blackout_Flag = 0;  
uint16_t gush_ADC_Blackout = 0;  

uint16_t gush_ADtemp1 = 0;  
uint32_t gul_Offset_a = 0;  
int32_t gssh_Ias0 = 0;  

void Init_AdcVariables(void)
{
	/* Trigger Timing Initialization – for operating test */
	gush_DMA_Trg1_MEM[0] = 1000; // 1-shunt currnet a
	gush_DMA_Trg1_MEM[1] = 2000; // 1-shunt current b
	gush_DMA_Trg1_MEM[2] = TIM1->ARR; // reserved 1
	gush_DMA_Trg1_MEM[3] = 2500; // reserved 2
	gush_DMA_Trg1_MEM[4] = 2000; // reserved 3
	gush_DMA_Trg1_MEM[5] = 0; // Vdc
}

void MotorStart(void)
{
	Init_AdcVariables();
	
	/* ADC Conversion stop sequence */
	LL_ADC_REG_StopConversion(ADC1);
	while ((ADC1->CR & ADC_CR_ADSTP) != 0){}
	if (LL_ADC_IsEnabled(ADC1)){
		LL_ADC_Disable(ADC1);
		while ((ADC1->CR & ADC_CR_ADEN) != 0){}
	}
	
	/* ADC data acquisition DMA configurations */
	//ADC1->CHSELR = 0x0F065110; // 1st :CH0, 2nd :CH1, 3rd :CH1, 4th :CH5, 5th :CH6, 6th :CH0
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISINGFALLING);
	
	/* DMA configuration for ADC Data acquisition */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)gush_DMA_ADC1_MEM);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) & (ADC1->DR));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC1_READ_NUM);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	// Enable ADC for ADC Data acquisition
	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0){}
	/* DMA configuration for Timer triger output */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)gush_DMA_Trg1_MEM);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) & (TIM1->CCR4));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ADC1_READ_NUM);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	LL_TIM_EnableDMAReq_CC4(TIM1);
	/* Set the all DMA count number to initial value */
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC1_READ_NUM);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ADC1_READ_NUM);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* ADC conversion start: External trigger ADC need to start conversion before triggering */
	LL_ADC_REG_StartConversion(ADC1);
	/* Timer1 start sequence */
	LL_TIM_OC_SetCompareCH1(TIM1, gush_PWM_period);
	LL_TIM_OC_SetCompareCH2(TIM1, gush_PWM_period);
	LL_TIM_OC_SetCompareCH3(TIM1, gush_PWM_period);
	LL_TIM_OC_SetCompareCH4(TIM1, 300); // must set below 1shunt sensing trigger timing
	//LL_TIM_CC_EnableChannel_Sub(); // TIM1 CH1,CH1N, CH2,CH2N, CH3,CH3N, CH4
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetCounter(TIM1, 0); // TIM1->CNT = 0;
	LL_TIM_EnableAllOutputs(TIM1); // TIM1->BDTR, TIM_BDTR_MOE
	LL_TIM_EnableCounter(TIM1); // TIM1 Start
	LL_TIM_EnableIT_UPDATE(TIM1); // TIM1 Update Interrupt
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
}

void MotorStop(void)
{
	/* Timer1 stop sequence */
	LL_TIM_DisableIT_UPDATE(TIM1);
	LL_TIM_DisableCounter(TIM1);
	LL_TIM_SetCounter(TIM1, 0);
	LL_TIM_DisableAllOutputs(TIM1);
	LL_TIM_DisableDMAReq_CC4(TIM1);
	//LL_TIM_CC_DisableChannel_Sub();	
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);	
	/* ADC Conversion stop sequence */
	LL_ADC_REG_StopConversion(ADC1);
	while ((ADC1->CR & ADC_CR_ADSTP) != 0){}
	if (LL_ADC_IsEnabled(ADC1)){
		LL_ADC_Disable(ADC1);
		while ((ADC1->CR & ADC_CR_ADEN) != 0){}
	}
	/* Disable all DMA channels */
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* Additional configuration for Timer1 Direction initialize */
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
	if(LL_TIM_GetDirection(TIM1))
	{
		TIM1->CR1 &= ~(TIM_CR1_DIR);
	}
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_CENTER_UP_DOWN);
	/* Reinitialize the Timer counter and generates an update of the registers. */
	TIM1->EGR |= TIM_EGR_UG;
}


void ADC2_Init(void)
{
	
	/** ADC Configuration **/
	/* ADC Start from Software trigger*/
	//LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	
	/* ADC Sequencer configuration*/
	//LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE); // G0
	LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_8);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_10);
	
	/* DMA disable before finishing ADC & DMA configurations */
	LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_NONE); 
	
	/* Enable ADC interrupt  
	LL_ADC_ClearFlag_EOC(ADC1);
	LL_ADC_EnableIT_EOC(ADC1);	
	LL_ADC_ClearFlag_EOS(ADC1);
	LL_ADC_EnableIT_EOS(ADC1);
	*/
	
	/* ADC Continuous mode setting
	1. Single conversion mode (CONT = 0): LL_ADC_REG_CONV_SINGLE
	- one conversion per trigger
	2. Continuous conversion mode (CONT = 1): LL_ADC_REG_CONV_CONTINUOUS
	- after the first trigger, following conversions launched successively automatically
	*/
	LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
	
	/* ADC Discontinuous mode setting
	This mode is enabled by setting the DISCEN bit in the ADC_CFGR1 register.
	In this mode (DISCEN = 1), a hardware or software trigger event is required 
	to start each conversion defined in the sequence.
	
	@arg @ref LL_ADC_REG_SEQ_DISCONT_DISABLE (DISCEN = 0)
	@arg @ref LL_ADC_REG_SEQ_DISCONT_1RANK (DISCEN = 1)
	*/
	//LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
	LL_ADC_REG_SetSequencerDiscont(ADC2, LL_ADC_REG_SEQ_DISCONT_3RANKS);
	
	/* ADC Enable */
	LL_ADC_ClearFlag_ADRDY(ADC2);
	LL_ADC_Enable(ADC2);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0){}
	
	
	/** DMA Configuration for ADC DataReg read **/	
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)(gush_DMA_ADC2_MEM) );
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)(&(ADC2->DR)) );
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ADC2_READ_NUM); 	
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
	
	/* In circular mode:
	After the last data transfer, the DMA_CNDTRx register is
	automatically reloaded with the initially programmed value. 
	The current internal address registers are reloaded with the base address 
	values from the DMA_CPARx and DMA_CMARx registers.
	
	If the channel x is configured in non-circular mode(DMA_CCR_CIRC = 0), 
	no DMA request is served after the last data transfer
	(once the number of single data to transfer reaches zero). 
	The DMA channel must be disabled in order to reload a new number of 
	data items into the DMA_CNDTRx register.
	
	@arg @ref LL_DMA_MODE_NORMAL (DMA_CCR_CIRC = 0)
	@arg @ref LL_DMA_MODE_CIRCULAR (DMA_CCR_CIRC = 1)
	*/
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);
	
	/** ADC Configuration **/
	/* Enable ADC DMA request & Limited mode:
	1. ADC conversions are transferred by DMA or not
	- ADC_CFGR1_DMAEN = 0 (ADC conversions are not transferred)
	- ADC_CFGR1_DMAEN = 1 (ADC conversions are transferred)
	2. ADC conversion data are transferred by DMA, in limited mode (one shot mode): ADC_CFGR1_DMAEN = 0
	- DMA transfer requests are stopped when number of DMA data transfers (number of ADC conversions) is reached. 
	- This ADC mode is intended to be used with DMA mode non-circular. 
	3. ADC conversion data are transferred by DMA, in unlimited mode: ADC_CFGR1_DMAEN = 1
	- DMA transfer requests are unlimited, whatever number of DMA data transferred (number of ADC conversions). 
	- This ADC mode is intended to be used with DMA mode circular. 
	
	@arg @ref LL_ADC_REG_DMA_TRANSFER_NONE       (0x00000000UL)
	@arg @ref LL_ADC_REG_DMA_TRANSFER_LIMITED    (                   ADC_CFGR1_DMAEN)
	@arg @ref LL_ADC_REG_DMA_TRANSFER_UNLIMITED  (ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN)  
	*/
	//LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_NONE); 
	LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_UNLIMITED); 
	
	/** DMA Configuration **/
	/* Enable DMA interrupt  */
	LL_DMA_ClearFlag_HT2(DMA1);
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_ClearFlag_TC2(DMA1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
	
	/* Enable DMA Channel  */
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
	
	guc_ADC2_RegConvReadyFlag = YES;
	
	/** Injected ADC configurations **/
	LL_ADC_EnableIT_JEOC(ADC2);
	LL_ADC_INJ_StartConversion(ADC2);
	
	
	/** Timer8 start sequence **/
	//LL_TIM_OC_SetCompareCH4(TIM8, (TIM8->CCR1)>>1 ); // must set below 1shunt sensing trigger timing
	//LL_TIM_CC_EnableChannel_Sub(); // TIM1 CH1,CH1N, CH2,CH2N, CH3,CH3N, CH4
	TIM8->ARR = 8000;
	TIM8->CCR1 = 4000;
	TIM8->CCR4 = 2000;
	LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetCounter(TIM8, 0); // TIM1->CNT = 0;
	LL_TIM_EnableAllOutputs(TIM8); // TIM1->BDTR, TIM_BDTR_MOE
	LL_TIM_EnableCounter(TIM8); // TIM1 Start
	//LL_TIM_EnableIT_UPDATE(TIM8); // TIM1 Update Interrupt
	//LL_TIM_EnableIT_CC1(TIM8); // TIM1 Update Interrupt
	//LL_TIM_EnableIT_CC4(TIM8); // TIM1 Update Interrupt
}

void ADC2_Regular_SWConversionStart(void)
{
	LL_ADC_REG_StartConversion(ADC2);
}

void ADC2_Injected_SWConversionStart(void)
{
	;
}


/*********************************************************************************
* End of File
*********************************************************************************/

