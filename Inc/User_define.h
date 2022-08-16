/* ###################################################################
**  Filename		: User_define.h
**  Title		:
**  Project		: 
**  Processor		: 
**  ToolChain		: 
**  Compiler		: 
**  Date/Time		: 2022.01.27
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.1 / 2020.03.27 / skkim
begin
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DEFINE_H
#define __USER_DEFINE_H

/************************* Project informatino define *************************/
#define PROJECT_VERSION		"0.0.1"

#define FW_VERSION_MAJOR1	0
#define FW_VERSION_MAJOR2	0
#define FW_VERSION_MINOR1	0
#define FW_VERSION_MINOR2       1
#define FW_VERSION_TEST1        0
#define FW_VERSION_TEST2        0

#define PROTOCOL_MAIN_VERSION   'A'	
#define PROTOCOL_DCM_VERSION    'A'
#define PROTOCOL_ROBOT_VERSION	'A'

/************************* Normal define **************************************/
#define NONE			0
#define READ			0
#define WRITE			1

#define FALSE			0
#define TRUE			1

#define LOW			0
#define HIGH			1

#define NO			0
#define YES			1

#define OFF			0
#define ON			1

#define STOP			0
#define RUN			1
#define START			1

#define OPEN			0
#define CLOSE                   1
#define RUNNING                 2

#define CLR			0	// Clear

#define FW  			0
#define BW	    		1

#define INACTIVE    		0
#define ACTIVE			1

#define NORMAL			0
#define EMERGENCY		1

#define ACK			1
#define NAK			2
#define	WAIT			3

#endif /*__USER_DEFINE_H*/
/*********************************************************************************
* End of File
*********************************************************************************/
