/* ###################################################################
**  Filename		: User_predefine.h
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
#ifndef __USER_PREDEFINE_H
#define __USER_PREDEFINE_H "USER_PREDEFINE"

#define	USER_PREDEFINE_VERSION		"0.0.1"		// Reserved.Release.Debug

/************************* Normal define **************************************/
#define NONE			0
#define READ			0
#define WRITE			1

#define FALSE			0
#define TRUE			1

#define LOW			0
#define HIGH			1

//#define RESET			0
//#define SET			1

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

#endif /*__USER_PREDEFINE_H*/
/*********************************************************************************
* End of File
*********************************************************************************/
