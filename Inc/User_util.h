/* ###################################################################
**  Filename	    	: User_util.h
**  Title		: Header file of User_util.c
**  Project		: General purpose
**  Processor		: General purpose
**  ToolChain		: IAR Embedded Workbench IDE - Arm 9.10.2
**  Compiler		: IAR C/C++ Compiler for ARM  9.10.2.313 
**  Date/Time		: 2022.02.04
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.1 / 2022.02.04 / skkim
begin
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_UTIL_H
#define __USER_UTIL_H	"USER_UTIL"

#define	USER_UTIL_VERSION		"0.0.1"		// Reserved.Release.Debug

/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

/*********************************************************************************
* Defines
*********************************************************************************/
#define NUMBER_ASCII_CODE		0x30
#define ALPHABET_ASCII_CODE		0x57

/*********************************************************************************
* Extern Variables
*********************************************************************************/



/*********************************************************************************
* Extern Functions
*********************************************************************************/
extern bool GetAbitfromuint16_t(uint16_t x, int n);
extern uint16_t Character_search(char* _String, char _Character);
extern uint16_t String_compare(char* _String1, char* _String2);
extern uint16_t String_length(char* _String);
extern uint16_t Strncpy(char* _String1, char* _String2, uint16_t _uFrom, uint16_t _uTo);

extern void User_itoa(int32_t _ilValue, char* _String);
extern void User_ltoa(int64_t _inValue, char* _String);
extern void User_lltoa(uint64_t _unValue, char* _String);
extern void User_ftoa(float _fValue, char* _String);
extern uint16_t User_atoi(char* _String);
extern uint32_t User_atol(char* _String);
extern uint64_t User_atoll(char* _String);

extern uint16_t Value_Search(uint16_t _uSize, uint16_t _uTarget_value, uint16_t* _puTarget_array_value);
extern uint16_t Binary_search(uint16_t _uTail, uint16_t _uTarget_value, uint16_t* _puTarget_array_value);

extern uint32_t User_pow(uint16_t _uValue, uint16_t _uNum);
extern uint16_t Array_average(uint16_t* _puArray, uint16_t _uSize);
extern uint16_t Curve(uint16_t _X_min, uint16_t _X_max, uint16_t _Y_min, uint16_t _Y_max, uint16_t _X_value);


#endif
/*********************************************************************************
* End of File
*********************************************************************************/

