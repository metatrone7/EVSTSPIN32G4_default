/* ###################################################################
**  Filename		: User_debug.h
**  Title		: Header file of User_debug.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DEBUG_H
#define __USER_DEBUG_H	"USER_DEBUG"

#define	USER_DEBUG_VERSION		"0.0.1"		// Reserved.Release.Debug

/*********************************************************************************
* Includes
*********************************************************************************/
#include "User_uart.h"
#include "User_SystemControl.h"

/*********************************************************************************
* Defines & Variables
*********************************************************************************/

// RUN Mode
#define ACTUATOR_TEST_MODE_IDLE		0
#define ACTUATOR_TEST_MODE_EMERGENCY	1
#define ACTUATOR_TEST_MODE_AUTO		2
#define ACTUATOR_TEST_MODE_ONESHOT	3
#define ACTUATOR_TEST_MODE_SERISE	4
#define ACTUATOR_TEST_MODE_MANUAL	5

// AUTO Mode
#define ACTUATOR_TEST_AUTO_CROSS	2	// 목적지 1회씩 교차 (리젝 <-> 카세트)
#define ACTUATOR_TEST_AUTO_LOOP		3	// 무한 루프


typedef struct
{
  bool	bDebug;  
  bool	bADC;  
//  bool	bFwStatus;  
//  bool	bScFile;
//  bool	bScRunTest;  
//  bool	bXintFile;  
//  bool	bProtocol;  
//  bool	bEmergency;
//  bool	bSC_M1;  
//  bool	bSpi;
//  bool	bEPC;
//  bool	bTest;
//  bool	bResult;
} Debug_Print_t;


/*********************************************************************************
* Extern Variables
*********************************************************************************/
extern bool	        bDebug_flag;
extern Debug_Print_t    stDbgPrt;

extern bool	bSystemInitializeFinishFlag;
extern bool	bReadyPinExternalInterruptFlag;
extern bool	bReadyPinInput;
extern bool	bFaultPinExternalInterruptFlag;
extern bool	bFaultPinInput;


/*********************************************************************************
* Extern Functions
*********************************************************************************/
extern void Init_DebugFlag(void);

extern void Version(void);
extern void DebugMenu(uint8_t _ucOption, uint8_t _ucPort);
extern void DebugCmdParser(Uart_t* _pstUartData);
extern void DebugWelcomeMessage(void);
extern void DebugControlRegisterStatusPrint(void);

/** =========================================================================**/
/** ============================  for Test Function ======================== **/
/** =========================================================================**/
/* Timer trigger and ADC data acquisition variables */
#define GPA8_ON         WRITE_REG(GPIOA->BSRR, LL_GPIO_PIN_8);
#define GPA8_OFF        WRITE_REG(GPIOA->BRR, LL_GPIO_PIN_8);
#define GPA11_ON        WRITE_REG(GPIOA->BSRR, LL_GPIO_PIN_11);
#define GPA11_OFF       WRITE_REG(GPIOA->BRR, LL_GPIO_PIN_11);	
#define GPA12_ON        WRITE_REG(GPIOA->BSRR, LL_GPIO_PIN_12);
#define GPA12_OFF       WRITE_REG(GPIOA->BRR, LL_GPIO_PIN_12);

#define ADC1_READ_NUM    			6
#define ADC2_READ_NUM    			3
#define VDDA_3000       			(3000UL)
#define OFFSET_CNT						10
#define VREFPLUS_CHARAC				3000 // [mV] refer to the G0 datasheet
#define VDDA_FROM_REGULATOR		3300 // [mV] refer to the G0 datasheet

extern uint8_t	guc_ADC1_READ_NUM;   
extern uint16_t gush_DMA_Trg1_MEM[];
extern uint16_t gush_DMA_ADC1_MEM[];
extern uint16_t gush_DMA_ADC1_MEM_mV[];
extern uint16_t gush_DMA_ADC2_MEM[];
extern uint16_t gush_DMA_ADC2_MEM_mV[];
extern uint16_t gush_INJ_ADC2_MEM;
extern uint16_t gush_INJ_ADC2_MEM_mV;
extern uint8_t	guc_ADC2_RegConvCount;
extern uint8_t	guc_ADC2_RegConvReadyFlag;
extern uint16_t gush_PWM_period;

extern uint16_t gush_VDDA_mVolt;

extern float gf_Kcompen;
extern uint32_t gul_Kcompen_shift16bit;
extern uint16_t gush_VREFINT_CAL;
extern uint16_t gush_VREFINT_CAL_mVolt;
extern uint16_t gush_VREFINT_READ;

extern uint8_t	guc_Boot_Flag;
extern uint16_t gush_First_TRG;
extern uint16_t gush_ADC_VbusData;

extern uint16_t guc_Blackout_Flag;
extern uint16_t gush_ADC_Blackout;

extern uint16_t gush_ADtemp1;
extern uint32_t gul_Offset_a;
extern int32_t gssh_Ias0;

extern void MotorStart(void);
extern void MotorStop(void);

extern void ADC2_Init(void);
extern void ADC2_Regular_SWConversionStart(void);
extern void ADC2_Injected_SWConversionStart(void);

#endif /* USER_DEBUG_H_ */
/*********************************************************************************
* End of File
*********************************************************************************/

