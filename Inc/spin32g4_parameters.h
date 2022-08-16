/* ###################################################################
**  Filename		: spin32g4_parameters.h
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
#ifndef __SPIN32G4_PARAMETERS_H
#define __SPIN32G4_PARAMETERS_H "SPIN32G4_PARAMETERS"

#define	SPIN32G4_PARAMETERS_VERSION		"0.0.1"		// Reserved.Release.Debug

/************************* Project informatino define *************************/
#define PROJECT_VERSION		"0.0.1"

#define FW_VERSION_MAJOR1	0
#define FW_VERSION_MAJOR2	0
#define FW_VERSION_MINOR1	0
#define FW_VERSION_MINOR2       1
#define FW_VERSION_TEST1        0
#define FW_VERSION_TEST2        1

#define PROTOCOL_MAIN_VERSION   'A'	
#define PROTOCOL_DCM_VERSION    'A'
#define PROTOCOL_ROBOT_VERSION	'A'

#define EVSPIN32G4_PCB_VERSION_MAJOR 1
#define EVSPIN32G4_PCB_VERSION_MINOR 0

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ             170000000uL
#define TIM_CLOCK_DIVIDER       1
#define ADV_TIM_CLK_MHz         170
#define ADC_CLK_MHz             42
#define HALL_TIM_CLK            170000000uL
#define APB1TIM_FREQ            170000000uL
#define REF_TIM_CLK             170000000uL
#define REF_TIM_CLK_MHz         170

/*************** Timer for PWM generation & currenst sensing parameters  ******/
/* Timer for PWM Frequency */
#define PWM_FREQUENCY           30000
#define PWM_FREQ_SCALING        1
#define PWM_PERIOD_CYCLES       (uint16_t)((ADV_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))&0xFFFE)

/* Timer dead time insertion */
#define SW_DEADTIME_NS          800 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */
#define DEADTIME_NS             SW_DEADTIME_NS
#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1      (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
  #define DEAD_TIME_COUNTS      (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
  #define DEAD_TIME_COUNTS      (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
  #define DEAD_TIME_COUNTS      (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
  #define DEAD_TIME_COUNTS      (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
  #define DEAD_TIME_COUNTS 510
#endif

/* Timer repeatition counter */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                  number of PWM cycles */
#define REP_COUNTER             (uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define SYS_TICK_FREQUENCY      2000
#define SYSTICK_DIVIDER         (SYS_TICK_FREQUENCY/1000)

#endif /*__SPIN32G4_PARAMETERS_H*/
/*********************************************************************************
* End of File
*********************************************************************************/
