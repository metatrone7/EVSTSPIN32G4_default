/* ###################################################################
**  Filename		: User_periodic.c
**  Title		: User's periodic function can be added between 
**                        user code begin and end in this file.
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

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "User_periodic.h"
#include "User_debug.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
bool	bTimer_1ms_flag = FALSE;
bool	bTimer_10ms_flag = FALSE;
bool	bTimer_100ms_flag = FALSE;
bool	bTimer_500ms_flag = FALSE;
bool	bTimer_1000ms_flag = FALSE;
bool	bTimer_5000ms_flag = FALSE;

bool	bAlarmLedToggleEnableFlag = FALSE;

volatile uint16_t uTimer_1ms_count = 0;
volatile uint16_t uTimer_10ms_count = 0;
volatile uint16_t uTimer_100ms_count = 0;
volatile uint16_t uTimer_500ms_count = 0;
volatile uint16_t uTimer_1000ms_count = 0;

/* Private function prototypes -----------------------------------------------*/
void RunPeriodicTimerCounter(void);
void PeriodicFunction_10msec(void);
void PeriodicFunction_100msec(void);
void PeriodicFunction_500msec(void);
void PeriodicFunction_1000msec(void);
void PeriodicFunction_5000msec(void);
void PeriodicFunction(void);

/* Private user code ---------------------------------------------------------*/
//-------------------------------------------------------------
// Function name	: RunPeriodicTimerCounter
// Description		: Periodic timer counter and flag update
//                        Please insert 'RunPeriodicTimerCounter()' to systick timer ISR
//                        systick timer ISR is existing in 'stm32g4xx_it.c'
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void RunPeriodicTimerCounter(void)
{  
  bTimer_1ms_flag = TRUE;
  if(++uTimer_1ms_count >= 10) // 10msec
  {
    uTimer_1ms_count = 0;
    bTimer_10ms_flag = TRUE;
    
    if(++uTimer_10ms_count >= 10) // 100msec
    {
      uTimer_10ms_count = 0;
      bTimer_100ms_flag = TRUE;
      
      if(++uTimer_100ms_count >= 5) // 500msec
      {
        uTimer_100ms_count = 0;
        bTimer_500ms_flag = TRUE;
        
        if(++uTimer_500ms_count >= 2) // 1000msec
        {
          uTimer_500ms_count = 0;
          bTimer_1000ms_flag = TRUE;
        
          if(++uTimer_1000ms_count >= 5) // 5000msec
          {
            uTimer_1000ms_count = 0;
            bTimer_5000ms_flag = TRUE;
          }        
        }        
      }      
    }
  }
}


//-------------------------------------------------------------
// Function name	: PeriodicFunction_1msec
// Description		: Periodic function for 1msec
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction_1msec(void)
{  
  bTimer_1ms_flag = FALSE;
/* USER CODE BEGIN 0 */
	if(guc_ADC2_RegConvReadyFlag == YES) 
	{
		ADC2_Regular_SWConversionStart();
	}

/* USER CODE END 0 */
}

//-------------------------------------------------------------
// Function name	: PeriodicFunction_10msec
// Description		: Periodic function for 10msec
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction_10msec(void)
{  
  bTimer_10ms_flag = FALSE;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
}

//-------------------------------------------------------------
// Function name	: PeriodicFunction_100msec
// Description		: Periodic function for 100msec
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction_100msec(void)
{  
  bTimer_100ms_flag = FALSE;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
}


//-------------------------------------------------------------
// Function name	: PeriodicFunction_500msec
// Description		: Periodic function for 500msec
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction_500msec(void)
{  
  bTimer_500ms_flag = FALSE;
/* USER CODE BEGIN 0 */
  if(bAlarmLedToggleEnableFlag) LED2_TOGGLE;

/* USER CODE END 0 */
}


//-------------------------------------------------------------
// Function name	: PeriodicFunction_1000msec
// Description		: Periodic function for 1000msec
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction_1000msec(void)
{  
  bTimer_1000ms_flag = FALSE;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
}



//-------------------------------------------------------------
// Function name	: PeriodicFunction_5000msec
// Description		: Periodic function for 5000msec
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction_5000msec(void)
{  
  bTimer_5000ms_flag = FALSE;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
}

//-------------------------------------------------------------
// Function name	: PeriodicFunction
// Description		: Insert this Periodic function into main while loop
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void PeriodicFunction(void)
{  
  if(bTimer_1ms_flag == TRUE)    PeriodicFunction_1msec();
  if(bTimer_10ms_flag == TRUE)   PeriodicFunction_10msec();
  if(bTimer_100ms_flag == TRUE)  PeriodicFunction_100msec();
  if(bTimer_500ms_flag == TRUE)  PeriodicFunction_500msec();
  if(bTimer_1000ms_flag == TRUE) PeriodicFunction_1000msec();
  if(bTimer_5000ms_flag == TRUE) PeriodicFunction_5000msec();
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
}


/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
