/* ###################################################################
**  Filename	    	: stspin32gx_register_config.c
**  Title		: STSPIN32GX gate driver with embedded STM32G431 MCU
**  Project		: test environment implementation for EVSPIN32G4 
**  Processor		: STM32G431VBx / STMicroelectronics
**  ToolChain		: IAR Embedded Workbench IDE - Arm 9.10.2
**  Compiler		: IAR C/C++ Compiler for ARM  9.10.2.313 
**  Date/Time		: 2022.02.04
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.0 / 2020.09.09 Lombardi, IPC Application Team
begin
    . I2C HAL driver used
- V0.0.1 / 2022.02.04 / skkim
update and add
    . I2C driver change to LL driver
    . variables structure changed, removed and added
    . functions changed, removed and added
*/

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "stspin32gx_register_config.h"
#include "stspin32gx_i2c.h"
#include "User_predefine.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t STSPIN32GX_bkupREADY;			// backup of the READ register during standby

STSPIN32G4_REG stSpinReg = {
  STSPIN32GX_I2C_POWMNG,	0x00,
  STSPIN32GX_I2C_LOGIC,		0x73,
  STSPIN32GX_I2C_READY,		0x09,
  STSPIN32GX_I2C_NFAULT,	0x7F,
  STSPIN32GX_I2C_CLEAR,		0x7F,
  STSPIN32GX_I2C_STBY,		0x00,
  STSPIN32GX_I2C_LOCK,		0xDD,
  STSPIN32GX_I2C_RESET,		0xFF,
  STSPIN32GX_I2C_STATUS,	0x80
};

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
//-------------------------------------------------------------
// Function name	: Clear_All_STSPIN32G_REG_Values
// Description		: set to 0 for all register values 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void Clear_All_STSPIN32G_REG_Values(void)
{
  stSpinReg.POWMNG.cValue.all = 0;  
  stSpinReg.LOGIC.cValue.all = 0;  
  stSpinReg.READY.cValue.all = 0;  
  stSpinReg.FAULT.cValue.all = 0;  
  stSpinReg.CLEAR.cValue.all = 0;  
  stSpinReg.STBY.cValue.all = 0;  
  stSpinReg.LOCK.cValue.all = 0;  
  stSpinReg.RST.cValue.all = 0;  
  stSpinReg.STATUS.cValue.all = 0;  
}

//-------------------------------------------------------------
// Function name	: LL_STSPIN32GX_readReg
// Description		: register value read & store 
// Parameter		: uint8_t regAddr - register address to read
//                      : uint8_t* value - data address to store
// Return		: none
//-------------------------------------------------------------
void LL_STSPIN32GX_readReg(uint8_t regAddr, uint8_t* value)
{
  BSP_IIC_ReadReg(STSPIN32GX_I2C_ADDR, (uint16_t) regAddr, value, 1, STSPIN32GX_I2C_TImeOUT);
}

//-------------------------------------------------------------
// Function name	: LL_STSPIN32GX_writeReg
// Description		: register values write
// Parameter		: uint8_t regAddr - register address to write
//                      : uint8_t value - data to write
// Return		: none
//-------------------------------------------------------------
void LL_STSPIN32GX_writeReg(uint8_t regAddr, uint8_t value)
{
  BSP_IIC_WriteReg(STSPIN32GX_I2C_ADDR, (uint16_t)regAddr, &value, 1, STSPIN32GX_I2C_TImeOUT);
}

//-------------------------------------------------------------
// Function name	: LL_STSPIN32GX_writeVerifyReg
// Description		: register values verify after writing
//                        The writing value will be stored to global variable [stSpinReg]
// Parameter		: uint8_t regAddr - register address to write
//                      : uint8_t value - data to write
// Return		: bool
//-------------------------------------------------------------
bool LL_STSPIN32GX_writeVerifyReg(uint8_t regAddr, uint8_t value)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_writeReg(regAddr, value);
  LL_STSPIN32GX_readReg(regAddr, &i2cReg);
  
  if(i2cReg == value)   
  {
    if(regAddr == STSPIN32GX_I2C_POWMNG)	stSpinReg.POWMNG.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_LOGIC)	stSpinReg.LOGIC.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_READY)	stSpinReg.READY.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_NFAULT)	stSpinReg.FAULT.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_CLEAR)	stSpinReg.CLEAR.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_STBY)	stSpinReg.STBY.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_LOCK)	stSpinReg.LOCK.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_RESET)	stSpinReg.RST.cValue.all = i2cReg;
    else if(regAddr == STSPIN32GX_I2C_STATUS)	stSpinReg.STATUS.cValue.all = i2cReg;
  
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

////////////////////////////////////////////////////////////////////////////////
// STSPIN32GX REG : Lock configuration
// [0x0B] Command register: Lock:  0xDD
//    - NLOCK = 0x0D :
//    - LOCK = 0x0D :
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_lockReg
// Description		: Locks protected registers
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_lockReg(void)
{
  uint8_t i2cReg = 0;
  
  i2cReg = ((STSPIN32GX_I2C_LOCKCODE<<4)&0xf0)|(STSPIN32GX_I2C_LOCKCODE&0x0f);
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOCK, i2cReg);
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_lockReg
// Description		: Unlocks protected registers
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_unlockReg(void)
{
  uint8_t i2cReg = 0;
  
  i2cReg = (((~STSPIN32GX_I2C_LOCKCODE)<<4)&0xf0)|(STSPIN32GX_I2C_LOCKCODE&0x0f);
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOCK, i2cReg);
 }

////////////////////////////////////////////////////////////////////////////////
// STSPIN32GX REG : Power manager configuration
// [0x01] Power manager configuration: 0x03
//    - REG3V3   = 0 (0: Internal 3.3V, 1: External 3.3V)
//    - VCC      = 0 (0: Internal VCC, 1: External VCC)
//    - STBY_REG = 0 (0: disable, 1: Enable)
//    - VCC_VAL  = 3 (0: 8V, 1: 10V, 2: 12V, 3: 15V)
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_VCC_SetVoltage
// Description		: sets the output value for the VCC regulator
// Parameter		: uint8_t cVccOutput - 0: 8V, 1: 10V, 2: 12V, 3: 15V
//	- STSPIN32GX_I2C_VCC_8V		(0)
//	- STSPIN32GX_I2C_VCC_10V	(1)
//	- STSPIN32GX_I2C_VCC_12V	(2)
//	- STSPIN32GX_I2C_VCC_15V	(3)	
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_VCC_SetVoltage(uint8_t cVccOutput)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= (~0x03); // clear VCC_VAL
  i2cReg |= cVccOutput;
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_VCC_Enable
// Description		: Enable the buck regulator(Internal VCC)
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_VCC_Enable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= ~STSPIN32GX_I2C_VCC_DIS; // set to 0: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_VCC_Disable
// Description		: Disables the buck regulator(External VCC)
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_VCC_Disable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg |= STSPIN32GX_I2C_VCC_DIS; // set to 1: disable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_REG3V3_Enable
// Description		: Enable the 3.3 V Internal linear regulator 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_REG3V3_Enable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= ~STSPIN32GX_I2C_REG3V3_DIS; // set to 0: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_REG3V3_Disable
// Description		: disable the 3.3 V Internal linear regulator 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_REG3V3_Disable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg |= STSPIN32GX_I2C_REG3V3_DIS; // set to 1: disable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_STBY_REG_Enable
// Description		: Enable the 3.3 V Standby regulator 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_STBY_REG_Enable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg |= STSPIN32GX_I2C_STBY_REG_EN; // set to 1: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_STBY_REG_Disable
// Description		: disable the 3.3 V Standby regulator 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_STBY_REG_Disable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= ~STSPIN32GX_I2C_STBY_REG_EN; // set to 0: disable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_POWMNG, i2cReg);
  STSPIN32GX_lockReg();
}

////////////////////////////////////////////////////////////////////////////////
// STSPIN32GX REG : Ready output configuration
// [0x07] READY output configuration: 0x0B
//    - STBY_RDY     = 1 (0: Disable, 1:Enable)
//    - THSD_RDY     = 1 (0: Disable, 1:Enable)
//    - VCC_UVLO_RDY = 1 (0: Disable, 1:Enable)
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_ReadyOuput_Config
// Description		: enables the signaling of the READY output 
//	- Gate Driver READY output is connected to PE14(GPIO input) of MCU
// Parameter		: uint8_t cRegValue
//	- STSPIN32GX_I2C_STBY_RDY	(1<<3)
// 		STBY_RDY enables the signaling of the standby request status 
// 		(0: Disable, 1:Enable)
//	- STSPIN32GX_I2C_THSD_RDY	(1<<1)
//		THSD_RDY enables the signaling of the thermal shutdown status 
// 		(0: Disable, 1:Enable)
//	- STSPIN32GX_I2C_VCC_UVLO_RDY	(1<<0)
//		VCC_UVLO_RDY enables the signaling of the VCC UVLO status 
// 		(0: Disable, 1:Enable)					
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_ReadyOuput_Config(uint8_t cRegValue)
{
  uint8_t i2cReg = 0;
  
  i2cReg &= ~(STSPIN32GX_I2C_STBY_RDY|
              STSPIN32GX_I2C_THSD_RDY|
              STSPIN32GX_I2C_VCC_UVLO_RDY); // clear all register
  i2cReg |= cRegValue; // set to 0: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_READY, i2cReg);
}


////////////////////////////////////////////////////////////////////////////////
// STSPIN32GX REG : nFault output configuration
// [0x08] nFAULT output configuration: 0x7F
//    - RESET_FLT    = 1 (0: Disable, 1:Enable)
//    - VDS_P_FLT    = 1 (0: Disable, 1:Enable)
//    - THSD _FLT    = 1 (0: Disable, 1:Enable)
//    - VCC_UVLO_FLT = 1 (0: Disable, 1:Enable)
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_nFaultOuput_Config
// Description		: enables the signaling of the READY output 
//	- Gate Driver nFAULT output is connected to PE15(GPIO input) of MCU
// Parameter		: uint8_t cRegValue
//	- STSPIN32GX_I2C_RESET_FLT		(1<<3)
//		RESET_FLT enables the signaling that a reset was done (RESERVED)
// 		(0: Disable, 1:Enable)
//	- STSPIN32GX_I2C_VDS_P_FLT		(1<<2)
//		VDS_P_FLT enables the signaling of the VDS protection triggering 
// 		(0: Disable, 1:Enable)
//	- STSPIN32GX_I2C_THSD_FLT		(1<<1)
//		THSD_FLT enables the signaling of the thermal shutdown status 
// 		(0: Disable, 1:Enable)					
//	- STSPIN32GX_I2C_VCC_UVLO_FLT		(1<<0)
//		VCC_UVLO_FLT enables the signaling of the VCC UVLO status 
// 		(0: Disable, 1:Enable)					
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_nFaultOuput_Config(uint8_t cRegValue)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_NFAULT, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= ~(STSPIN32GX_I2C_RESET_FLT|
              STSPIN32GX_I2C_VDS_P_FLT|
              STSPIN32GX_I2C_THSD_FLT|
              STSPIN32GX_I2C_VCC_UVLO_FLT); // clear all register
  i2cReg |= cRegValue; // set to 0: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_NFAULT, i2cReg);
  STSPIN32GX_lockReg();
}


////////////////////////////////////////////////////////////////////////////////
// STSPIN32GX REG : Driving logic configuration:
// [0x02] Driving logic configuration: 0x7F
//    - VDS_P_DEG = 3 (0: 6us, 1: 4us, 2: 3us, 3: 2us)
//    - DTMIN     = 1 (0: Disable, 1: Enable(100~200ns)
//    - ILOCK     = 1 (0: Disable, 1: Enable)
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_Interlocking_Enable
// Description		: enables interlocking 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_Interlocking_Enable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOGIC, &i2cReg);  
  STSPIN32GX_unlockReg();
  
  i2cReg |= STSPIN32GX_I2C_ILOCK; // set to 1: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOGIC, i2cReg);
  STSPIN32GX_lockReg();
}  

//-------------------------------------------------------------
// Function name	: STSPIN32GX_Interlocking_Disable
// Description		: disable interlocking
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_Interlocking_Disable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOGIC, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= ~STSPIN32GX_I2C_ILOCK; // set to 0: disable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOGIC, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_MinimumDeadTime_Enable
// Description		: enables minimum dead time(100~200ns) insertion 
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_MinimumDeadTime_Enable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOGIC, &i2cReg);  
  STSPIN32GX_unlockReg();
  
  i2cReg |= STSPIN32GX_I2C_DTMIN; // set to 1: enable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOGIC, i2cReg);
  STSPIN32GX_lockReg();
}  

//-------------------------------------------------------------
// Function name	: STSPIN32GX_MinimumDeadTime_Disable
// Description		: disable minimum dead time insertion
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_MinimumDeadTime_Disable(void)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOGIC, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= ~STSPIN32GX_I2C_DTMIN; // set to 0: disable
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOGIC, i2cReg);
  STSPIN32GX_lockReg();
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_VdsProtection_DeglitchTime_Set
// Description		: configures the deglitch time for the VDS protection 
// Parameter		: uint8_t cDeglitchTime - 0: 6us, 1: 4us, 2: 3us, 3: 2us
//	- STSPIN32GX_I2C_VDS_P_DEG_6US		(0<<2)
//	- STSPIN32GX_I2C_VDS_P_DEG_4US		(1<<2)
//	- STSPIN32GX_I2C_VDS_P_DEG_3US		(2<<2)
//	- STSPIN32GX_I2C_VDS_P_DEG_2US		(3<<2)
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_VdsProtection_DeglitchTime_Set(uint8_t cDeglitchTime)
{
  uint8_t i2cReg = 0;
  
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOGIC, &i2cReg);
  STSPIN32GX_unlockReg();
  
  i2cReg &= (~STSPIN32GX_I2C_VDS_P_DEG_2US); // clear DeglitchTime
  i2cReg |= cDeglitchTime;
  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_LOGIC, i2cReg);
  STSPIN32GX_lockReg();
}


////////////////////////////////////////////////////////////////////////////////
// [0x09] Command register: Fault clear: 0xFF
//    - FAULT_CLEAR (clear command is 0xFF)
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_clearFaults
// Description		: Setting high all the bits clears all the latched failure conditions. 
//                        Any other value is rejected.
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_clearFaults(void)
{
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_CLEAR, 0xFF);
}

////////////////////////////////////////////////////////////////////////////////
// [0x0C] Command register: RESET: 0xFF
//    - RESET (reset command is 0xFF)
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 ----------- STSPIN32G4 Register default condition -----------
 [0x01] Power manager configuration: 0x00
    - REG3V3   = 0 (0: Internal 3.3V, 1: External 3.3V)
    - VCC      = 0 (0: Internal VCC, 1: External VCC)
    - STBY_REG = 0 (0: disable, 1: Enable)
    - VCC_VAL  = 0 (0: 8V, 1: 10V, 2: 12V, 3: 15V)

 [0x02] Driving logic configuration: 0x73
    - VDS_P_DEG = 0 (0: 6us, 1: 4us, 2: 3us, 3: 2us)
    - DTMIN     = 1 (0: Disable, 1: Enable(100~200ns)
    - ILOCK     = 1 (0: Disable, 1: Enable)

 [0x07] READY output configuration: 0x09
    - STBY_RDY     = 1 (0: Disable, 1:Enable)
    - THSD_RDY     = 0 (0: Disable, 1:Enable)
    - VCC_UVLO_RDY = 1 (0: Disable, 1:Enable)

 [0x08] nFAULT output configuration: 0x7F
    - RESET_FLT    = 1 (0: Disable, 1:Enable)
    - VDS_P_FLT    = 1 (0: Disable, 1:Enable)
    - THSD _FLT    = 1 (0: Disable, 1:Enable)
    - VCC_UVLO_FLT = 1 (0: Disable, 1:Enable)

 [0x09] Command register: Fault clear: 0xFF
    - FAULT_CLEAR (clear command is 0xFF)

 [0x0A] Standby: 0x00
    - STBY = 0 (0: normal, 1: Standby)

 [0x0B] Lock:  0xDD
    - NLOCK = 0x0D :
    - LOCK = 0x0D :

 [0x0C] Command register: RESET: 0xFF
    - RESET (reset command is 0xFF)

 [0x80] Reports the device status: 0x80
    - LOCK      = 1 (0: Unlocked, 1:Locked)
    - RESET     = 0 (0: No reset, 1: Reset)
    - VDS_P     = 0 (0: Not triggered, 1: Triggered)
    - THSD      = 0 (0: Not triggered, 1: Triggered)
    - VCC_UVLO  = 0 (0: Above Threshold, 1: Under Threshold)
*******************************************************************************/
//-------------------------------------------------------------
// Function name	: STSPIN32GX_reset
// Description		: Setting high all the bits resets the registers to the default value. 
//                        Any other value is rejected.
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_reset(void)
{
  STSPIN32GX_unlockReg();  
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_RESET, 0xFF);  
  STSPIN32GX_lockReg();
}

////////////////////////////////////////////////////////////////////////////////
// [0x0A] Command register: Standby: 0x00
//    - STBY = 0 (0: normal, 1: Standby)
////////////////////////////////////////////////////////////////////////////////

#define STSPIN32GX_RCC_DEINIT
#ifdef STSPIN32GX_RCC_DEINIT
/**
  * @brief  Reset the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 OFF
  *            - All interrupts disabled
  *            - All interrupt and reset flags cleared
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  * @retval HAL status
  */
LL_StatusTypeDef USER_RCC_DeInit(void)
{
  uint32_t tickstart;

  /* Get Start Tick*/
  tickstart = uwTick;//HAL_GetTick();

  /* Set HSION bit to the reset value */
  SET_BIT(RCC->CR, RCC_CR_HSION);

  /* Wait till HSI is ready */
  while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0U)
  {
    if ((uwTick - tickstart) > 2U)
    {
      return LL_TIMEOUT;
    }
  }

 /* Set HSITRIM[6:0] bits to the reset value */
  SET_BIT(RCC->ICSCR, 0x40U << RCC_ICSCR_HSITRIM_Pos);

  /* Get Start Tick*/
  tickstart = uwTick; //HAL_GetTick();

  /* Reset CFGR register (HSI is selected as system clock source) */
  RCC->CFGR = 0x00000001u;

  /* Wait till HSI is ready */
  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
  {
    if ((uwTick - tickstart) > 5000U)
    {
      return LL_TIMEOUT;
    }
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HSI_VALUE;

  /* Adapt Systick interrupt period */
//  if (HAL_InitTick(uwTickPrio) != HAL_OK)
//  {
//    return HAL_ERROR;
//  }

  /* Clear CR register in 2 steps: first to clear HSEON in case bypass was enabled */
  RCC->CR = RCC_CR_HSION;

  /* Then again to HSEBYP in case bypass was enabled */
  RCC->CR = RCC_CR_HSION;

  /* Get Start Tick*/
  tickstart = uwTick;//HAL_GetTick();

  /* Wait till PLL is OFF */
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != 0U)
  {
    if ((uwTick - tickstart) > 2)
    {
      return LL_TIMEOUT;
    }
  }

  /* once PLL is OFF, reset PLLCFGR register to default value */
  RCC->PLLCFGR = RCC_PLLCFGR_PLLN_4;

  /* Disable all interrupts */
  CLEAR_REG(RCC->CIER);

  /* Clear all interrupt flags */
  WRITE_REG(RCC->CICR, 0xFFFFFFFFU);

  /* Clear all reset flags */
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);

  return LL_OK;
}

void TurnOff_SystemClockforStandbyEnter(void)
{
	LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_DAC1);
	
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
	
  //LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C3);
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USART1);
	
  LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_HSI);
}

void TurnOn_SystemClockforStandbyEnter(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);
	
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
	
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	
  LL_RCC_SetI2CClockSource(LL_RCC_I2C3_CLKSOURCE_PCLK1);

}

void User_SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLP_DIV_8);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_EnableDomain_ADC();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1?s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
}
#endif

//-------------------------------------------------------------
// Function name	: STSPIN32GX_enterStandby
// Description		: Setting high the STBY bit requests the device to enter low consumption mode.
// Parameter		: bool enableStbyReg - 0: normal, 1: Standby
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_enterStandby(void)
{
  uint8_t i2cReg = 0;
	uint8_t status = 0;
		
	status = LL_OK;
	
	// Standby regulator enable to enter Standby Mode
	STSPIN32GX_STBY_REG_Enable();  
	
  // create backup of the READY register
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_READY, &STSPIN32GX_bkupREADY);
  
  i2cReg = STSPIN32GX_I2C_STBY_RDY;
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_READY, i2cReg);
  
  // 1. WAKE line low
  LL_GPIO_ResetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
	#ifdef STSPIN32GX_HSI16  
		if(enableStbyReg)      status = SystemClock_deInit(); 	// HSI16 to reduce current consumption
	#endif
		
  // 2. set STBY bit to 1 for requesting to enter standby
	#ifdef STSPIN32GX_RCC_DEINIT 
		USER_RCC_DeInit();
	#endif
		
  i2cReg = 0x01;
  STSPIN32GX_unlockReg();
  status = LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_STBY, i2cReg);
  STSPIN32GX_lockReg();
  
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_waitForStandby
// Description		: Wait for entering standby mode.
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_waitForStandby(void)
{
  //uint32_t tickstart = HAL_GetTick();
  uint32_t tickstart = uwTick;
  
  //if(HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_SET)
  if(LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin))
  {
    //HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);
    LL_GPIO_SetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
    //status =  HAL_ERROR;
  }
  else
  {
    //while(HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_RESET)
    while(!LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin))
    {
      //if((HAL_GetTick() - tickstart) > 1)		// expected time is 100us
      if((uwTick - tickstart) > 1)		// expected time is 100us
      {
        //status = HAL_TIMEOUT;
        break;
      }	
    }
  }
  
#ifdef STSPIN32GX_HSI16
  // if the driver failed to enter standby clock is reconfigured
  SystemClock_init();		// restore clock configuration
#endif
}

//-------------------------------------------------------------
// Function name	: STSPIN32GX_wakeup
// Description		: Set WAKEUP pin(PE7) to high and wait for wakeup.
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_wakeup(uint8_t timeout_ms)
{  
  uint8_t i2cReg = 0;
	
  //HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);
  LL_GPIO_SetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
  
  if(timeout_ms < 4)    timeout_ms = 4;	// The soft start is expected to take 4ms
  //uint32_t tickstart = HAL_GetTick();
  uint32_t tickstart = uwTick;
    
  //while(HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_RESET)
  while(!LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin))
  {
    //if((HAL_GetTick() - tickstart) > timeout_ms)	
    if((uwTick - tickstart) > timeout_ms)	
    {
      //status = HAL_TIMEOUT;
      break;
    }
  }
  
#ifdef STSPIN32GX_HSI16
  SystemClock_init();
#endif
  
  // Restore READY register
  LL_STSPIN32GX_writeVerifyReg(STSPIN32GX_I2C_READY, STSPIN32GX_bkupREADY);
  STSPIN32GX_lockReg();
		
	// Standby reg disable
	STSPIN32GX_STBY_REG_Disable();
	
	// initialize system clock again.
  //SystemClock_Config();	
	STSPIN32G4_STBY_Recovery_for_CLK();
  /* Initialize all configured peripherals */
	// ...
	// ...
	// ...
	
	
}

////////////////////////////////////////////////////////////////////////////////
// STSPIN32GX REG : Status register check
// [0x80] Reports the device status: 0x80
////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------
// Function name	: STSPIN32GX_getStatus
// Description		: Read status register and store the value to global variable
// Parameter		: none
// Return		: none
//-------------------------------------------------------------
void STSPIN32GX_getStatus(void)
{
  //LL_STSPIN32GX_readReg(STSPIN32GX_I2C_STATUS, (uint8_t*)status);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_STATUS,	&stSpinReg.STATUS.cValue.all);
}

////////////////////////////////////////////////////////////////////////////////
// -----| All register check
////////////////////////////////////////////////////////////////////////////////
void CheckAllGateDriverRegisters(void)
{
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_POWMNG,	&stSpinReg.POWMNG.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOGIC,	&stSpinReg.LOGIC.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_READY,	&stSpinReg.READY.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_NFAULT,	&stSpinReg.FAULT.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_CLEAR,	&stSpinReg.CLEAR.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_STBY,	&stSpinReg.STBY.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_LOCK,	&stSpinReg.LOCK.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_RESET,	&stSpinReg.RST.cValue.all);
  LL_STSPIN32GX_readReg(STSPIN32GX_I2C_STATUS,	&stSpinReg.STATUS.cValue.all);
}

////////////////////////////////////////////////////////////////////////////////
// -----| Initialize all of the STSPIN32GX registers
////////////////////////////////////////////////////////////////////////////////
void Init_STSPING4_GateDriver_Register(void)
{
  // -----------| 7	Command register: RESET
  STSPIN32GX_reset();  
  
  // -----------| 0	Power manager configuration
  STSPIN32GX_VCC_SetVoltage(STSPIN32GX_I2C_VCC_10V);
  STSPIN32GX_VCC_Enable();
  STSPIN32GX_REG3V3_Enable();
  
  // -----------| 1	Driving logic configuration:
  STSPIN32GX_Interlocking_Enable();
  STSPIN32GX_MinimumDeadTime_Enable();
  STSPIN32GX_VdsProtection_DeglitchTime_Set(STSPIN32GX_I2C_VDS_P_DEG_6US);
  
  // -----------| 2	READY output configuration:
  STSPIN32GX_ReadyOuput_Config(STSPIN32GX_I2C_STBY_RDY|
                               STSPIN32GX_I2C_THSD_RDY|
                               STSPIN32GX_I2C_VCC_UVLO_RDY);
  STSPIN32GX_ReadyOuput_Config(STSPIN32GX_I2C_STBY_RDY);
  
  // -----------| 3	nFAULT output configuration:
  STSPIN32GX_nFaultOuput_Config(STSPIN32GX_I2C_RESET_FLT|
                                STSPIN32GX_I2C_VDS_P_FLT|
                                STSPIN32GX_I2C_THSD_FLT|
                                STSPIN32GX_I2C_VCC_UVLO_FLT);  
  //STSPIN32GX_nFaultOuput_Config(0);
  
  // -----------| 4	Command register: Fault clear
  STSPIN32GX_clearFaults();
  
  // -----------| 8	Reports the device status:  
  CheckAllGateDriverRegisters();
}
/*********************************************************************************
* End of File
*********************************************************************************/