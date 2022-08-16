/* ###################################################################
**  Filename	    	: stspin32gx_register_config.h
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
- V0.0.0 / 2020.09.09 Lombardi, IPC Application Team
begin
    . I2C HAL driver used
- V0.0.1 / 2022.02.04 / skkim
update and add
    . I2C driver change to LL driver
    . variables structure changed, removed and added
    . functions changed, removed and added
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN32G4_REGISTER_CONFIG_H
#define __STSPIN32G4_REGISTER_CONFIG_H	"STSPIN32G4_REGISTER_CONFIG"

#define	STSPIN32G4_REGISTER_CONFIG_VERSION	"0.0.1"	

/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>


/*********************************************************************************
* Defines
*********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/      
#define STSPIN32GX_I2C_TImeOUT			(100)
#define STSPIN32GX_I2C_ADDR			(0x8E)

#define STSPIN32GX_I2C_POWMNG			(0x01)
#define STSPIN32GX_I2C_REG3V3_DIS		(1<<6)
#define STSPIN32GX_I2C_VCC_DIS			(1<<5)
#define STSPIN32GX_I2C_STBY_REG_EN		(1<<4)
#define STSPIN32GX_I2C_VCC_VAL_0		(0)
#define STSPIN32GX_I2C_VCC_VAL_1		(1)
#define STSPIN32GX_I2C_VCC_VAL_2		(2)
#define STSPIN32GX_I2C_VCC_VAL_3		(3)

#define STSPIN32GX_I2C_VCC_8V			(0)
#define STSPIN32GX_I2C_VCC_10V			(1)
#define STSPIN32GX_I2C_VCC_12V			(2)
#define STSPIN32GX_I2C_VCC_15V			(3)

#define STSPIN32GX_I2C_LOGIC			(0x02)
#define STSPIN32GX_I2C_VDS_P_DEG_0		(0<<2)
#define STSPIN32GX_I2C_VDS_P_DEG_1		(1<<2)
#define STSPIN32GX_I2C_VDS_P_DEG_2		(2<<2)
#define STSPIN32GX_I2C_VDS_P_DEG_3		(3<<2)

#define STSPIN32GX_I2C_VDS_P_DEG_6US		(0<<2)
#define STSPIN32GX_I2C_VDS_P_DEG_4US		(1<<2)
#define STSPIN32GX_I2C_VDS_P_DEG_3US		(2<<2)
#define STSPIN32GX_I2C_VDS_P_DEG_2US		(3<<2)

#define STSPIN32GX_I2C_DTMIN			(1<<1)
#define STSPIN32GX_I2C_ILOCK			(1<<0)

#define STSPIN32GX_I2C_READY			(0x07)
#define STSPIN32GX_I2C_STBY_RDY			(1<<3)
#define STSPIN32GX_I2C_THSD_RDY			(1<<1)
#define STSPIN32GX_I2C_VCC_UVLO_RDY		(1<<0)

#define STSPIN32GX_I2C_NFAULT			(0x08)
#define STSPIN32GX_I2C_LOCK_FLT		        (1<<7)
#define STSPIN32GX_I2C_RESET_FLT		(1<<3)
#define STSPIN32GX_I2C_VDS_P_FLT		(1<<2)
#define STSPIN32GX_I2C_THSD_FLT			(1<<1)
#define STSPIN32GX_I2C_VCC_UVLO_FLT		(1<<0)

#define STSPIN32GX_I2C_CLEAR			(0x09)

#define STSPIN32GX_I2C_STBY			(0x0A)

#define STSPIN32GX_I2C_LOCK			(0x0B)
#define STSPIN32GX_I2C_LOCKCODE			(0xD)
//#define STSPIN32GX_I2C_LOCKUSEPARANOID

#define STSPIN32GX_I2C_RESET			(0x0C)

#define STSPIN32GX_I2C_STATUS			(0x80)


/*********************************************************************************
* typedef
*********************************************************************************/

#define GD_READY_Pin LL_GPIO_PIN_14
#define GD_READY_GPIO_Port GPIOE
#define GD_NFAULT_Pin LL_GPIO_PIN_15
#define GD_NFAULT_GPIO_Port GPIOE
//-------------------------------------------------------------
/* enum LL status */
//-------------------------------------------------------------
typedef enum
{
  LL_OK       = 0x00U,
  LL_ERROR    = 0x01U,
  LL_BUSY     = 0x02U,
  LL_TIMEOUT  = 0x03U
} LL_StatusTypeDef;

//-------------------------------------------------------------
/* struct  POWER_MANAGER		POWMNG; */
//-------------------------------------------------------------
struct POWMNG_VAL_BITS
{
  uint8_t	ubVCC_VAL	:2;	// VCC_VAL sets the output value for the VCC regulator (0: 8V, 1: 10V, 2: 12V, 3: 15V)
  uint8_t	rsvd_1		:2;	// Reserved
  uint8_t	ubSTBY_REG_EN	:1;	// STBY_REG_EN enables the standby linear regulator (0: disable, 1: Enable)
  uint8_t	ubVCC_DIS	:1;	// VCC_DIS disables the VCC buck regulator (0: Enable the buck regulator(Internal VCC), 1: Disables the buck regulator(External VCC))
  uint8_t	ubREG3V3_DIS	:1;	// REG3V3_DIS disables the 3.3 V Internal linear regulator (0: Enable, 1: Disable)
  uint8_t	rsvd_2		:1;	// Reserved
};
union POWMNG_VAL
{
  uint8_t	all;
  struct	POWMNG_VAL_BITS	bit;
};

struct POWER_MANAGER
{
  const uint8_t		cAddr;
  union POWMNG_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  DRIVING_LOGIC		LOGIC; */
//-------------------------------------------------------------
struct LOGIC_VAL_BITS
{
  uint8_t	ubILOCK		:1;	// ILOCK enables interlocking (0: Disable, 1: Enable - The high-side and low-side MOSFETs of an half-bridge cannot be both on at the same time)
  uint8_t	ubDTMIN		:1;	// DTMIN enables minimum dead time insertion (0: Disable, 1: Enable(100~200ns) - it is not available when the interlocking is disabled)
  uint8_t	ubVDS_P_DEG	:2;	// VDS_P_DEG configures the deglitch time for the VDS protection (0: 6us, 1: 4us, 2: 3us, 3: 2us)
  uint8_t	rsvd_1		:4;	// Reserved
};
union LOGIC_VAL
{
  uint8_t	all;
  struct	LOGIC_VAL_BITS	bit;
};

struct DRIVING_LOGIC
{
  const uint8_t		cAddr;
  union LOGIC_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  READY_OUTPUT		READY; */
//-------------------------------------------------------------
struct READY_VAL_BITS
{
  uint8_t	ubVCC_UVLO_RDY	:1;	// VCC_UVLO_RDY enables the signaling of the VCC UVLO status (0: Disable, 1:Enable)
  uint8_t	ubTHSD_RDY	:1;	// THSD_RDY enables the signaling of the thermal shutdown status (0: Disable, 1:Enable)
  uint8_t	rsvd_1		:1;	// Reserved
  uint8_t	ubSTBY_RDY	:1;	// STBY_RDY enables the signaling of the standby request status (0: Disable, 1:Enable)
  uint8_t	rsvd_2		:4;	// Reserved
};
union READY_VAL
{
  uint8_t	all;
  struct	READY_VAL_BITS	bit;
};

struct READY_OUTPUT
{
  const uint8_t		cAddr;
  union READY_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  NFAULT_OUTPUT		FAULT; */
//-------------------------------------------------------------
struct NFAULT_VAL_BITS
{
  uint8_t	ubVCC_UVLO_FLT	:1;	// VCC_UVLO_FLT enables the signaling of the VCC UVLO status (0: Disable, 1:Enable)
  uint8_t	ubTHSD_FLT	:1;	// THSD_FLT enables the signaling of the thermal shutdown status (0: Disable, 1:Enable)
  uint8_t	ubVDS_P_FLT	:1;	// VDS_P_FLT enables the signaling of the VDS protection triggering (0: Disable, 1:Enable)
  uint8_t	ubRESET_FLT	:1;	// RESET_FLT enables the signaling that a reset was done (RESERVED from the customer point of view) (0: Disable, 1:Enable)
  uint8_t	rsvd_1		:4;	// Reserved
};
union NFAULT_VAL
{
  uint8_t	all;
  struct	NFAULT_VAL_BITS	bit;
};

struct NFAULT_OUTPUT
{
  const uint8_t		cAddr;
  union NFAULT_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  FAULT_CLR_CMD		CLEAR; */
//-------------------------------------------------------------
struct CLEAR_VAL_BITS
{
  uint8_t	ubCLEAR		:8;	// Setting high all the bits clears all the latched failure conditions. Any other value is rejected.
};
union CLEAR_VAL
{
  uint8_t	all;
  struct	CLEAR_VAL_BITS	bit;
};

struct FAULT_CLR_CMD
{
  const uint8_t		cAddr;
  union CLEAR_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  STANDBY_CMD		STBY; */
//-------------------------------------------------------------
struct STBY_VAL_BITS
{
  uint8_t	ubSTBY		:1;	// Setting high the STBY bit requests the device to enter low consumption mode. (0: normal, 1: Standby)
  uint8_t	rsvd_1		:7;	// Reserved
};
union STBY_VAL
{
  uint8_t	all;
  struct	STBY_VAL_BITS	bit;
};

struct STANDBY_CMD
{
  const uint8_t		cAddr;
  union STBY_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  LOCK_CMD		LOCK; */
//-------------------------------------------------------------
struct LOCK_VAL_BITS
{
  uint8_t	ubLOCK		:4;	// When LOCK is different from the bitwise not of NLOCK, the writing of the protected registers is not allowed.
  uint8_t	ubNLOCK		:4;	// When LOCK is equal to the bitwise not of NLOCK the writing of the protected registers is allowed and all gate drivers outputs are forced low.
};
union LOCK_VAL
{
  uint8_t	all;
  struct	LOCK_VAL_BITS	bit;
};

struct LOCK_CMD
{
  const uint8_t		cAddr;
  union LOCK_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  REG_RESET_CMD		RST; */
//-------------------------------------------------------------
struct RESET_VAL_BITS
{
  uint8_t	ubRESET		:8;	// Setting high all the bits resets the registers to the default value and then resets the register to its default (all zeroes). Any other value is rejected.
};
union RESET_VAL
{
  uint8_t	all;
  struct	RESET_VAL_BITS	bit;
};

struct REG_RESET_CMD
{
  const uint8_t		cAddr;
  union RESET_VAL 	cValue;
};

//-------------------------------------------------------------
/* struct  DEVICE_STATUS		STATUS; */
//-------------------------------------------------------------
struct STATUS_VAL_BITS
{
  uint8_t	ubVCC_UVLO	:1;	// VCC_UVLO indicates the VCC UVLO status (0: Above Threshold, 1: Under Threshold)
  uint8_t	ubTHSD		:1;	// THSD indicates the thermal shutdown status (0: Not triggered, 1: Triggered)
  uint8_t	ubVDS_P		:1;	// VDS_P indicates the VDS protection triggering (0: Not triggered, 1: Triggered)
  uint8_t	ubRESET		:1;	// RESET indicates the registers had been reset to the default (reset command or power-up) (0: No reset, 1: Reset)
  uint8_t	rsvd_1		:3;	// Reserved
  uint8_t	ubLOCK		:1;	// LOCK indicates the protected registers are locked (read-only) (0: Unlocked, 1:Locked)
};
union STATUS_VAL
{
  uint8_t	all;
  struct	STATUS_VAL_BITS	bit;
};

struct DEVICE_STATUS
{
  const uint8_t		cAddr;
  union STATUS_VAL 	cValue;
};

//-------------------------------------------------------------
/* STSPIN32G_REG - Register Struct */
//-------------------------------------------------------------
typedef struct
{
  struct  POWER_MANAGER		POWMNG;
  struct  DRIVING_LOGIC		LOGIC;
  struct  READY_OUTPUT		READY;
  struct  NFAULT_OUTPUT		FAULT;
  struct  FAULT_CLR_CMD		CLEAR;
  struct  STANDBY_CMD		STBY;
  struct  LOCK_CMD		LOCK;
  struct  REG_RESET_CMD		RST;
  struct  DEVICE_STATUS		STATUS;
} STSPIN32G4_REG;
//-------------------------------------------------------------

  
/*********************************************************************************
* External variables
*********************************************************************************/
extern STSPIN32G4_REG stSpinReg;
extern __IO uint32_t uwTick;


/*********************************************************************************
* External functions
*********************************************************************************/
// General function
extern void Clear_All_STSPIN32G_REG_Values(void);
extern void LL_STSPIN32GX_readReg(uint8_t regAddr, uint8_t* value);
extern void LL_STSPIN32GX_writeReg(uint8_t regAddr, uint8_t value);
extern bool LL_STSPIN32GX_writeVerifyReg(uint8_t regAddr, uint8_t value);

// [0x01] Power manager configuration: 0x03
extern void STSPIN32GX_VCC_SetVoltage(uint8_t cVccOutput);
extern void STSPIN32GX_VCC_Enable(void);
extern void STSPIN32GX_VCC_Disable(void);
extern void STSPIN32GX_REG3V3_Enable(void);
extern void STSPIN32GX_REG3V3_Disable(void);
extern void STSPIN32GX_STBY_REG_Enable(void);
extern void STSPIN32GX_STBY_REG_Disable(void);

// [0x02] Driving logic configuration: 0x7F
extern void STSPIN32GX_Interlocking_Enable(void);
extern void STSPIN32GX_Interlocking_Disable(void);
extern void STSPIN32GX_MinimumDeadTime_Enable(void);
extern void STSPIN32GX_MinimumDeadTime_Disable(void);
extern void STSPIN32GX_VdsProtection_DeglitchTime_Set(uint8_t cDeglitchTime);

// [0x07] READY output configuration: 0x0B
extern void STSPIN32GX_ReadyOuput_Config(uint8_t cRegValue);

// [0x08] nFAULT output configuration: 0x7F
extern void STSPIN32GX_nFaultOuput_Config(uint8_t cRegValue);

// [0x09] Command register: Fault clear: 0xFF
extern void STSPIN32GX_clearFaults(void);

// [0x0C] Command register: RESET: 0xFF
extern void STSPIN32GX_reset(void);

// [0x0A] Command register: Standby: 0x00
extern void STSPIN32GX_enterStandby(void);
extern void STSPIN32GX_waitForStandby(void);
extern void STSPIN32GX_wakeup(uint8_t timeout_ms);

// [0x0B] Command register: Lock:  0xDD
extern void STSPIN32GX_lockReg(void);
extern void STSPIN32GX_unlockReg(void);

// [0x80] Reports the device status: 0x80
extern void STSPIN32GX_getStatus(void);

// -----| All register check
extern void CheckAllGateDriverRegisters(void);

// -----| Initialize all of the STSPIN32GX registers
extern void Init_STSPING4_GateDriver_Register(void);


#endif /* __STSPIN32G4_REGISTER_CONFIG_H */
/*********************************************************************************
* End of File
*********************************************************************************/
