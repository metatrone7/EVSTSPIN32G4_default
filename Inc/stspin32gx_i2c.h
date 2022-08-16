/* ###################################################################
**  Filename	    	: stspin32gx_i2c.h
**  Title		: Header file of stspin32gx_i2c.c
**  Project		: test environment implementation for EVSPIN32G4 
**  Processor		: STM32G431VBx / STMicroelectronics
**  ToolChain		: IAR Embedded Workbench IDE - Arm 9.10.2
**  Compiler		: IAR C/C++ Compiler for ARM  9.10.2.313 
**  Date/Time		: 2020.09.09
**  Author		: Sungkyu Kim
** ###################################################################*/

/* Debug & Release Note
*
- V0.0.1 / 2020.09.09 Lombardi, IPC Application Team
begin
    . I2C HAL driver used
- V0.0.2 / 2022.02.04 / skkim
update and add
    . I2C driver change to LL driver
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN32G4_I2C_H
#define __STSPIN32G4_I2C_H	"STSPIN32G4_I2C"

#define	STSPIN32G4_I2C_VERSION	"0.0.2"	


/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>


/** @defgroup I2C_MEMORY_ADDRESS_SIZE I2C Memory Address Size*/
#define I2C_MEMADD_SIZE_8BIT            (0x00000001U)
#define I2C_MEMADD_SIZE_16BIT           (0x00000002U)

#define I2C_MEM_ADD_MSB(__ADDRESS__)	((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))
#define I2C_MEM_ADD_LSB(__ADDRESS__)	((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))

#define MAX_NBYTE_SIZE			(0x08)

extern void BSP_IIC_ReadReg(uint16_t DevAddress, 
                        uint16_t MemAddress, 
                        uint8_t *pData, 
                        uint16_t usLen, 
                        uint32_t ulTimeout);
extern void BSP_IIC_WriteReg(uint16_t DevAddress, 
                         uint16_t MemAddress, 
                         uint8_t *pData, 
                         uint16_t usLen, 
                         uint32_t ulTimeout);

#endif /* __STSPIN32G4_I2C_H */
/*********************************************************************************
* End of File
*********************************************************************************/
