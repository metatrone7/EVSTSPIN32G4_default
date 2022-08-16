/* ###################################################################
**  Filename	    	: stspin32gx_i2c.c
**  Title		: I2C read/write driver implementation
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

#include <stdint.h>
#include <stdbool.h>
#include "stm32g431xx.h"
#include "stm32g4xx_ll_i2c.h"

#include "stspin32gx_i2c.h"

//-------------------------------------------------------------
// Function name	: BSP_IIC_ReadReg
// Description		: I2C read function
// Parameter		: 
// Return		: 
//-------------------------------------------------------------
void BSP_IIC_ReadReg(uint16_t DevAddress, 
                        uint16_t MemAddress, 
                        uint8_t *pData, 
                        uint16_t usLen, 
                        uint32_t ulTimeout)
{ 
  uint16_t XferCount = usLen;
  uint8_t *RxBuffer = pData;
  uint16_t XferSize = 0;
  
  /*??I2C????*/
  while(((I2C3->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY);
  
  /* Send Slave Address and Memory Address */
  LL_I2C_HandleTransfer(I2C3,
                        DevAddress, 
                        LL_I2C_ADDRSLAVE_7BIT, 
                        I2C_MEMADD_SIZE_8BIT, 
                        LL_I2C_MODE_SOFTEND, 
                        LL_I2C_GENERATE_START_WRITE);
  
  while(((I2C3->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS);
  
  /* Send Memory Address */
  I2C3->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  while(((I2C3->ISR) & I2C_ISR_TC) != I2C_ISR_TC);
  
  /* Send Slave Address */
  /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
  if (XferCount > MAX_NBYTE_SIZE)
  {
    XferSize = MAX_NBYTE_SIZE;
    LL_I2C_HandleTransfer(I2C3, 
                          DevAddress, 
                          LL_I2C_ADDRSLAVE_7BIT, 
                          (uint8_t)XferSize, 
                          I2C_CR2_RELOAD , 
                          LL_I2C_GENERATE_START_READ);
  }
  else
  {
    XferSize = XferCount;
    LL_I2C_HandleTransfer(I2C3, 
                          DevAddress, 
                          LL_I2C_ADDRSLAVE_7BIT, 
                          (uint8_t)XferSize, 
                          I2C_CR2_AUTOEND ,
                          LL_I2C_GENERATE_START_READ);
  }
  
  do
  {
    /* Wait until RXNE flag is set */
    while(((I2C3->ISR) & I2C_ISR_RXNE) != I2C_ISR_RXNE);
    
    /* Read data from RXDR */
    *RxBuffer = (uint8_t)((I2C3->RXDR  >> 0) & 0xFF);
    
    /* Increment Buffer pointer */
    RxBuffer++;
    
    XferCount--;
    XferSize--;
    
    if ((XferCount != 0U) && (XferSize == 0U))
    {
      /* Wait until TCR flag is set */
      while(((I2C3->ISR) & I2C_ISR_TCR) != I2C_ISR_TCR);
      
      if (XferCount > MAX_NBYTE_SIZE)
      {
        XferSize = MAX_NBYTE_SIZE;
        LL_I2C_HandleTransfer(I2C3, 
                              DevAddress, 
                              LL_I2C_ADDRSLAVE_7BIT, 
                              (uint8_t)XferSize, 
                              I2C_CR2_RELOAD , 
                              LL_I2C_GENERATE_NOSTARTSTOP);
      }
      else
      {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2C3, 
                              DevAddress, 
                              LL_I2C_ADDRSLAVE_7BIT, 
                              (uint8_t)XferSize, 
                              I2C_CR2_AUTOEND , 
                              LL_I2C_GENERATE_NOSTARTSTOP);
      }
    }
  }
  while(XferCount > 0U);
  
  /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
  /* Wait until STOPF flag is reset */
  while(((I2C3->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
  {
    
  }
  /* Clear NACKF Flag */
  //    __HAL_I2C_CLEAR_FLAG(I2C3, I2C_ISR_NACKF);
  LL_I2C_ClearFlag_NACK(I2C3);
  
  /* Clear STOP Flag */
  LL_I2C_ClearFlag_STOP(I2C3);
  
  /* Clear Configuration Register 2 */
  I2C3->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | 
                                      I2C_CR2_HEAD10R | 
                                        I2C_CR2_NBYTES | 
                                          I2C_CR2_RELOAD | 
                                            I2C_CR2_RD_WRN));
  
}

//-------------------------------------------------------------
// Function name	: BSP_IIC_WriteReg
// Description		: I2C write function
// Parameter		: 
// Return		: 
//-------------------------------------------------------------
void BSP_IIC_WriteReg(uint16_t DevAddress, 
                         uint16_t MemAddress, 
                         uint8_t *pData, 
                         uint16_t usLen, 
                         uint32_t ulTimeout)
{
  uint16_t XferCount = usLen;
  uint8_t *TxBuffer = pData;
  uint16_t XferSize = 0;
  
  /*??I2C????*/
  while(((I2C3->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY);
  
  /* Send Slave Address and Memory Address */
  LL_I2C_HandleTransfer(I2C3, 
                        DevAddress, 
                        LL_I2C_ADDRSLAVE_7BIT, 
                        I2C_MEMADD_SIZE_8BIT, 
                        I2C_CR2_RELOAD , 
                        LL_I2C_GENERATE_START_WRITE);
  //      LL_I2C_HandleTransfer(I2C3, DevAddress, LL_I2C_ADDRSLAVE_7BIT, I2C_MEMADD_SIZE_8BIT, LL_I2C_MODE_AUTOEND , LL_I2C_GENERATE_START_WRITE);
  while(((I2C3->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS);
  /* Send Memory Address */
  I2C3->TXDR  = I2C_MEM_ADD_LSB(MemAddress);
  
  while(((I2C3->ISR) & I2C_ISR_TCR) != I2C_ISR_TCR);
  
  /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
  if (XferCount > MAX_NBYTE_SIZE)
  {
    XferSize = MAX_NBYTE_SIZE;
    LL_I2C_HandleTransfer(I2C3, 
                          DevAddress, 
                          LL_I2C_ADDRSLAVE_7BIT, 
                          (uint8_t)XferSize, 
                          I2C_CR2_RELOAD , 
                          LL_I2C_GENERATE_NOSTARTSTOP);
  }
  else
  {
    XferSize = XferCount;
    LL_I2C_HandleTransfer(I2C3, 
                          DevAddress, 
                          LL_I2C_ADDRSLAVE_7BIT, 
                          (uint8_t)XferSize, 
                          I2C_CR2_AUTOEND , 
                          LL_I2C_GENERATE_NOSTARTSTOP);
  }
  
  do
  {
    /* Wait until TXIS flag is set */
    while(((I2C3->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS);
    /* Write data to TXDR */
    I2C3->TXDR = *TxBuffer;
    /* Increment Buffer pointer */
    TxBuffer++;
    
    XferCount--;
    XferSize--;
    
    if ((XferCount != 0U) && (XferSize == 0U))
    {
      /* Wait until TCR flag is set */
      while(((I2C3->ISR) & I2C_ISR_TCR) != I2C_ISR_TCR);
      
      if (XferCount > MAX_NBYTE_SIZE)
      {
        XferSize = MAX_NBYTE_SIZE;
        LL_I2C_HandleTransfer(I2C3, 
                              DevAddress, 
                              LL_I2C_ADDRSLAVE_7BIT, 
                              (uint8_t)XferSize, 
                              I2C_CR2_RELOAD , 
                              LL_I2C_GENERATE_NOSTARTSTOP);
      }
      else
      {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2C3, 
                              DevAddress, 
                              LL_I2C_ADDRSLAVE_7BIT, 
                              (uint8_t)XferSize, 
                              I2C_CR2_AUTOEND , 
                              LL_I2C_GENERATE_NOSTARTSTOP);
      }
    }
  }
  while(XferCount > 0U);
  
  /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
  /* Wait until STOPF flag is reset */
  while(((I2C3->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF);
  
  /* Clear NACKF Flag */
  //    __HAL_I2C_CLEAR_FLAG(I2C3, I2C_ISR_NACKF);
  
  LL_I2C_ClearFlag_NACK(I2C3);
  
  /* Clear STOP Flag */
  LL_I2C_ClearFlag_STOP(I2C3);
  
  /* Clear Configuration Register 2 */
  I2C3->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | 
                                      I2C_CR2_HEAD10R | 
                                        I2C_CR2_NBYTES | 
                                          I2C_CR2_RELOAD | 
                                            I2C_CR2_RD_WRN));
}
