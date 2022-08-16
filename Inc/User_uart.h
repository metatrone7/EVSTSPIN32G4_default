/* ###################################################################
**  Filename		: user_uart.h
**  Title		: Header file of user_uart.c
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
#ifndef __USER_UART_H
#define __USER_UART_H	"USER_UART"

#define	USER_UART_VERSION		"0.0.1"		// Reserved.Release.Debug


/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>


/*********************************************************************************
* Defines
*********************************************************************************/
#define USER_UART_VERSION	"0.0.1"		// Reserved.Release.Debug

#define CAR_RET			0x0D		// CR: Carrage Return
#define LIN_FED			0x0A		// LF: Line Feed

#define BS_KEY			0x08
#define FUNTION_KEY		0x1B
#define F_UP			0x41
#define F_DOWN			0x42

#define UART_RX_BUFFER_SIZE	100
#define UART_TX_BUFFER_SIZE	100


#define	MAX_TX_LIMIT		512
#define	MAX_RX_LIMIT		512
#define	MAX_PACKET		512

#define SCI_1_ACTIVE		0
#define SCI_2_ACTIVE		1

#define SCI_A_TX		17
#define SCI_A_RX		18
#define SCI_B_TX           	8
#define SCI_B_RX            	9

#define SCI_MOTOR		'A'
#define SCI_DEBUG		'B'

#define SCI_DCM_ACTIVE		SCI_1_ACTIVE
#define SCI_DEBUG_ACTIVE	SCI_2_ACTIVE

#define SCI_MOTOR_BAUDRATE  	115200
#define SCI_DEBUG_BAUDRATE  	115200

#define MACHINE_NAME(x)		(Send_string_print(x, "\r\nEVSPIN32G4>>"))
#define ANSWER_BAR(x)		(Send_string_print(x, "\r\n     A> "))
#define	EMPTY_LINE(x)		(Send_string_print(x, "\r\n"))
#define	EMPTY_TAB(x)		(Send_char_print((x), '\t'))

typedef struct Rx_index{
	unsigned char	chRx_buf[MAX_RX_LIMIT];
	unsigned short	nRx_head;
	unsigned short	nRx_tail;
	unsigned short	nRx_cnt;
}Rx_index;

typedef struct
{
  bool	bRX_Buffer_over_flag;
  bool	bTX_Buffer_over_flag;
  bool	bRX_flag;
  bool	bTX_flag;
  
  char	cRX_Buffer[UART_RX_BUFFER_SIZE];
  char	cTX_Buffer[UART_TX_BUFFER_SIZE];
  
  uint16_t	uPort;
  uint16_t	uRX_Buffer_index;
  uint16_t	uTX_Buffer_index;
} Uart_t;



/*********************************************************************************
* Extern Variables
*********************************************************************************/
extern bool	Print_flag;
extern Uart_t	stSci_Debug;



/*********************************************************************************
* Extern Functions
*********************************************************************************/
extern void Init_Sci_Variable(Uart_t* _pstSci, uint16_t _uPort);
extern void Init_User_UART(void);
extern void Send_char_print(uint8_t _Port, char _Char);
extern void Send_string_print(uint8_t _Port, char* _String);
extern void Send_string_char_print(uint8_t _Port, uint8_t _size, uint8_t* _String);
extern void Send_int_print(uint8_t _Port, int16_t _iData);
extern void Send_uint_print(uint8_t _Port, uint16_t _uData);
extern void Send_long_print(uint8_t _Port, int32_t _ilData);
extern void Send_ulong_print(uint8_t _Port, uint32_t _ulData);
extern void Send_int64_print(uint8_t _Port, int64_t _inData);
extern void Send_uint64_print(uint8_t _Port, uint64_t _unData);
extern void Send_float_print(uint8_t _Port, float _fData);
extern void Send_int_array_print(uint8_t _Port, int32_t* _pilData);
extern void Send_hex_print(uint8_t _Port, uint16_t _uData, uint8_t _Express);
extern void Send_lhex_print(uint8_t _Port, uint32_t _ulData, uint8_t _Express);
extern void Send_llhex_print(uint8_t _Port, uint64_t _unData, uint8_t _Express);
extern void User_printf(uint8_t _Port, char* _String, ...);



#endif /* __USER_UART_H_ */
/*********************************************************************************
* End of File
*********************************************************************************/

