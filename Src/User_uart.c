/* ###################################################################
**  Filename		: user_uart.c
**  Title		: uart print function implementation for debug
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


/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for memset
#include "stm32g4xx_ll_usart.h" 

#include "User_predefine.h"
#include "User_uart.h"
#include "User_util.h"




/*********************************************************************************
* Variables
*********************************************************************************/
static uint8_t	_uHex_string[] = "0123456789ABCDEF";
bool	Print_flag = TRUE;
Uart_t	stSci_Debug;



/*********************************************************************************
* Functions
*********************************************************************************/
//-------------------------------------------------------------
// Function name	: Init_Sci_Variable
// Description		: Sci ?? ???.
// Parameter		: _pstSci = Uart
//			  _uPort = SCI 
// Return		: none
//-------------------------------------------------------------
void Init_Sci_Variable(Uart_t* _pstSci, uint16_t _uPort)
{
  _pstSci->bRX_flag = FALSE;
  _pstSci->bTX_flag = FALSE;
  _pstSci->bRX_Buffer_over_flag = FALSE;
  _pstSci->bTX_Buffer_over_flag = FALSE;
  _pstSci->uPort = _uPort;
  _pstSci->uRX_Buffer_index = 0;
  _pstSci->uTX_Buffer_index = 0;
  memset(_pstSci->cRX_Buffer, 0, sizeof(_pstSci->cRX_Buffer));
  memset(_pstSci->cTX_Buffer, 0, sizeof(_pstSci->cTX_Buffer));
}


//-------------------------------------------------------------
// Function name	: Init_User_UART
// Description		: SCI ? ?? ???.
// Parameter		: ??.
// Return		: ??.
//-------------------------------------------------------------
void Init_User_UART(void)
{
#if SCI_DCM_ACTIVE
  // USART1
#endif
#if SCI_DEBUG_ACTIVE
  // USART2
  LL_USART_EnableIT_RXNE(USART2);
  Init_Sci_Variable(&stSci_Debug, SCI_DEBUG);
#endif
}

//-------------------------------------------------------------
// Function name	: Send_char_print
// Description		: RS232포트로 1byte 문자 출력.
// Parameter		: _port = 출력포트
//			  _char = 1byte 문자
// Return		: 없음.
//-------------------------------------------------------------
void Send_char_print(uint8_t _Port, char _Char)
{  
  switch(_Port)
  {
    case	SCI_MOTOR :
      while(RESET == LL_USART_IsActiveFlag_TXE(USART1));
      LL_USART_TransmitData8(USART2, _Char);
      break;
    case	SCI_DEBUG :
      while(RESET == LL_USART_IsActiveFlag_TXE(USART2));
      LL_USART_TransmitData8(USART2, _Char);
      break;
    default :
      break;
  }
}


//-------------------------------------------------------------
// Function name	: Send_string_print
// Description		: RS232포트로 문자열 출력.
// Parameter		: _port = 출력포트
//			  _string = 문자열
// Return		: 없음.
//-------------------------------------------------------------
void Send_string_print(uint8_t _Port, char* _String)
{
  while(*_String)
  {
    Send_char_print(_Port, *_String);
    _String++;
  }
}


//-------------------------------------------------------------
// Function name	: Send_string_char_print
// Description		: RS232포트로 문자열 출력.
// Parameter		: _port = 출력포트
//					  _size = 크기
//					  _string = 문자열
// Return		: 없음.
//-------------------------------------------------------------
void Send_string_char_print(uint8_t _Port, uint8_t _size, uint8_t* _String)
{
  uint8_t _cnt = 0;
  
  while(_size != _cnt)
  {
    Send_char_print(_Port, *_String);
    _String++;
    _cnt++;
  }
}


//-------------------------------------------------------------
// Function name	: Send_int_print
// Description		: RS232포트로 int16형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _iData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_int_print(uint8_t _Port, int16_t _iData)
{
  char _string[20];
  
  User_itoa(_iData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_uint_print
// Description		: RS232포트로 unsigned int16형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _uData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_uint_print(uint8_t _Port, uint16_t _uData)
{
  char _string[20];
  
  User_itoa(_uData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_long_print
// Description		: RS232포트로  int32형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _ilData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_long_print(uint8_t _Port, int32_t _ilData)
{
  char _string[20];
  
  User_ltoa(_ilData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_ulong_print
// Description		: RS232포트로 unsigned int32형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _ulData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_ulong_print(uint8_t _Port, uint32_t _ulData)
{
  char _string[20];
  
  User_ltoa(_ulData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_int64_print
// Description		: RS232포트로 int64형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _inData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_int64_print(uint8_t _Port, int64_t _inData)
{
  char _string[20];
  
  User_ltoa(_inData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_uint64_print
// Description		: RS232포트로 unsigned int64형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _unData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_uint64_print(uint8_t _Port, uint64_t _unData)
{
  char _string[20];
  
  User_lltoa(_unData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_float_print
// Description		: RS232포트로 float32형 데이터 출력.
// Parameter		: _port = 출력포트
//					  _fData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_float_print(uint8_t _Port, float _fData)
{
  char _string[20];
  
  User_ftoa(_fData, _string);
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_int_array_print
// Description		: RS232포트로 long unsigned int형 array 데이터 출력.
// Parameter		: _port = 출력포트
//					  _pilData = 데이터
// Return		: 없음.
//-------------------------------------------------------------
void Send_uint_array_print(uint8_t _Port, uint32_t* _pulData)
{
  uint16_t _size = 0, _for_i;
  
  _size = sizeof(_pulData);
  for(_for_i = 0; _for_i < _size; _for_i++)
  {
    Send_int_print(_Port, _pulData[_for_i]);
  }
}


//-------------------------------------------------------------
// Function name	: Send_hex_print
// Description		: RS232포트로 unsigned int형 데이터를 HEX 값으로 출력.
// Parameter		: _port = 출력포트
//					  _uData = 데이터
//					  _Express = 0x 표현
// Return		: 없음.
//-------------------------------------------------------------
void Send_hex_print(uint8_t _Port, uint16_t _uData, uint8_t _Express)
{
  char _string[7];
  uint8_t	_for_i = 0, _cnt, _position;
  
  if(_Express)
  {
    _string[_for_i++] = '0';
    _string[_for_i++] = 'x';
  }
  
  if(_uData > 0xFF) { _position = 4 + _for_i; _cnt = 16; }
  else			  { _position = 2 + _for_i; _cnt = 8; }
  
  for( ; _for_i < _position; _for_i++)
  {
    _string[_for_i] = _uHex_string[(_uData >> (_cnt -= 4)) & 0xF];
  }
  _string[_for_i] = NULL;
  
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_lhex_print
// Description		: RS232포트로 unsigned long형 데이터를 HEX 값으로 출력.
// Parameter		: _port = 출력포트
//					  _ulData = 데이터
//					  _Express = 0x 표현
// Return		: 없음.
//-------------------------------------------------------------
void Send_lhex_print(uint8_t _Port, uint32_t _ulData, uint8_t _Express)
{
  char _string[11];
  uint8_t	_for_i = 0, _cnt, _position;
  
  if(_Express)
  {
    _string[_for_i++] = '0';
    _string[_for_i++] = 'x';
  }
  
  if	   (_ulData > 0xFFFFFF) { _position = 8 + _for_i; _cnt = 32; }
  else if(_ulData > 0xFFFF)	{ _position = 6 + _for_i; _cnt = 24; }
  else if(_ulData > 0xFF)		{ _position = 4 + _for_i; _cnt = 16; }
  else				 		{ _position = 2 + _for_i; _cnt = 8; }
  
  for( ; _for_i < _position; _for_i++)
  {
    _string[_for_i] = _uHex_string[(_ulData >> (_cnt -= 4)) & 0xF];
  }
  _string[_for_i] = NULL;
  
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: Send_llhex_print
// Description		: RS232포트로 unsigned long long형 데이터를 HEX 값으로 출력.
// Parameter		: _port = 출력포트
//					  _unData = 데이터
//					  _Express = 0x 표현
// Return		: 없음.
//-------------------------------------------------------------
void Send_llhex_print(uint8_t _Port, uint64_t _unData, uint8_t _Express)
{
  char _string[19];
  uint8_t	_for_i = 0, _cnt, _position;
  
  if(_Express)
  {
    _string[_for_i++] = '0';
    _string[_for_i++] = 'x';
  }
  
  if     (_unData > 0xFFFFFFFFFFFFFF) { _position = 16 + _for_i; _cnt = 64; }
  else if(_unData > 0xFFFFFFFFFFFF)	{ _position = 14 + _for_i; _cnt = 56; }
  else if(_unData > 0xFFFFFFFFFF)		{ _position = 12 + _for_i; _cnt = 48; }
  else if(_unData > 0xFFFFFFFF)		{ _position = 10 + _for_i; _cnt = 40; }
  else if(_unData > 0xFFFFFF)			{ _position = 8  + _for_i; _cnt = 32; }
  else if(_unData > 0xFFFF)			{ _position = 6  + _for_i; _cnt = 24; }
  else if(_unData > 0xFF)				{ _position = 4  + _for_i; _cnt = 16; }
  else				 				{ _position = 2  + _for_i; _cnt = 8; }
  
  for( ; _for_i < _position; _for_i++)
  {
    _string[_for_i] = _uHex_string[(_unData >> (_cnt -= 4)) & 0xF];
  }
  _string[_for_i] = NULL;
  
  Send_string_print(_Port, _string);
}


//-------------------------------------------------------------
// Function name	: User_printf
// Description		: RS232포트로 여러 형 데이터를 출력.
// Parameter		: _port = 출력포트
//					  _String = 문자열
//					  ... 가변인자
// Return		: 없음.
//-------------------------------------------------------------
void User_printf(uint8_t _Port, char* _String, ...)
{
  char			_arg_char = 0;
  char*			_arg_string = 0;
  int			_arg_int16 = 0;
  unsigned int		_arg_uint16 = 0;
  long			_arg_int32 = 0;
  unsigned long		_arg_uint32 = 0;
  float			_arg_float32 = 0;
  long long		_arg_int64 = 0;
  unsigned long long	_arg_uint64 = 0;
  
  va_list	_list;
  
  if(Print_flag)
  {
    va_start(_list, _String);
    
    while(_String[0] != NULL)
    {
      if(_String[0] == '%')
      {
        switch(_String[1])
        {
        case 'd':
          _arg_int16 = va_arg(_list, int);
          Send_int_print(_Port, _arg_int16);
          _String += 2;
          break;
        case 'u':
          _arg_uint16 = va_arg(_list, unsigned int);
          Send_uint_print(_Port, _arg_uint16);
          _String += 2;
          break;
        case 'x':
          _arg_uint16 = va_arg(_list, unsigned int);
          Send_hex_print(_Port, _arg_uint16, 1);
          _String += 2;
          break;
        case 'z':
          _arg_uint16 = va_arg(_list, unsigned int);
          Send_hex_print(_Port, _arg_uint16, 0);
          _String += 2;
          break;
        case 'c':
          _arg_char = va_arg(_list, unsigned int);
          Send_char_print(_Port, _arg_char);
          _String += 2;
          break;
        case 's':
          _arg_string = va_arg(_list, char*);
          Send_string_print(_Port, _arg_string);
          _String += 2;
          break;
        case 'f':
          //_arg_float32 = va_arg(_list, float);
          _arg_float32 = va_arg(_list, double);
          Send_float_print(_Port, _arg_float32);
          _String += 2;
          break;
        case 'l':
          switch(_String[2])
          {
          case 'd':
            _arg_int32 = va_arg(_list, long);
            Send_long_print(_Port, _arg_int32);
            _String += 3;
            break;
          case 'u':
            _arg_uint32 = va_arg(_list, unsigned long);
            Send_ulong_print(_Port, _arg_uint32);
            _String += 3;
            break;
          case 'x':
            _arg_uint32 = va_arg(_list, unsigned long);
            Send_lhex_print(_Port, _arg_uint32, 1);
            _String += 3;
            break;
          case 'z':
            _arg_uint16 = va_arg(_list, unsigned long);
            Send_lhex_print(_Port, _arg_uint32, 0);
            _String += 3;
            break;
          case 'l':
            switch(_String[3])
            {
            case 'd':
              _arg_uint64 = va_arg(_list, long long);
              Send_int64_print(_Port, _arg_int64);
              _String += 4;
              break;
            case 'u':
              _arg_uint64 = va_arg(_list, unsigned long long);
              Send_uint64_print(_Port, _arg_uint64);
              _String += 4;
              break;
            case 'x':
              _arg_uint64 = va_arg(_list, unsigned long long);
              Send_llhex_print(_Port, _arg_uint64, 1);
              _String += 4;
              break;
            case 'z':
              _arg_uint64 = va_arg(_list, unsigned long long);
              Send_llhex_print(_Port, _arg_uint64, 0);
              _String += 4;
              break;
            default:
              Send_char_print(_Port, _String[3]);
              _String++;
              break;
            }
            break;
          default:
            Send_char_print(_Port, _String[2]);
            _String++;
            break;
          }
          break;
        default:
          Send_char_print(_Port, _String[1]);
          _String++;
          break;
        }
      }
      else
      {
        Send_char_print(_Port, _String[0]);
        _String++;
      }
    }
    
    va_end(_list);
  }
}



/*********************************************************************************
* End of File
*********************************************************************************/

