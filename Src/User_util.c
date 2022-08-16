/* ###################################################################
**  Filename		: user_util.c
**  Title		: General purpose
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


/*********************************************************************************
* Includes
*********************************************************************************/
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "User_predefine.h" 
#include "User_util.h"


/*********************************************************************************
* Variables
*********************************************************************************/



/*********************************************************************************
* Functions
*********************************************************************************/
//-------------------------------------------------------------
// Function name    : GetAbitfromuint16_t
// Description      : unsigned int ���� 1��Ʈ ����
// Parameter        : x = ���� ������
//                    n = ã�� bit ��ġ
// Return           : (bool) ã�� ��
//-------------------------------------------------------------
bool GetAbitfromuint16_t(uint16_t x, int n)
{
  return (x & (1 << n)) >> n;
}

//-------------------------------------------------------------
// Function name	: Character_search
// Description		: ���ڿ����� ���� ��ġ ã��.
// Parameter		: _String = ���ڿ�
//					  _Character = ã�� ����
// Return			: (uint16_t) ���� ��ġ ��
//-------------------------------------------------------------
uint16_t Character_search(char* _String, char _Character)
{
  uint16_t	_cnt = 1;
  
  while(*_String)
  {
    if(*_String == _Character) return _cnt;
    else _cnt++;
    _String++;
  }
  return NULL;
}


//-------------------------------------------------------------
// Function name	: String_length
// Description		: ���ڿ� ����.
// Parameter		: _String = ���ڿ�
// Return			: (uint16_t) ���ڿ� ���� ��
//-------------------------------------------------------------
uint16_t String_length(char* _String)
{
  uint16_t	_cnt = 0;
  
  while(*_String)
  {
    _cnt++;
    _String++;
  }
  return _cnt;
}


//-------------------------------------------------------------
// Function name	: String_compare
// Description		: ���ڿ� ��.
// Parameter		: _String1 = ���ڿ�
//					  _String2 = ���ڿ�
// Return			: (uint16_t) TRUE or FALSE
//-------------------------------------------------------------
uint16_t String_compare(char* _String1, char* _String2)
{
  if(String_length(_String1) != String_length(_String2)) return FALSE;
  
  while(*_String1)
  {
    if(*_String1++ != *_String2++) return FALSE;
  }
  return TRUE;
}


//-------------------------------------------------------------
// Function name	: Strncpy
// Description		: ���ڿ� ���� ����.
// Parameter		: _String1 = �Է� ����
//					  _String2 = ���� ����
//					  _From = ���� ���� ��ġ
//					  _To = ���� �� ��ġ
// Return			: (uint16_t) TRUE or FALSE
//-------------------------------------------------------------
uint16_t Strncpy(char* _String1, char* _String2, uint16_t _uFrom, uint16_t _uTo)
{
  uint16_t	_cnt;
  
  if((_uTo - _uFrom) <= 0) return FALSE;
  if(_uTo < String_length(_String2)) return FALSE;
  
  _cnt = _uTo - _uFrom;
  _String2 += _uFrom;
  while(_cnt)
  {
    *_String1++ = *_String2++;
    _cnt--;
  }
  return TRUE;
}


//-------------------------------------------------------------
// Function name	: User_itoa
// Description		: int16 or uint16_t�� �����͸� ���ڿ��� ��ȯ.
// Parameter		: _ilValue = ������
//					  _String = ���ڿ� ����
// Return			: ����.
//-------------------------------------------------------------
void User_itoa(int32_t _ilValue, char* _String)
{
  uint16_t	_for_i, _cnt = 0, _cnt2 = 0;
  int32_t	_data = _ilValue;
  
  if(_ilValue < 0)
  {
    _String[_cnt++] = '-';
    _data = _ilValue * -1;
  }
  
  if(_data > 0)
  {
    while(_data)
    {
      _String[_cnt++] = NUMBER_ASCII_CODE + _data % 10;
      _data /= 10;
    }
  }
  else
  {
    _String[_cnt++] = NUMBER_ASCII_CODE;
  }
  _String[_cnt] = NULL;
  
  // ���ڿ��� �������� �����Ƿ� �յڸ� �ٲ�
  if(_ilValue > 0)
  {
    _for_i = 0;
    _cnt2 = _cnt;
    _cnt--;
  }
  else
  {
    _for_i = 1;
    _cnt2 = _cnt + 1;
  }
  for( ; _for_i < (_cnt2 / 2); _for_i++)
  {
    _String[_for_i] ^= _String[_cnt - _for_i];
    _String[_cnt - _for_i] ^= _String[_for_i];
    _String[_for_i] ^= _String[_cnt - _for_i];
  }
}


//-------------------------------------------------------------
// Function name	: User_ltoa
// Description		: int32_t or uint32_t or int64�� �����͸� ���ڿ��� ��ȯ.
// Parameter		: _inValue = ������
//					  _String = ���ڿ� ����
// Return			: ����.
//-------------------------------------------------------------
void User_ltoa(int64_t _inValue, char* _String)
{
  uint16_t	_for_i, _cnt = 0, _cnt2 = 0;
  int64_t	_data = _inValue;
  
  if(_inValue < 0)
  {
    _String[_cnt++] = '-';
    _data = _inValue * -1;
  }
  
  if(_data > 0)
  {
    while(_data)
    {
      _String[_cnt++] = NUMBER_ASCII_CODE + _data % 10;
      _data /= 10;
    }
  }
  else
  {
    _String[_cnt++] = NUMBER_ASCII_CODE;
  }
  _String[_cnt] = NULL;
  
  // ���ڿ��� �������� �����Ƿ� �յڸ� �ٲ�
  if(_inValue > 0)
  {
    _for_i = 0;
    _cnt2 = _cnt;
    _cnt--;
  }
  else
  {
    _for_i = 1;
    _cnt2 = _cnt + 1;
  }
  for( ; _for_i < (_cnt2 / 2); _for_i++)
  {
    _String[_for_i] ^= _String[_cnt - _for_i];
    _String[_cnt - _for_i] ^= _String[_for_i];
    _String[_for_i] ^= _String[_cnt - _for_i];
  }
}


//-------------------------------------------------------------
// Function name	: User_lltoa
// Description		: unsigned int64�� �����͸� ���ڿ��� ��ȯ.
// Parameter		: _unValue = ������
//					  _String = ���ڿ� ����
// Return			: ����.
//-------------------------------------------------------------
void User_lltoa(uint64_t _unValue, char* _String)
{
  uint16_t	_for_i, _cnt = 0;
  
  if(_unValue != 0)
  {
    while(_unValue)
    {
      _String[_cnt++] = NUMBER_ASCII_CODE + _unValue % 10;
      _unValue /= 10;
    }
  }
  else
  {
    _String[_cnt++] = NUMBER_ASCII_CODE;
  }
  _String[_cnt] = NULL;
  
  _cnt--;
  // ���ڿ��� �������� �����Ƿ� �յڸ� �ٲ�
  for(_for_i = 0; _for_i < ((_cnt + 1) / 2); _for_i++)
  {
    _String[_for_i] ^= _String[_cnt - _for_i];
    _String[_cnt - _for_i] ^= _String[_for_i];
    _String[_for_i] ^= _String[_cnt - _for_i];
  }
}


//-------------------------------------------------------------
// Function name	: User_ftoa
// Description		: float32�� �����͸� ���ڿ��� ��ȯ.
// Parameter		: _fValue = ������
//					  _String = ���ڿ� ����
// Return			: ����.
//-------------------------------------------------------------
void User_ftoa(float _fValue, char* _String)
{
  uint16_t	_for_i, _cnt = 0, _cnt2 = 0, _ecnt = 0, _Hdata, _outData;
  float _Ldata, _data = _fValue;
  
  if(_data < 0)
  {
    _String[_cnt++] = '-';
    _data = _fValue * -1.0;
  }
  
  _Hdata = (uint16_t)_data;
  _Ldata = _data - _Hdata;
  
  if(_Hdata > 0)
  {
    while(_Hdata)
    {
      _String[_cnt++] = NUMBER_ASCII_CODE + _Hdata % 10;
      _Hdata /= 10;
    }
    
    // ���ڿ��� �������� �����Ƿ� �յڸ� �ٲ�
    if(_fValue > 0)
    {
      _for_i = 0;
      _cnt2 = _cnt;
      _cnt--;
    }
    else
    {
      _for_i = 1;
      _cnt2 = _cnt + 1;
    }
    for( ; _for_i < (_cnt2 / 2); _for_i++)
    {
      _String[_for_i] ^= _String[_cnt - _for_i];
      _String[_cnt - _for_i] ^= _String[_for_i];
      _String[_for_i] ^= _String[_cnt - _for_i];
    }
    if(_fValue > 0) _cnt++;
  }
  else
  {
    _String[_cnt++] = '0';
  }
  
  if(_Ldata > 0)
  {
    _cnt2 = 0;
    _Hdata = 1;
    _String[_cnt++] = '.';
    while(_Ldata)
    {
      _Hdata *= 10;
      _outData = (uint16_t)(_Ldata * _Hdata) % 10;
      _String[_cnt++] = NUMBER_ASCII_CODE + _outData;
      
      _ecnt++;
      if(_outData == 0)
      {
        if(++_cnt2 >= 2)
        {
          if(++_cnt2 >= 2) _String[_cnt - 2] = NULL;
          break;
        }
      }
      else
      {
        _cnt2 = 0;
      }
      if(_ecnt >= 5) break;
    }
  }
  
  _String[_cnt] = NULL;
}


//-------------------------------------------------------------
// Function name	: User_atoi
// Description		: ���� ���ڿ��� uint16_t�� �����ͷ� ��ȯ.
// Parameter		: _String = ���� ���ڿ�
// Return			: (uint16_t) ������
//-------------------------------------------------------------
uint16_t User_atoi(char* _String)
{
  uint16_t	_cnt = 0;
  uint16_t	_uValue = 0;
  
  _cnt = String_length(_String);
  if(_cnt > 1)
  {
    --_cnt;
    while(*_String)
    {
      if((*_String < NUMBER_ASCII_CODE) && (*_String > 0x39)) return 0;
      
      _uValue += (*_String - NUMBER_ASCII_CODE) * User_pow(10, _cnt--);
      _String++;
    }
  }
  else _uValue = *_String - NUMBER_ASCII_CODE;
  
  return _uValue;
}


//-------------------------------------------------------------
// Function name	: User_atol
// Description		: ���� ���ڿ��� Uing32�� �����ͷ� ��ȯ.
// Parameter		: _String = ���� ���ڿ�
// Return			: (uint32_t) ������
//-------------------------------------------------------------
uint32_t User_atol(char* _String)
{
  uint16_t	_cnt = 0;
  uint32_t	_ulValue = 0;
  
  _cnt = String_length(_String);
  if(_cnt > 1)
  {
    --_cnt;
    while(*_String)
    {
      if((*_String < NUMBER_ASCII_CODE) && (*_String > 0x39)) return 0;
      
      _ulValue += (*_String - NUMBER_ASCII_CODE) * User_pow(10, _cnt--);
      _String++;
    }
  }
  else _ulValue = *_String - NUMBER_ASCII_CODE;
  
  return _ulValue;
}


//-------------------------------------------------------------
// Function name	: User_atoll
// Description		: ���� ���ڿ��� uint64_t�� �����ͷ� ��ȯ.
// Parameter		: _String = ���� ���ڿ�
// Return			: (uint64_t) ������
//-------------------------------------------------------------
uint64_t User_atoll(char* _String)
{
  uint16_t	_cnt = 0;
  uint64_t	_unValue = 0;
  
  _cnt = String_length(_String);
  if(_cnt > 1)
  {
    --_cnt;
    while(*_String)
    {
      if((*_String < NUMBER_ASCII_CODE) && (*_String > 0x39)) return 0;
      
      _unValue += (*_String - NUMBER_ASCII_CODE) * User_pow(10, _cnt--);
      _String++;
    }
  }
  else _unValue = *_String - NUMBER_ASCII_CODE;
  
  return _unValue;
}


//-------------------------------------------------------------
// Function name	: User_pow
// Description		: �ŵ� ����.
// Parameter		: _uValue = ����
//					  _uNum = ����
// Return			: (uint32_t) ������
//-------------------------------------------------------------
uint32_t User_pow(uint16_t _uValue, uint16_t _uNum)
{
  if( _uNum == 0 ) return 1;
  return _uValue * User_pow(_uValue, _uNum - 1);
}


//-------------------------------------------------------------
// Function name	: Value_Search
// Description		: �迭 �� ��ġ�ϴ� ������ �˻�.
// Parameter		: _uSize = �迭�� ũ��
//					  _uTarget_value = ã�� ��
//					  _puTarget_array_value = ������ �迭
// Return			: (uint16_t) ��ġ
//-------------------------------------------------------------
uint16_t Value_Search(uint16_t _uSize, uint16_t _uTarget_value, uint16_t* _puTarget_array_value)
{
  uint16_t	_for_i;
  uint16_t	_uTarget_position, _uData, _uValue = 100;
  
  for(_for_i = 0; _for_i < _uSize; _for_i++)
  {
    if(_uTarget_value < _puTarget_array_value[_for_i])
    {
      _uData = _puTarget_array_value[_for_i] - _uTarget_value;
      if(_uValue > _uData)
      {
        _uValue = _uData;
        _uTarget_position = _for_i;
      }
    }
    else if(_uTarget_value > _puTarget_array_value[_for_i])
    {
      _uData = _uTarget_value - _puTarget_array_value[_for_i];
      if(_uValue > _uData)
      {
        _uValue = _uData;
        _uTarget_position = _for_i;
      }
    }
    else
    {
      _uTarget_position = _for_i;
      break;
    }
  }
  
  return _uTarget_position;
}


//-------------------------------------------------------------
// Function name	: Binary_search
// Description		: �����˻�.
// Parameter		: _uTail = �迭�� ������ ��ġ
//					  _target_value = ã�� ��
//					  _pTarget_array_value = ������ �迭
// Return			: (uint16_t) ��ġ
//-------------------------------------------------------------
uint16_t Binary_search(uint16_t _uTail, uint16_t _uTarget_value, uint16_t* _puTarget_array_value)
{
  uint16_t	_uHead = 0;
  uint16_t	_uMid = 0;
  
  _uTail--;
  
  if(_uTarget_value < _puTarget_array_value[0])
    _uMid = _uHead;
  else if(_uTarget_value > _puTarget_array_value[_uTail])
    _uMid = _uTail;
  else
  {
    while(_uHead <= _uTail)
    {
      _uMid = ((_uHead + _uTail) / 2);
      if(_uTarget_value == _puTarget_array_value[_uMid])
      {
        break;
      }
      else if(_uTarget_value < _puTarget_array_value[_uMid])
      {
        _uTail = _uMid - 1;
        if(_uHead > _uTail) _uMid--;
      }
      else if(_uTarget_value > _puTarget_array_value[_uMid])
      {
        _uHead = _uMid + 1;
      }
    }
  }
  return _uMid;
}


//-------------------------------------------------------------
// Function name	: Array_average
// Description		: �迭 ���.
// Parameter		: _puArray = ������ �迭
//					  _uSize = �迭 ����
// Return			: (uint16_t) ��갪
//-------------------------------------------------------------
uint16_t Array_average(uint16_t* _puArray, uint16_t _uSize)
{
  uint16_t	_for_i;
  uint32_t	_ulSum = 0;
  
  for(_for_i = 0; _for_i < _uSize; _for_i++)
  {
    _ulSum += _puArray[_for_i];
  }
  
  return (_ulSum / _uSize);
}


//-------------------------------------------------------------
// Function name	: Curve
// Description		: 1���Լ�.
// Parameter		: _x_min = (uint16_t), _x_max = (uint16_t)
//					  _y_min = (uint16_t), _y_max = (uint16_t)
//					  _x_value = (uint16_t)
// Return			: (uint16_t) y value
//-------------------------------------------------------------
uint16_t Curve(uint16_t _X_min, uint16_t _X_max, uint16_t _Y_min, uint16_t _Y_max, uint16_t _X_value)
{
  float	_f_a;
  float	_f_b;
  
  _f_a = (_Y_max * 1.0 - _Y_min) / (_X_max - _X_min);
  _f_b = _Y_max - (_X_max * _f_a);
  return (uint16_t)((_f_a * _X_value) + _f_b);
}



/*********************************************************************************
* End of File
*********************************************************************************/

