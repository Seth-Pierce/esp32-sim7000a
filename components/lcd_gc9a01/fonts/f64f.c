//------------------------------------------------------------------------------
// This is Open source software. You can place this code on your site, but don't
// forget a link to my YouTube-channel: https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// ��� ����������� ����������� ���������������� ��������. �� ������ ���������
// ��� �� ����� �����, �� �� �������� ������� ������ �� ��� YouTube-�����
// "����������� � ���������" https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// �����: �������� ������ / Nadyrshin Ruslan
//------------------------------------------------------------------------------
#include "font.h"
#include "f64f.h"


// ������� �������� ������ �����
// ������ 2 ����� ������� - ������ � ������ (��� ������������ ������� - ���������)
const uint8_t f64f_table[f64f_NOFCHARS][144 + 2] = {
  // 0x30
  {
    24,
    f64_FLOAT_HEIGHT,
    ______XX,XX______,
    ___XXXXX,XXXXX___,
    _XXXXX__,__XXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    _XXXXX__,__XXXXX_,
    ___XXXXX,XXXXX___,
    ______XX,XX______}
  // 0x31
  ,{
    24,
    f64_FLOAT_HEIGHT,
    _______X,XXXXXXXX,
    ______XX,XXXXXXXX,
    ______XX,XXXXXXXX,
    _____XXX,XXXXXXXX,
    ____XXXX,XXXXXXXX,
    ____XXXX,XXXXXXXX,
    ___XXXXX,XXXXXXXX,
    __XXXXXX,XXXXXXXX,
    _XXXXXXX,XXXXXXXX,
    _XXXXXX_,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXX____,__XXXXXX,
    XX______,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX}
  // 0x32
  ,{
    24,
    f64_FLOAT_HEIGHT,
    _____XXX,XXX_____,
    ___XXXXX,XXXXX___,
    _XXXXXX_,_XXXXXX_,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,_XXXXXXX,
    ________,XXXXXXX_,
    _______X,XXXXXX__,
    ______XX,XXXXX___,
    _____XXX,XXXX____,
    ____XXXX,XXX_____,
    ___XXXXX,XX______,
    __XXXXXX,X_______,
    _XXXXXXX,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXXX_,________,
    XXXXXXXX,_____XXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX}
  ,{
    24,
    f64_FLOAT_HEIGHT,
    ______XX,XX______,
    ___XXXXX,XXXXX___,
    _XXXXXX_,_XXXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,__XXXXXX,
    ________,XXXXXXX_,
    _____XXX,XXXXX___,
    _____XXX,XXXX____,
    _____XXX,XXXX____,
    _____XXX,XXXXX___,
    ________,XXXXXXX_,
    ________,__XXXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    _XXXXXX_,_XXXXXX_,
    ___XXXXX,XXXXX___,
    ______XX,XX______}
  // 0x34
  ,{
    24,
    f64_FLOAT_HEIGHT,
    XXXXX___,XXXXXXX_,
    XXXXX___,XXXXXXX_,
    XXXXX___,XXXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXX___,_XXXXXX_,
    XXXXXX__,_XXXXXX_,
    XXXXXX__,_XXXXXX_,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    _XXXXXXX,XXXXXXXX,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_,
    ________,_XXXXXX_}
  // 0x35
  ,{
    24,
    f64_FLOAT_HEIGHT,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXX_,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXXX_,________,
    XXXXXXXX,XXXXXXX_,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    _XXXXXXX,XXXXXXXX,
    ________,XXXXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    ________,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    XXXXXX__,__XXXXXX,
    _XXXXXX_,_XXXXXX_,
    ___XXXXX,XXXXX___,
    _____XXX,XXX_____}
  // 0x36
  ,{
    24,
    f64_FLOAT_HEIGHT,
    ______XX,XX______,
    ___XXXXX,XXXXX___,
    _XXXXXX_,_XXXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXX___,________,
    XXXXXX__,________,
    XXXXXXX_,________,
    XXXXXXXX,X_______,
    XXXXXXXX,XXXX____,
    XXXXXXXX,XXXXX___,
    XXXXXXX_,_XXXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    _XXXXXX_,_XXXXXX_,
    ___XXXXX,XXXXX___,
    ______XX,XX______}
  // 0x37
  ,{
    24,
    f64_FLOAT_HEIGHT,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    XXXXXXXX,XXXXXXXX,
    ________,XXXXXXXX,
    _______X,XXXXXXX_,
    _______X,XXXXXX__,
    _______X,XXXXX___,
    ______XX,XXXX____,
    ______XX,XXXX____,
    ______XX,XXXX____,
    ______XX,XXXX____,
    _____XXX,XXX_____,
    _____XXX,XXX_____,
    _____XXX,XXX_____,
    ____XXXX,XX______,
    ____XXXX,XX______,
    ____XXXX,XX______,
    ____XXXX,XX______,
    ___XXXXX,X_______,
    ___XXXXX,X_______,
    ___XXXXX,X_______,
    ___XXXXX,X_______,
    __XXXXXX,________,
    __XXXXXX,________,
    __XXXXXX,________,
    __XXXXXX,________,
    __XXXXXX,________,
    _XXXXXX_,________,
    _XXXXXX_,________,
    _XXXXXX_,________,
    _XXXXXX_,________,
    _XXXXXX_,________,
    _XXXXXX_,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________,
    XXXXXX__,________}
  // 0x38
  ,{
    24,
    f64_FLOAT_HEIGHT,
    ______XX,XX______,
    ___XXXXX,XXXXX___,
    _XXXXXX_,_XXXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    _XXXXXX_,_XXXXXX_,
    ___XXXXX,XXXXX___,
    ____XXXX,XXXX____,
    ____XXXX,XXXX____,
    ___XXXXX,XXXXX___,
    _XXXXXX_,_XXXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    _XXXXXX_,_XXXXXX_,
    ___XXXXX,XXXXX___,
    ______XX,XX______}
  // 0x39
  ,{
    24,
    f64_FLOAT_HEIGHT,
    ______XX,XX______,
    ___XXXXX,XXXXX___,
    _XXXXXX_,_XXXXXX_,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    XXXXX___,___XXXXX,
    _XXXXXX_,_XXXXXXX,
    ___XXXXX,XXXXXXXX,
    ____XXXX,XXXXXXXX,
    _____XXX,XXXXXXXX,
    ________,XXXXXXXX,
    ________,_XXXXXXX,
    ________,__XXXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX,
    ________,___XXXXX}
};

const uint8_t f64f_table2[64 + 2] =
// 0x20
{
  8,
  f64_FLOAT_HEIGHT,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________
};

const uint8_t f64f_table3[64 + 2] =
// 0x2E
{
  8,
  f64_FLOAT_HEIGHT,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________
};

const uint8_t f64f_table4[64 + 2] =
// 0xB0
{
  15,
  f64_FLOAT_HEIGHT,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  _____XXX,XX______,
  ___XXXXX,XXXX____,
  __XXXXXX,XXXXX___,
  __XXXX__,_XXXX___,
  _XXXX___,__XXXX__,
  _XXXX___,__XXXX__,
  _XXXX___,__XXXX__,
  _XXXX___,__XXXX__,
  __XXXX__,_XXXX___,
  __XXXXXX,XXXXX___,
  ___XXXXX,XXXX____,
  _____XXX,XXX_____,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________
};

//==============================================================================
// ������� ���������� ��������� �� ���������� ������� Char
// ������ �����
//==============================================================================
uint8_t *f64f_GetCharTable(char Char)
{
  // ������ �����
  if ((Char >= 0x30) && (Char <= 0x39))
    return (uint8_t *)(&f64f_table[Char - 0x30][0]);
  if (Char == 0x20)
	  return (uint8_t *) f64f_table2;
  if (Char == 0x2E)
	  return (uint8_t *) f64f_table3;
  if (Char == 0xB0)
	  return (uint8_t *) f64f_table4;

  return 0;
}
//==============================================================================
