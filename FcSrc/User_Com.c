/**
 * @file User_Com.c
 * @brief 用户下位机通信模块
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-08
 *
 * THINK DIFFERENTLY
 */

#include "User_Com.h"

#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "Drv_Uart.h"
#include "LX_FC_State.h"

/**
 * @brief 用户协议数据获取,在串口中断中调用,解析完成后调用UserCom_DataAnl
 * @param  data             数据
 */
void UserCom_GetOneByte(u8 data) {
  static u8 _user_data_temp[50];
  static u8 _user_data_cnt = 0;
  static u8 _data_len = 0;
  static u8 state = 0;
  if (state == 0 && data == 0xAA) {
    state = 1;
    _user_data_temp[0] = data;
  } else if (state == 1 && data == 0x22) {
    state = 2;
    _user_data_temp[1] = data;
  } else if (state == 2)  //功能字
  {
    state = 3;
    _user_data_temp[2] = data;
  } else if (state == 3)  //长度
  {
    state = 4;
    _user_data_temp[3] = data;
    _data_len = data;  //数据长度
    _user_data_cnt = 0;
    // if (_data_len == 1) state = 5;
  } else if (state == 4 && _data_len > 0) {
    _data_len--;
    _user_data_temp[4 + _user_data_cnt++] = data;  //数据
    if (_data_len == 0) state = 5;
  } else if (state == 5) {
    state = 0;
    _user_data_temp[4 + _user_data_cnt] = data;  // check sum
    _user_data_temp[5 + _user_data_cnt] = 0;
    UserCom_DataAnl(_user_data_temp, 4 + _user_data_cnt);
  } else
    state = 0;
}

/**
 * @brief 用户命令解析执行,数据接收完成后自动调用
 * @param  data_buf         数据缓存
 * @param  data_len         数据长度
 */
void UserCom_DataAnl(u8* data_buf, u8 data_len) {
  static u8 option;
  static u8 sub_option;
  static u8 recv_check;
  static u8 calc_check;
  static u8 len;
  static u8 connected = 0;
  static u8* p_data;
  static float val1, val2, val3;
  static int16_t temp_s16;
  static int32_t temp_s32;
  p_data = (uint8_t*)(data_buf + 4);
  option = data_buf[2];
  sub_option = p_data[0];
  len = data_buf[3];
  recv_check = data_buf[4 + len];
  calc_check = 0;
  for (u8 i = 0; i < len + 4; i++) {
    calc_check += data_buf[i];
  }
  if (calc_check != recv_check) {
    LxPrintf("R: checksum error");
    return;
  }
  LxPrintf("R: %d %d", option, sub_option);
  switch (option) {
    case 0x00:  // 握手
      if (p_data[0] == 0x01) {
        connected = 1;
        LxPrintf("Ctrl Connected");
        break;
      }
    case 0x01:  // 流程控制
      if (sub_option == 0x10) {
      }
      break;
    case 0x02:  // 实时控制
      temp_s16 = p_data[1] << 8 | p_data[2];
      val1 = temp_s16 / 100.0f;
      temp_s16 = p_data[3] << 8 | p_data[4];
      val2 = temp_s16 / 100.0f;
      LxPrintf("R: sub_option:%d,val1:%s,val2:%s", sub_option,
               FloatToString(val1), FloatToString(val2));
      switch (sub_option) {
        case 0x01:
          break;
        case 0x02:
          break;
        case 0x03:
          break;
        case 0x04:
          break;
        case 0x05:
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}