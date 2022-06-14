/**
 * @file User_Com.c
 * @brief �û���λ��ͨ��ģ��
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
#include "Drv_WS2812.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"

void UserCom_DataAnl(u8* data_buf, u8 data_len);
void UserCom_DataExchange(void);
void UserCom_SendData(u8* dataToSend, u8 Length);
void UserCom_SendAck(u8 ack_data);

static u8 user_connected = 0;       //�û���λ���Ƿ�����
static u16 user_heartbeat_cnt = 0;  //�û���λ����������
_user_pos_st user_pos;              //�û���λ��λ������

_to_user_un to_user_data;
_user_ack_st user_ack;

/**
 * @brief �û�Э�����ݻ�ȡ,�ڴ����ж��е���,������ɺ����UserCom_DataAnl
 * @param  data             ����
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
  } else if (state == 2)  //������
  {
    state = 3;
    _user_data_temp[2] = data;
  } else if (state == 3)  //����
  {
    state = 4;
    _user_data_temp[3] = data;
    _data_len = data;  //���ݳ���
    _user_data_cnt = 0;
    // if (_data_len == 1) state = 5;
  } else if (state == 4 && _data_len > 0) {
    _data_len--;
    _user_data_temp[4 + _user_data_cnt++] = data;  //����
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
 * @brief �û��������ִ��,���ݽ�����ɺ��Զ�����
 * @param  data_buf         ���ݻ���
 * @param  data_len         ���ݳ���
 */
void UserCom_DataAnl(u8* data_buf, u8 data_len) {
  static u8 option;
  static u8 suboption;
  static u8 recv_check;
  static u8 calc_check;
  static u8 len;
  static u8* p_data;
  static s32* p_s32;
  static u8 u8_temp;
  static u32 u32_temp;

  p_data = (uint8_t*)(data_buf + 4);
  option = data_buf[2];
  len = data_buf[3];
  recv_check = data_buf[data_len];
  calc_check = 0;
  for (u8 i = 0; i < len + 4; i++) {
    calc_check += data_buf[i];
  }
  if (calc_check != recv_check) {
    LxPrintf("DBG: usercom checksum error");
    return;
  }
  switch (option) {
    case 0x00:  // ������
      if (p_data[0] == 0x01) {
        if (!user_connected) {
          user_connected = 1;
          LxPrintf("DBG: user connected");
        }
        user_heartbeat_cnt = 0;
        break;
      }
    case 0x01:  // ����32(Ԥ��)
      suboption = p_data[0];
      switch (suboption) {
        case 0x01:  // WS2812����
          u32_temp = 0xff000000;
          u8_temp = p_data[1];  // R
          u32_temp |= u8_temp << 16;
          u8_temp = p_data[2];  // G
          u32_temp |= u8_temp << 8;
          u8_temp = p_data[3];  // B
          u32_temp |= u8_temp;
          WS2812_SetAll(u32_temp);
          WS2812_SendBuf();
          break;
        case 0x02:  // λ����Ϣ�ش�
          p_s32 = (s32*)(p_data + 1);
          user_pos.pos_x = *p_s32;
          p_s32++;
          user_pos.pos_y = *p_s32;
          p_s32++;
          user_pos.pos_z = *p_s32;
          user_pos.pos_update_cnt++;
          break;
      }
      break;
    case 0x02:  // ת����IMU, �����ʽӦ��ѭ����ͨ��Э��, ��������Ҫ����ACK
      if (dt.wait_ck == 0) {
        dt.cmd_send.CID = p_data[0];
        for (u8 i = 0; i < len + 1; i++) {
          dt.cmd_send.CMD[i] = p_data[i + 1];
        }
        CMD_Send(0xFF, &dt.cmd_send);
        user_ack.ack_data = 0x02 + p_data[0];
        user_ack.WTS = 1;  // ����ACK
        LxPrintf("DBG: to imu: 0x%02X 0x%02X 0x%02X", dt.cmd_send.CID,
                 dt.cmd_send.CMD[0], dt.cmd_send.CMD[1]);
      } else {
        LxPrintf("DBG: cmd to imu dropped for wait_ck");
      }
      break;
    default:
      break;
  }
}

/**
 * @brief �û�ͨѶ�����������ڵ������е���
 * @param  dT_s
 */
void UserCom_Task(float dT_s) {
  static u16 data_exchange_cnt = 0;
  if (user_connected) {
    //������ʱ���
    user_heartbeat_cnt++;
    if (user_heartbeat_cnt * dT_s >= USER_HEARTBEAT_TIMEOUT_S) {
      user_connected = 0;
      LxPrintf("DBG: user disconnected");
      if (fc_sta.unlock_sta == 1) {  //����ǽ���״̬�����ȡ��ȫ��ʩ
        // OneKey_Land(); //����
        OneKey_Stable();  //�ָ���ͣ
      }
    }

    // ACK���ͼ��
    if (user_ack.WTS == 1) {
      user_ack.WTS = 0;
      UserCom_SendAck(user_ack.ack_data);
    }

    //���ݽ���
    data_exchange_cnt++;
    if (data_exchange_cnt * dT_s >= USER_DATA_EXCHANGE_TIMEOUT_S) {
      data_exchange_cnt = 0;
      UserCom_DataExchange();
    }
  }
}

/**
 * @brief �����ɿ�����
 */
void UserCom_DataExchange(void) {
  static u8 user_data_size = sizeof(to_user_data.byte_data);

  // ��ʼ������
  to_user_data.st_data.head1 = 0xAA;
  to_user_data.st_data.head2 = 0x55;
  to_user_data.st_data.length = user_data_size - 4;
  to_user_data.st_data.cmd = 0x01;

  // ���ݸ�ֵ
  to_user_data.st_data.rol_x100 = fc_att.st_data.rol_x100;
  to_user_data.st_data.pit_x100 = fc_att.st_data.pit_x100;
  to_user_data.st_data.yaw_x100 = fc_att.st_data.yaw_x100;
  to_user_data.st_data.alt_fused = fc_alt.st_data.alt_fused;
  to_user_data.st_data.alt_add = fc_alt.st_data.alt_add;
  to_user_data.st_data.vel_x = fc_vel.st_data.vel_x;
  to_user_data.st_data.vel_y = fc_vel.st_data.vel_y;
  to_user_data.st_data.vel_z = fc_vel.st_data.vel_z;
  to_user_data.st_data.pos_x = fc_pos.st_data.pos_x;
  to_user_data.st_data.pos_y = fc_pos.st_data.pos_y;
  to_user_data.st_data.voltage_100 = fc_bat.st_data.voltage_100;
  to_user_data.st_data.fc_mode_sta = fc_sta.fc_mode_sta;
  to_user_data.st_data.unlock_sta = fc_sta.unlock_sta;
  to_user_data.st_data.CID = fc_sta.cmd_fun.CID;

  // У���
  to_user_data.st_data.check_sum = 0;
  for (u8 i = 0; i < user_data_size - 1; i++) {
    to_user_data.st_data.check_sum += to_user_data.byte_data[i];
  }

  UserCom_SendData(to_user_data.byte_data, user_data_size);
}

void UserCom_SendAck(u8 ack_data) {
  static u8 data_to_send[6];
  data_to_send[0] = 0xAA;      // head1
  data_to_send[1] = 0x55;      // head2
  data_to_send[2] = 0x02;      // length
  data_to_send[3] = 0x02;      // cmd
  data_to_send[4] = ack_data;  // data
  data_to_send[5] = 0;         // check_sum
  for (u8 i = 0; i < 5; i++) {
    data_to_send[5] += data_to_send[i];
  }
  UserCom_SendData(data_to_send, 6);
}

/**
 * @brief �û�ͨѶ���ݷ���
 */
void UserCom_SendData(u8* dataToSend, u8 Length) {
  DrvUart2SendBuf(dataToSend, Length);
}