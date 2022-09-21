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
#include "Drv_Misc.h"
#include "Drv_Uart.h"
#include "Drv_WS2812.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"

void UserCom_DataAnl(u8* data_buf, u8 data_len);
void UserCom_DataExchange(void);
void UserCom_SendData(u8* dataToSend, u8 Length);
void UserCom_SendAck(u8 ack_data);
void UserCom_CalcAck(u8 option, u8* data_p, u8 data_len);

static u8 user_connected = 0;           //用户下位机是否连接
static u16 user_heartbeat_cnt = 0;      //用户下位机心跳计数
static u8 realtime_control_enable = 0;  //实时控制是否开启
static u16 realtime_control_cnt = 0;    //实时控制超时计数
static uint32_t pod_target_time = 0;    //吊舱目标时间
static uint32_t pod_start_time = 0;     //吊舱开始时间
static u8 pod_state = 0;                //吊舱状态
s16 user_pwm[4] = {0};                  //范围0-10000
_user_pos_st user_pos;                  //用户下位机位置数据
_to_user_un to_user_data;               //回传状态数据
_user_ack_st user_ack;                  // ACK数据

/**
 * @brief 用户协议数据获取,在串口中断中调用,解析完成后调用UserCom_DataAnl
 * @param  data             数据
 */
void UserCom_GetOneByte(u8 data) {
  static u8 _user_data_temp[128];
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
  static u8 suboption;
  static u8 recv_check;
  static u8 calc_check;
  static u8 len;
  static u8* p_data;
  static s16* p_s16;
  static s32* p_s32;
  static u32* p_u32;
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
    LxStringSend(LOG_COLOR_RED, "ERR: usercom checksum error");
    return;
  }
  switch (option) {
    case 0x00:  // 心跳包
      if (p_data[0] == 0x01) {
        if (!user_connected) {
          user_connected = 1;
          LxStringSend(LOG_COLOR_GREEN, "INFO: user connected");
        }
        user_heartbeat_cnt = 0;
        break;
      }
    case 0x01:  // 本地处理
      suboption = p_data[0];
      switch (suboption) {
        case 0x01:                  // WS2812控制
          if (p_data[4] == 0x11) {  // 帧结尾，确保接收完整
            u32_temp = 0xff000000;
            u8_temp = p_data[1];  // R
            u32_temp |= u8_temp << 16;
            u8_temp = p_data[2];  // G
            u32_temp |= u8_temp << 8;
            u8_temp = p_data[3];  // B
            u32_temp |= u8_temp;
            WS2812_SetAll(u32_temp);
            WS2812_SendBuf();
          }
          break;
        case 0x02:                   // 位置信息回传
          if (p_data[13] == 0x22) {  // 帧结尾，确保接收完整
            p_s32 = (s32*)(p_data + 1);
            user_pos.pos_x = *p_s32;
            p_s32++;
            user_pos.pos_y = *p_s32;
            p_s32++;
            user_pos.pos_z = *p_s32;
            user_pos.pos_update_cnt++;  // 触发发送
          }
          break;
        case 0x03:                  // 实时控制帧
          if (p_data[9] == 0x33) {  // 帧结尾，确保接收完整
            p_s16 = (s16*)(p_data + 1);
            rt_tar.st_data.vel_x = *p_s16;  // 头向速度，厘米每秒
            p_s16++;
            rt_tar.st_data.vel_y = *p_s16;  // 左向速度，厘米每秒
            p_s16++;
            rt_tar.st_data.vel_z = *p_s16;  // 天向速度，厘米每秒
            p_s16++;
            rt_tar.st_data.yaw_dps = *p_s16;  // 航向角速度，度每秒，逆时针为正
            dt.fun[0x41].WTS = 1;  // 触发发送
            //此处启用实时控制安全检查, 实时控制命令发送应持续发送且间隔小于1秒
            //超时会自动停止运动
            realtime_control_enable = 1;
            realtime_control_cnt = 0;
            if (rt_tar.st_data.vel_x == 0 && rt_tar.st_data.vel_y == 0 &&
                rt_tar.st_data.vel_z == 0 && rt_tar.st_data.yaw_dps == 0) {
              realtime_control_enable = 0;
            }
          }
          break;
        case 0x04:                       // 用户PWM控制
          if (p_data[4] == 0x44) {       // 帧结尾，确保接收完整
            u8_temp = p_data[1];         // 设置通道
            p_s16 = (s16*)(p_data + 2);  // 设置PWM值
            if (u8_temp <= 3) {          // 有效通道0-3
              user_pwm[u8_temp] = *p_s16;
              UserCom_CalcAck(0x01, p_data, 5);  // 重要的无反馈操作,需要ACK
            }
          }
          break;
        case 0x05:                  // IO控制
          if (p_data[3] == 0x55) {  // 帧结尾，确保接收完整
            DOut_Set(p_data[1], p_data[2]);
          }
          break;
        case 0x06:                  // 吊舱 控制
          if (p_data[6] == 0x66) {  // 帧结尾，确保接收完整
            pod_state = p_data[1];
            p_u32 = (u32*)(p_data + 2);
            pod_target_time = *p_u32;
            pod_start_time = GetSysRunTimeMs();
            UserCom_CalcAck(0x01, p_data, 7);  // 重要的无反馈操作,需要ACK
          }
          break;
      }
      break;
    case 0x02:  // 转发到IMU, 命令格式应遵循匿名通信协议, 此命令需要返回ACK
      if (dt.wait_ck == 0) {
        dt.cmd_send.CID = p_data[0];
        user_ack.ack_data = (0x02 + p_data[0]) % 0xFF;
        if (len > 11) {
          len = 11;  // 防止越界
        }
        for (u8 i = 0; i < len - 1; i++) {
          dt.cmd_send.CMD[i] = p_data[i + 1];
          user_ack.ack_data += p_data[i + 1];
        }
        CMD_Send(0xFF, &dt.cmd_send);
        user_ack.WTS = 1;  // 触发ACK
        // LxPrintf("DBG: to imu: 0x%02X 0x%02X 0x%02X", dt.cmd_send.CID,
        //          dt.cmd_send.CMD[0], dt.cmd_send.CMD[1]);
      } else {
        LxStringSend(LOG_COLOR_RED, "ERR: cmd to imu dropped for wait_ck");
      }
      break;
    default:
      break;
  }
}

void UserCom_CalcAck(u8 option, u8* data_p, u8 data_len) {
  user_ack.ack_data = option;
  for (u8 i = 0; i < data_len; i++) {
    user_ack.ack_data += data_p[i];
  }
  user_ack.WTS = 1;
}

/**
 * @brief 用户通讯持续性任务，在调度器中调用
 * @param  dT_s
 */
void UserCom_Task(float dT_s) {
  static u16 data_exchange_cnt = 0;
  if (user_connected) {
    //心跳超时检查
    user_heartbeat_cnt++;
    if (user_heartbeat_cnt * dT_s >= USER_HEARTBEAT_TIMEOUT_S) {
      user_connected = 0;
      LxStringSend(LOG_COLOR_RED, "WARN: user disconnected");
      if (fc_sta.unlock_sta == 1) {  //如果是解锁状态，则采取安全措施
        // OneKey_Land(); //降落
        OneKey_Stable();  //恢复悬停
        realtime_control_enable = 0;
      }
    }

    // ACK发送检查
    if (user_ack.WTS == 1) {
      user_ack.WTS = 0;
      UserCom_SendAck(user_ack.ack_data);
      user_ack.ack_data = 0;
    }

    //数据交换
    data_exchange_cnt++;
    if (data_exchange_cnt * dT_s >= USER_DATA_EXCHANGE_TIMEOUT_S) {
      data_exchange_cnt = 0;
      UserCom_DataExchange();
    }

    //实时控制安全检查
    if (fc_sta.fc_mode_sta == 3) {
      // 程控模式, 实时控制失效
      realtime_control_enable = 0;
      realtime_control_cnt = 0;
      rt_tar.st_data.vel_x = 0;
      rt_tar.st_data.vel_y = 0;
      rt_tar.st_data.vel_z = 0;
      rt_tar.st_data.yaw_dps = 0;
      dt.fun[0x41].WTS = 1;  // 触发发送
    }
    if (realtime_control_enable) {
      realtime_control_cnt++;
      if (realtime_control_cnt * dT_s >= REALTIME_CONTROL_TIMEOUT_S) {
        //超时, 停止运动
        realtime_control_cnt = 0;
        realtime_control_enable = 0;
        rt_tar.st_data.vel_x = 0;
        rt_tar.st_data.vel_y = 0;
        rt_tar.st_data.vel_z = 0;
        rt_tar.st_data.yaw_dps = 0;
        dt.fun[0x41].WTS = 1;  // 触发发送
        LxStringSend(LOG_COLOR_RED, "WARN: realtime control timeout");
      }
    }

    // 吊舱状态检测
    if (pod_state == 0x01) {  //放线开始
      user_pwm[1] = 7000;
      if (GetSysRunTimeMs() - pod_start_time > 1000) {  //超时
        pod_state = 0x03;
      }
    } else if (pod_state == 0x02) {  //收线
      if (Button_Get(0x02) == 0) {
        user_pwm[1] = 4800;
      }
      if (GetSysRunTimeMs() - pod_start_time > pod_target_time) {  //超时
        pod_state = 0x00;
        user_pwm[1] = 6000;
      } else if (Button_Get(0x02) == 1) {  //限位按钮按下
        pod_state = 0x00;
        user_pwm[1] = 6000;
      }
    } else if (pod_state == 0x03) {  //放线等待
      user_pwm[1] = 7000;
      if (GetSysRunTimeMs() - pod_start_time > pod_target_time) {  //超时
        pod_state = 0x00;
        user_pwm[1] = 6000;
      } else if (Button_Get(0x02) == 1) {  //限位按钮按下
        pod_state = 0x00;
        user_pwm[1] = 6000;
      }
    }
  }
}

/**
 * @brief 交换飞控数据
 */
void UserCom_DataExchange(void) {
  static u8 user_data_size = sizeof(to_user_data.byte_data);

  // 初始化数据
  to_user_data.st_data.head1 = 0xAA;
  to_user_data.st_data.head2 = 0x55;
  to_user_data.st_data.length = user_data_size - 4;
  to_user_data.st_data.cmd = 0x01;

  // 数据赋值
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
  to_user_data.st_data.CMD_0 = fc_sta.cmd_fun.CMD_0;
  to_user_data.st_data.CMD_1 = fc_sta.cmd_fun.CMD_1;

  // 校验和
  to_user_data.st_data.check_sum = 0;
  for (u8 i = 0; i < user_data_size - 1; i++) {
    to_user_data.st_data.check_sum += to_user_data.byte_data[i];
  }

  UserCom_SendData(to_user_data.byte_data, user_data_size);
}

static u8 data_to_send[12];

void UserCom_SendAck(u8 ack_data) {
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
 * @brief 发送事件
 * @param  event            事件代码
 * @param  op               操作代码
 */
void UserCom_SendEvent(u8 event, u8 op) {
  data_to_send[0] = 0xAA;   // head1
  data_to_send[1] = 0x55;   // head2
  data_to_send[2] = 0x03;   // length
  data_to_send[3] = 0x03;   // cmd
  data_to_send[4] = event;  // event code
  data_to_send[5] = op;     // op code
  data_to_send[6] = 0;      // check_sum
  for (u8 i = 0; i < 6; i++) {
    data_to_send[6] += data_to_send[i];
  }
  UserCom_SendData(data_to_send, 7);
}

/**
 * @brief 用户通讯数据发送
 */
void UserCom_SendData(u8* dataToSend, u8 Length) {
  DrvUart2SendBuf(dataToSend, Length);
}