/**
 * @file User_Com.h
 * @brief see User_Com.c
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-08
 *
 * THINK DIFFERENTLY
 */

#ifndef _USER_COM_H
#define _USER_COM_H

#include "SysConfig.h"

#define USER_DATA_EXCHANGE_TIMEOUT_S (0.05f - 0.001f)
#define USER_HEARTBEAT_TIMEOUT_S (1.0f - 0.001f)
#define REALTIME_CONTROL_TIMEOUT_S (1.0f - 0.001f)

//事件代码
#define USER_EVENT_KEY_SHORT 0x01
#define USER_EVENT_KEY_LONG 0x02
#define USER_EVENT_KEY_DOUBLE 0x03
//事件操作
#define USER_EVENT_OP_SET 0x01
#define USER_EVENT_OP_CLEAR 0x02

extern s16 user_pwm[4];

//回传数据结构
typedef struct {
  u8 head1;
  u8 head2;
  u8 length;
  //
  u8 cmd;
  //
  s16 rol_x100;
  s16 pit_x100;
  s16 yaw_x100;
  s32 alt_fused;
  s32 alt_add;
  s16 vel_x;
  s16 vel_y;
  s16 vel_z;
  s32 pos_x;
  s32 pos_y;
  u16 voltage_100;
  u8 fc_mode_sta;
  u8 unlock_sta;
  u8 CID;
  u8 CMD_0;
  u8 CMD_1;
  //
  u8 check_sum;
} __attribute__((__packed__)) _to_user_st;

typedef union {
  u8 byte_data[40];
  _to_user_st st_data;
} _to_user_un;

// ACK数据结构
typedef struct {
  u8 ack_data;
  u8 WTS;
} __attribute__((__packed__)) _user_ack_st;

//通用位置数据结构
typedef struct {
  u8 pos_update_cnt;
  //
  s32 pos_x;
  s32 pos_y;
  s32 pos_z;
} __attribute__((__packed__)) _user_pos_st;

extern _user_pos_st user_pos;

void UserCom_GetOneByte(u8 data);

void UserCom_Task(float dT_s);

void UserCom_SendEvent(u8 event, u8 op);

#endif