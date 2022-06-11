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

//�ش����ݽṹ
typedef struct {
  u8 head1;
  u8 head2;
  u8 length;
  s16 rol_x100;
  s16 pit_x100;
  s16 yaw_x100;
  s32 alt_fused;
  s16 vel_x;
  s16 vel_y;
  s16 vel_z;
  s32 pos_x;
  s32 pos_y;
  u16 voltage_100;
  u8 fc_mode_sta;
  u8 unlock_sta;
  u8 CID;
  u8 check_sum;
} __attribute__((__packed__)) _to_user_st;

typedef union {
  u8 byte_data[33];
  _to_user_st st_data;
} _to_user_un;


void UserCom_GetOneByte(u8 data);

void UserCom_Task(float dT_s);
#endif