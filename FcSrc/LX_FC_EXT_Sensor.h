#ifndef __LX_FC_EXT_SENSOR_H
#define __LX_FC_EXT_SENSOR_H

//==引用
#include "SysConfig.h"

//==定义/声明
//====通用传感器数据====
typedef struct {
  //
  vec3_s16 hca_velocity_cmps;

} __attribute__((__packed__)) _general_vel_st;

typedef struct {
  //
  vec3_s32 ulhca_pos_cm;

} __attribute__((__packed__)) _general_pos_st;

typedef struct {
  //
  u8 direction;
  u16 angle_100;
  s32 distance_cm;

} __attribute__((__packed__)) _general_dis_st;

typedef union {
  u8 byte[6];
  _general_vel_st st_data;
} _general_vel_un;

typedef union {
  u8 byte[12];
  _general_pos_st st_data;
} _general_pos_un;

typedef union {
  u8 byte[7];
  _general_dis_st st_data;
} _general_dis_un;

typedef struct {
  //
  _general_vel_un gen_vel;
  _general_pos_un gen_pos;
  _general_dis_un gen_dis;

} _fc_ext_sensor_st;
//==数据声明
extern _fc_ext_sensor_st ext_sens;
//==函数声明
// static

// public
void LX_FC_EXT_Sensor_Task(float dT_s);

#endif
