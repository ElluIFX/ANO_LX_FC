#include "User_Task.h"

#include "Drv_PwmOut.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"

void UserTask_OneKeyCmd(void) {
  //////////////////////////////////////////////////////////////////////
  //一键起飞/降落例程
  //////////////////////////////////////////////////////////////////////
  //用静态变量记录一键起飞/降落指令已经执行。
  static u8 one_key_takeoff_f = 1, one_key_land_f = 1;
  static u8 emergency_stop_f = 1;
  //判断有遥控信号才执行
  if (rc_in.fail_safe == 0) {
    //判断第6通道拨杆位置 1300<CH_6<1700
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700) {
    } else {
    }
    //
    //判断第6通道拨杆位置 800<CH_6<1200
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200) {
      //还没有执行
      if (one_key_land_f == 0) {
        //标记已经执行
        one_key_land_f =
            //执行一键降落
            OneKey_Land();
      }
    } else {
      //复位标记，以便再次执行
      one_key_land_f = 0;
    }
    //判断第6通道拨杆位置 1700<CH_6<2000
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200) {
      //还没有执行
      if (one_key_takeoff_f == 0) {
        //标记已经执行
        one_key_takeoff_f =
            //执行一键起飞
            OneKey_Takeoff(100);  //参数单位：厘米； 0：默认上位机设置的高度。
      }
    } else {
      //复位标记，以便再次执行
      one_key_takeoff_f = 0;
    }
    ////////////////////////////////////////////////////////////////////////
  }
  //第七通道
  if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1700 &&
      rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2200) {
    if (emergency_stop_f == 0) {
      emergency_stop_f = 1;
      //执行一键锁桨
      FC_Lock();
      pwm_to_esc.pwm_m1 = 0;
      pwm_to_esc.pwm_m2 = 0;
      pwm_to_esc.pwm_m3 = 0;
      pwm_to_esc.pwm_m4 = 0;
    }
  } else {
    emergency_stop_f = 0;
  }
}