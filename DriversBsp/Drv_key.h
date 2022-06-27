#ifndef __key_H
#define __key_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SysConfig.h"

#define shortmaxclickms (300)
#define shortminclickms (10)
#define longclickms (800)

#define KEY_EVENT_NULL 0x0000
#define KEY_EVENT_DOWN 0x0001
#define KEY_EVENT_UP_SHORT 0x0002   // 短按后松开事件
#define KEY_EVENT_UP_LONG 0x0003    // 长按后松开事件
#define KEY_EVENT_UP_DOUBLE 0x0004  // 双击后松开事件
#define KEY_EVENT_SHORT 0x0005
#define KEY_EVENT_LONG 0x0006
#define KEY_EVENT_DOUBLE 0x0007

#define KEY_READ_DOWN 0x00 /* key is pressed          */
#define KEY_READ_UP 0x01   /* Key isn't pressed       */

/******************************************************************************
                           User Interface [START]
*******************************************************************************/

#define KEY1_NUM 0x0001

#define KEY1_DOWN (KEY_EVENT_DOWN | KEY1_NUM << 8)
#define KEY1_UP_SHORT (KEY_EVENT_UP_SHORT | KEY1_NUM << 8)
#define KEY1_UP_LONG (KEY_EVENT_UP_LONG | KEY1_NUM << 8)
#define KEY1_UP_DOUBLE (KEY_EVENT_UP_DOUBLE | KEY1_NUM << 8)
#define KEY1_SHORT (KEY_EVENT_SHORT | KEY1_NUM << 8)
#define KEY1_LONG (KEY_EVENT_LONG | KEY1_NUM << 8)
#define KEY1_DOUBLE (KEY_EVENT_DOUBLE | KEY1_NUM << 8)

// #define KEY2_NUM 0x0002
//
// #define KEY2_DOWN (KEY_EVENT_DOWN | KEY2_NUM << 8)
// #define KEY2_UP_SHORT (KEY_EVENT_UP_SHORT | KEY2_NUM << 8)
// #define KEY2_UP_LONG (KEY_EVENT_UP_LONG | KEY2_NUM << 8)
// #define KEY2_UP_DOUBLE (KEY_EVENT_UP_DOUBLE | KEY2_NUM << 8)
// #define KEY2_SHORT (KEY_EVENT_SHORT | KEY2_NUM << 8)
// #define KEY2_LONG (KEY_EVENT_LONG | KEY2_NUM << 8)
// #define KEY2_DOUBLE (KEY_EVENT_DOUBLE | KEY2_NUM << 8)

/******************************************************************************
                           User Interface [END]
*******************************************************************************/

void key_check_all_loop_1ms(void);
unsigned short key_read_value(void);

#ifdef __cplusplus
}
#endif

#endif
