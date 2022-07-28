/**
 * @file Drv_Misc.h
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-27
 *
 * THINK DIFFERENTLY
 */

#ifndef _DRV_MISC_H_
#define _DRV_MISC_H_

#include "Drv_Sys.h"
#include "SysConfig.h"

// PIN defination
#define __BUTTON1_ID 0x01
#define __BUTTON1_RCC RCC_AHB1Periph_GPIOB
#define __BUTTON1_GPIO GPIOB
#define __BUTTON1_PIN GPIO_Pin_0

#define __BUTTON2_ID 0x02
#define __BUTTON2_RCC RCC_AHB1Periph_GPIOB
#define __BUTTON2_GPIO GPIOB
#define __BUTTON2_PIN GPIO_Pin_1
//
#define __DOUT_RCC RCC_AHB1Periph_GPIOD
#define __DOUT_GPIO GPIOD
#define __DOUT_0_PIN GPIO_Pin_12
#define __DOUT_1_PIN GPIO_Pin_13
#define __DOUT_2_PIN GPIO_Pin_14
#define __DOUT_3_PIN GPIO_Pin_15
#define __DOUT_ALL_PIN \
  (__DOUT_0_PIN | __DOUT_1_PIN | __DOUT_2_PIN | __DOUT_3_PIN)
////

void DrvMiscInit(void);

uint8_t Button_Get(u8 button);

void DOut_Set(u8 id, u8 on);

#endif