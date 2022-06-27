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
#define __BUZZER_RCC RCC_AHB1Periph_GPIOD
#define __BUZZER_GPIO GPIOD
#define __BUZZER_PIN GPIO_Pin_13
//
#define __BUTTON1_ID 0x01
#define __BUTTON1_RCC RCC_AHB1Periph_GPIOD
#define __BUTTON1_GPIO GPIOD
#define __BUTTON1_PIN GPIO_Pin_14
////

void DrvMiscInit(void);

void DrvBuzzerInit(void);
void Buzzer_Set(uint8_t on);

void DrvButtonInit(void);
uint8_t Button_Get(u8 button);

#endif