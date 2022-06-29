/**
 * @file Drv_WS2812.h
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-11
 *
 * THINK DIFFERENTLY
 */

#ifndef _DRV_WS2812_H_
#define _DRV_WS2812_H_
#include "Drv_Sys.h"
#include "SysConfig.h"

// PIN defination
#define __WS2812_RCC RCC_AHB1Periph_GPIOC
#define __WS2812_GPIO GPIOC
#define __WS2812_PIN GPIO_Pin_6
////

#define WS2812_NUM 8

#define __WS2812_HIGH __WS2812_GPIO->BSRRL = __WS2812_PIN
#define __WS2812_LOW __WS2812_GPIO->BSRRH = __WS2812_PIN

////
#define __DELAY_700NS \
  for (uint8_t i = 0; i < 18; i++) __NOP()
#define __DELAY_600NS \
  for (uint8_t i = 0; i < 10; i++) __NOP()
//
#define __DELAY_350NS \
  for (uint8_t i = 0; i < 8; i++) __NOP()
#define __DELAY_800NS \
  for (uint8_t i = 0; i < 15; i++) __NOP()
//
#define __WS2812_HIGH_BIT \
  __WS2812_HIGH;          \
  __DELAY_700NS;          \
  __WS2812_LOW;           \
  __DELAY_600NS
#define __WS2812_LOW_BIT \
  __WS2812_HIGH;         \
  __DELAY_350NS;         \
  __WS2812_LOW;          \
  __DELAY_800NS
////

void DrvWS2812Init(void);
void WS2812_SendBit(uint8_t* data, uint8_t len);
void WS2812_SetARGB(uint32_t argb_data, uint8_t index);
void WS2812_SetRGB(uint32_t rgb_data, uint8_t index);
void WS2812_SetAll(uint32_t argb_data);
void WS2812_BufRightShift(void);
void WS2812_BufLeftShift(void);
void WS2812_BufFlip(void);
void WS2812_SendBuf(void);

#endif