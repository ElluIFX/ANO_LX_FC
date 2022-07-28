/**
 * @file Drv_Misc.c
 * @brief 外设驱动
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-27
 *
 * THINK DIFFERENTLY
 */

#include "Drv_Misc.h"

/**
 * @brief Get button status
 * @param  button           button id
 * @return 1: pressed, 0: released
 */
uint8_t Button_Get(u8 button) {
  if (button == __BUTTON1_ID) {
    return !GPIO_ReadInputDataBit(__BUTTON1_GPIO, __BUTTON1_PIN);
  } else if (button == __BUTTON2_ID) {
    return !GPIO_ReadInputDataBit(__BUTTON2_GPIO, __BUTTON2_PIN);
  }
  return 0;
}

/**
 * @brief Set output pin on or off
 * @param  id               pin id
 * @param  on               1: on, 0: off
 */
void DOut_Set(u8 id, u8 on) {
  const static uint16_t PINS_LIST[4] = {__DOUT_0_PIN, __DOUT_1_PIN,
                                        __DOUT_2_PIN, __DOUT_3_PIN};
  if (id < 4) {
    if (on != 0) {
      GPIO_SetBits(__DOUT_GPIO, PINS_LIST[id]);
    } else {
      GPIO_ResetBits(__DOUT_GPIO, PINS_LIST[id]);
    }
  }
}

void DrvButtonInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(__BUTTON1_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = __BUTTON1_PIN | __BUTTON2_PIN;
  GPIO_Init(__BUTTON1_GPIO, &GPIO_InitStructure);
}

void DrvDOutInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(__DOUT_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = __DOUT_ALL_PIN;
  GPIO_Init(__DOUT_GPIO, &GPIO_InitStructure);

  GPIO_ResetBits(__DOUT_GPIO, __DOUT_ALL_PIN);
}

void DrvMiscInit(void) {
  DrvButtonInit();
  DrvDOutInit();
}