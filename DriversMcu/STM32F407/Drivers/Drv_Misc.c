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
 * @brief Set Buzzer on or off
 * @param  on               1: on, 0: off
 */
void Buzzer_Set(uint8_t on) {
  if (on != 0) {
    GPIO_SetBits(__BUZZER_GPIO, __BUZZER_PIN);
  } else {
    GPIO_ResetBits(__BUZZER_GPIO, __BUZZER_PIN);
  }
}

/**
 * @brief Get button status
 * @param  button           button id
 * @return 1: pressed, 0: released
 */
uint8_t Button_Get(u8 button) {
  if (button == __BUTTON1_ID) {
    return !GPIO_ReadInputDataBit(__BUTTON1_GPIO, __BUTTON1_PIN);
  }
  return 0;
}

/**
 * @brief Set output pin on or off
 * @param  id               pin id
 * @param  on               1: on, 0: off
 */
void DOut_Set(u8 id, u8 on) {
  if (id == 0x00) {
    if (on != 0) {
      GPIO_SetBits(__DOUT_GPIO, __DOUT_0_PIN);
    } else {
      GPIO_ResetBits(__DOUT_GPIO, __DOUT_0_PIN);
    }
  } else if (id == 0x01) {
    if (on != 0) {
      GPIO_SetBits(__DOUT_GPIO, __DOUT_1_PIN);
    } else {
      GPIO_ResetBits(__DOUT_GPIO, __DOUT_1_PIN);
    }
  }
}

void DrvBuzzerInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(__BUZZER_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = __BUZZER_PIN;
  GPIO_Init(__BUZZER_GPIO, &GPIO_InitStructure);

  GPIO_ResetBits(__BUZZER_GPIO, __BUZZER_PIN);
}

void DrvButtonInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(__BUTTON1_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = __BUTTON1_PIN;
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
  DrvBuzzerInit();
  DrvButtonInit();
  DrvDOutInit();
}