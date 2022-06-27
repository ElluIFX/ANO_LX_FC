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

void DrvMiscInit(void) {
  DrvBuzzerInit();
  DrvButtonInit();
}

void DrvBuzzerInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(__BUZZER_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = __BUZZER_PIN;
  GPIO_Init(__BUZZER_GPIO, &GPIO_InitStructure);

  GPIO_ResetBits(__BUZZER_GPIO, __BUZZER_PIN);
}

/**
 * @brief Set Buzzer on or off
 * @param  on               1: on, 0: off
 */
void Buzzer_Set(uint8_t on) {
  if (on) {
    GPIO_SetBits(__BUZZER_GPIO, __BUZZER_PIN);
  } else {
    GPIO_ResetBits(__BUZZER_GPIO, __BUZZER_PIN);
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
  GPIO_InitStructure.GPIO_Pin = __BUTTON1_PIN;
  GPIO_Init(__BUTTON1_GPIO, &GPIO_InitStructure);
}

uint8_t Button_Get(u8 button) {
  if (button == __BUTTON1_ID) {
    return !GPIO_ReadInputDataBit(__BUTTON1_GPIO, __BUTTON1_PIN);
  }
  return 0;
}
