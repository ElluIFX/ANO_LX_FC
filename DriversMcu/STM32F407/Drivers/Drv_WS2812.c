/**
 * @file Drv_WS2812.c
 * @brief Driver for WS2812 RGB LED
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-11
 *
 * THINK DIFFERENTLY
 */

#include "Drv_WS2812.h"
static uint8_t ws2812_buf[3 * WS2812_NUM + 1];

/**
 * @brief Initialize WS2812 GPIO and buffer
 */
void DrvWS2812Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(__WS2812_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = __WS2812_PIN;
  GPIO_Init(__WS2812_GPIO, &GPIO_InitStructure);

  GPIO_ResetBits(__WS2812_GPIO, __WS2812_PIN);

  for (u8 i = 0; i < WS2812_NUM; i++) {
    ws2812_buf[3 * i + 0] = 0;
    ws2812_buf[3 * i + 1] = 0;
    ws2812_buf[3 * i + 2] = 0;
  }
  WS2812_SendBit(ws2812_buf, WS2812_NUM);
}

// #pragma GCC push_options
// #pragma GCC optimize("O0")
/**
 * @brief Send data bits to WS2812
 * @param  data             uint32_t LED array, each in G R B order
 * @param  len              the number of LEDs
 */
void WS2812_SendBit(uint8_t* data, uint8_t len) {
  static uint8_t _i = 0;
  static uint8_t _j = 0;
  for (_i = 0; _i < len * 3; _i++) {
    for (_j = 0; _j < 8; _j++) {
      if (*data & (128 >> _j)) {
        __WS2812_HIGH_BIT;
      } else {
        __WS2812_LOW_BIT;
      }
    }
    data++;
  }
}
// #pragma GCC pop_options

/**
 * @brief Set specific LED to ARGB color
 * @param  argb_data        uint32_t ARGB color (Alpha, Red, Green, Blue)
 * @param  index            the index of LED (0 - WS2812_NUM)
 */
void WS2812_SetARGB(uint32_t argb_data, uint8_t index) {
  if (index >= WS2812_NUM) {
    return;
  }
  uint16_t brightness = (uint16_t)(argb_data >> 24 & 0xFF);
  uint8_t r = (argb_data >> 8) & 0xFF;
  uint8_t g = (argb_data >> 16) & 0xFF;
  uint8_t b = argb_data & 0xFF;
  ws2812_buf[3 * index] = (r * brightness) / 255;
  ws2812_buf[3 * index + 1] = (g * brightness) / 255;
  ws2812_buf[3 * index + 2] = (b * brightness) / 255;
}

/**
 * @brief Set specific LED to RGB color, alpha is set to 255
 * @param  argb_data        uint32_t RGB color (Red, Green, Blue)
 * @param  index            the index of LED (0 - WS2812_NUM)
 */
void WS2812_SetRGB(uint32_t rgb_data, uint8_t index) {
  WS2812_SetARGB(rgb_data | 0xFF000000, index);
}

/**
 * @brief Send data bits to WS2812 immediately.
 * This function is not thread safe. Will disable interrupts
 * during transmission.
 */
void WS2812_SendBuf(void) {
  // __disable_irq();  // disable all interrupts
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
  WS2812_SendBit(ws2812_buf, WS2812_NUM);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  // __enable_irq();  // enable all interrupts
}

/**
 * @brief Set all LEDs to one ARGB color
 * @param  argb_data        uint32_t ARGB color (Alpha, Red, Green, Blue)
 */
void WS2812_SetAll(uint32_t argb_data) {
  for (uint8_t i = 0; i < WS2812_NUM; i++) {
    WS2812_SetARGB(argb_data, i);
  }
}

/**
 * @brief Right shift all LEDs by one position
 */
void WS2812_BufRightShift(void) {
  u8 r_bit0 = ws2812_buf[(WS2812_NUM - 1) * 3 + 0];
  u8 r_bit1 = ws2812_buf[(WS2812_NUM - 1) * 3 + 1];
  u8 r_bit2 = ws2812_buf[(WS2812_NUM - 1) * 3 + 2];
  for (uint8_t i = WS2812_NUM - 1; i > 0; i--) {
    ws2812_buf[i * 3 + 0] = ws2812_buf[(i - 1) * 3 + 0];
    ws2812_buf[i * 3 + 1] = ws2812_buf[(i - 1) * 3 + 1];
    ws2812_buf[i * 3 + 2] = ws2812_buf[(i - 1) * 3 + 2];
  }
  ws2812_buf[0] = r_bit0;
  ws2812_buf[1] = r_bit1;
  ws2812_buf[2] = r_bit2;
}

/**
 * @brief Left shift all LEDs by one position
 */
void WS2812_BufLeftShift(void) {
  u8 l_bit0 = ws2812_buf[0];
  u8 l_bit1 = ws2812_buf[1];
  u8 l_bit2 = ws2812_buf[2];
  for (uint8_t i = 0; i < WS2812_NUM - 1; i++) {
    ws2812_buf[i * 3 + 0] = ws2812_buf[(i + 1) * 3 + 0];
    ws2812_buf[i * 3 + 1] = ws2812_buf[(i + 1) * 3 + 1];
    ws2812_buf[i * 3 + 2] = ws2812_buf[(i + 1) * 3 + 2];
  }
  ws2812_buf[(WS2812_NUM - 1) * 3 + 0] = l_bit0;
  ws2812_buf[(WS2812_NUM - 1) * 3 + 1] = l_bit1;
  ws2812_buf[(WS2812_NUM - 1) * 3 + 2] = l_bit2;
}

/**
 * @brief Flip the buffer
 */
void WS2812_BufFlip(void) {
  uint8_t r_bit0;
  uint8_t r_bit1;
  uint8_t r_bit2;
  for (uint8_t i = 0; i < WS2812_NUM / 2; i++) {
    r_bit0 = ws2812_buf[i * 3 + 0];
    r_bit1 = ws2812_buf[i * 3 + 1];
    r_bit2 = ws2812_buf[i * 3 + 2];
    ws2812_buf[i * 3 + 0] = ws2812_buf[(WS2812_NUM - i - 1) * 3 + 0];
    ws2812_buf[i * 3 + 1] = ws2812_buf[(WS2812_NUM - i - 1) * 3 + 1];
    ws2812_buf[i * 3 + 2] = ws2812_buf[(WS2812_NUM - i - 1) * 3 + 2];
    ws2812_buf[(WS2812_NUM - i - 1) * 3 + 0] = r_bit0;
    ws2812_buf[(WS2812_NUM - i - 1) * 3 + 1] = r_bit1;
    ws2812_buf[(WS2812_NUM - i - 1) * 3 + 2] = r_bit2;
  }
}
