#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"

/* STM32H7 TIM15 is clocked such that Period=299 gets 800KHz */
#define CODE_ONE_DUTY 216
#define CODE_ZERO_DUTY 72

#define RST_PERIOD_NUM 100
#define WS2812_NUM 1

extern uint32_t ws2812_color[WS2812_NUM];

/* 非 0 = LVGL 手动控制模式（禁止 main 循环自动彩虹），0 = 自动彩虹模式 */
extern uint8_t g_ws2812_manual_mode;

// 将颜色数组直接更新到 LED，不使用渐变过渡
void ws2812_update(void);

// 渐变的更新LED颜色
void ws2812_gradient(uint8_t steps, uint16_t delay_ms);

// 设置LED颜色（24bit颜色）
void ws2812_set(uint8_t led_id, uint32_t color);

// 设置LED颜色（RGB）
void ws2812_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b);

// 设置所有LED颜色
void ws2812_set_all(uint32_t color);

// RGB转换为24bit颜色
uint32_t rgb_to_color(uint8_t r, uint8_t g, uint8_t b);

// 24bit颜色转换为RGB
void color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b);

// =============== 以下为额外的效果函数 ===============

// 彩虹颜色生成
uint32_t rainbow_color(float frequency, int phase, int center, int width);

// 彩虹效果（原始实现，向后兼容）
void rainbow_effect(uint8_t steps, uint16_t delay_ms);

// 彩虹效果（带亮度控制）
// brightness: 0.0f (全灭) .. 1.0f (原始峰值亮度)
void rainbow_effect_with_brightness(uint8_t steps, uint16_t delay_ms, float brightness);

// 等待上一帧 DMA 传输完成，timeout_ms 单位毫秒
void ws2812_wait_dma(uint32_t timeout_ms);

// 设置所有 LED 为白光，brightness: 0.0f (关) .. 1.0f (最大亮度)
void ws2812_set_all_white(float brightness);

// 初始化 WS2812 驱动并将所有 LED 设为全灭
void ws2812_Init(void);
void ws2812_running_rainbow_cycle(int step, float brightness);

#endif
