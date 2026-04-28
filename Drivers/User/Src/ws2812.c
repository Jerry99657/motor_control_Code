#include "ws2812.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

extern TIM_HandleTypeDef htim15;

// LED颜色
uint32_t ws2812_color[WS2812_NUM] = {0};

// 当前LED颜色
static uint32_t _ws2812_color_current[WS2812_NUM];

// DMA/传输 busy 标志（用于等�?DMA 完成�?
static volatile uint8_t _ws2812_dma_busy = 0;

/**
 * @brief  直接更新LED颜色
 */
void ws2812_update(void)
{
	// 数据缓冲，每个LED占用24个字节，�?0个LED，前100个字节用于复位信�?
	static uint16_t ws2812_data[RST_PERIOD_NUM + WS2812_NUM * 24];

	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
	{
		_ws2812_color_current[led_id] = ws2812_color[led_id];
		static uint8_t r, g, b;
		color_to_rgb(_ws2812_color_current[led_id], &r, &g, &b);
		uint16_t *p = ws2812_data + RST_PERIOD_NUM + led_id * 24;
		for (uint8_t i = 0; i < 8; i++)
		{
			p[i] = (r << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
			p[i + 8] = (g << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
			p[i + 16] = (b << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
		}
	}
	// 标记为忙，DMA 传输将在完成回调中清除该标志
	_ws2812_dma_busy = 1;
	HAL_TIM_PWM_Start_DMA(&htim15, TIM_CHANNEL_2, (uint32_t *)ws2812_data,
						  RST_PERIOD_NUM + WS2812_NUM * 24);
}

/**
 * @brief 等待当前 DMA 传输完成（或超时�?
 * @param timeout_ms 超时时间（毫秒）�? 表示不等待直接返�?
 */
void ws2812_wait_dma(uint32_t timeout_ms)
{
	if (timeout_ms == 0) return;
	uint32_t start = HAL_GetTick();
	while (_ws2812_dma_busy)
	{
		if ((HAL_GetTick() - start) >= timeout_ms) break;
		/* 轻等待，避免忙等过于激�?*/
		HAL_Delay(1);
	}
}

/**
 * @brief HAL 回调：PWM 脉冲（DMA）完�?
 * 这里覆盖 weak 回调，检测是否为 TIM3，清除忙标志并停�?DMA 通道
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim15)
	{
		_ws2812_dma_busy = 0;
		/* 停止 DMA，以便下一次安全启动（HAL 会在回调时已经处理好状态，但显式停止更稳妥�?*/
		HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
	}
}

/**
 * @brief  通过渐变方式更新LED颜色（线性插值）
 * @param  steps: 渐变步数
 * @param  delay_ms: 每步之间的延迟时间（毫秒�?
 */
void ws2812_gradient(uint8_t steps, uint16_t delay_ms)
{
	static uint8_t start_r[WS2812_NUM], start_g[WS2812_NUM], start_b[WS2812_NUM];
	static float r_step[WS2812_NUM], g_step[WS2812_NUM], b_step[WS2812_NUM];

	// 提取初始颜色，并计算每步的渐变步�?
	for (uint8_t i = 0; i < WS2812_NUM; i++)
	{
		color_to_rgb(_ws2812_color_current[i], &start_r[i], &start_g[i], &start_b[i]);
		uint8_t target_r, target_g, target_b;
		color_to_rgb(ws2812_color[i], &target_r, &target_g, &target_b);

		r_step[i] = (float)(target_r - start_r[i]) / steps;
		g_step[i] = (float)(target_g - start_g[i]) / steps;
		b_step[i] = (float)(target_b - start_b[i]) / steps;
	}

	// 逐步渐变
	for (uint8_t step = 1; step <= steps; step++)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			// 计算当前步的颜色
			uint8_t r = (uint8_t)(start_r[led_id] + r_step[led_id] * step);
			uint8_t g = (uint8_t)(start_g[led_id] + g_step[led_id] * step);
			uint8_t b = (uint8_t)(start_b[led_id] + b_step[led_id] * step);

			ws2812_set_rgb(led_id, r, g, b);
		}

		ws2812_update();
		// (原始实现此处可能等待 DMA 完成)
		ws2812_wait_dma(100);
		HAL_Delay(delay_ms);
	}
}

/**
 * @brief  设置LED颜色(RGB格式)
 * @param  led_id: LED编号（学习板一共有10个LED，编号范�?-9�?
 * @param  r: 红色亮度�?-255�?
 * @param  g: 绿色亮度�?-255�?
 * @param  b: 蓝色亮度�?-255�?
 */
void ws2812_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b)
{
	ws2812_color[led_id] = rgb_to_color(r, g, b);
}

/**
 * @brief  设置LED颜色�?4bit颜色格式�?
 * @param  led_id: LED编号（学习板一共有10个LED，编号范�?-9�?
 * @param  color: 24bit颜色
 */
void ws2812_set(uint8_t led_id, uint32_t color)
{
	ws2812_color[led_id] = color;
}

/**
 * @brief  设置所有LED颜色�?4bit颜色格式�?
 * @param  color: 24bit颜色
 */
void ws2812_set_all(uint32_t color)
{
	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
	{
		ws2812_color[led_id] = color;
	}
}

/**
 * @brief 将所�?LED 设置为白光，�?brightness 缩放亮度
 * @param brightness 0.0f (全灭) .. 1.0f (原始峰值亮�?
 */
void ws2812_set_all_white(float brightness)
{
	if (brightness < 0.0f) brightness = 0.0f;
	if (brightness > 1.0f) brightness = 1.0f;

	uint8_t level = (uint8_t)(255.0f * brightness);

	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
	{
		ws2812_set_rgb(led_id, level, level, level);
	}

	// 发送并等待 DMA 完成（避免下一次更新覆盖缓冲）
	ws2812_update();
	ws2812_wait_dma(100);
}

/**
 * @brief 初始�?WS2812 驱动并将所�?LED 设为全灭
 *        调用后所有灯会被写为 0 并发送一次数据（阻塞等待 DMA 完成�?
 */
void ws2812_Init(void)
{
  /* Stop any ongoing transfer */
  HAL_TIM_PWM_Stop_DMA(&htim15, TIM_CHANNEL_2);
  _ws2812_dma_busy = 0;
  
  /* Set all to 0 (off) */
  ws2812_set_all(0);
  ws2812_update();
  ws2812_wait_dma(100);
}

void ws2812_running_rainbow_cycle(int step, float brightness)
{
  /* Clear background */
  ws2812_set_all(0);

  /* Draw 4 LEDs: Head (brightest), Body1, Body2, Tail (dimmest) */
  for (int i = 0; i < 4; i++)
  {
    /* Calculate index wrapping around WS2812_NUM */
    int led_index = (step - i);
    while (led_index < 0) led_index += WS2812_NUM;
    led_index %= WS2812_NUM;

    /* Brightness fade: Head=1.0, Body1=0.75, Body2=0.5, Tail=0.25 */
    float fade = 1.0f - (i * 0.25f);

    /* Color: Rainbow based on step (moving color) */
    /* Use step to shift hue so it looks like a rainbow cycle */
    uint32_t color = rainbow_color(0.1f, step * 2 + i * 5, 128, 127);
    
    uint8_t r, g, b;
    color_to_rgb(color, &r, &g, &b);

    /* Apply global brightness and fade */
    r = (uint8_t)((float)r * brightness * fade);
    g = (uint8_t)((float)g * brightness * fade);
    b = (uint8_t)((float)b * brightness * fade);

    ws2812_set_rgb(led_index, r, g, b);
  }
  ws2812_update();
  ws2812_wait_dma(50);
}

/**
 * @brief  RGB转换�?4bit颜色
 * @param  r: 红色亮度�?-255�?
 * @param  g: 绿色亮度�?-255�?
 * @param  b: 蓝色亮度�?-255�?
 * @retval 24bit颜色
 */
uint32_t rgb_to_color(uint8_t r, uint8_t g, uint8_t b)
{
	return (r << 16) | (g << 8) | b;
}

/**
 * @brief  24bit颜色转换为RGB
 * @param  color: 24bit颜色
 * @param  r: 红色亮度�?-255�?
 * @param  g: 绿色亮度�?-255�?
 * @param  b: 蓝色亮度�?-255�?
 */
void color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
	*r = (color >> 16) & 0xFF;
	*g = (color >> 8) & 0xFF;
	*b = color & 0xFF;
}

// =============== 以下为额外的效果演示函数 ================

uint32_t rainbow_color(float frequency, int phase, int center, int width)
{
	float r = sinf(frequency * phase + 0) * width + center;
	float g = sinf(frequency * phase + 2) * width + center;
	float b = sinf(frequency * phase + 4) * width + center;
	return rgb_to_color((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

void rainbow_effect(uint8_t steps, uint16_t delay_ms)
{
	/* 原始实现已被注释，保留以供参考：
	float frequency = 0.1;
	int center = 128;
	int width = 127;

	if (steps == 0) return; // 防止除零或无效循�?

	for (int i = 0; i < steps; i++)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			uint32_t color = rainbow_color(frequency, i + led_id * 2, center, width);
			ws2812_set(led_id, color);
		}
		ws2812_update();
		// (原始实现在此可能等待 DMA 完成)
		HAL_Delay(delay_ms);
	}

	 // 倒序：i = steps-2 .. 0 （避免重复播放最后一步）
	for (int i = steps - 2; i >= 0; i--)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			uint32_t color = rainbow_color(frequency, i + led_id * 2, center, width);
			ws2812_set(led_id, color);
		}
		ws2812_update();
		// (原始实现在此可能等待 DMA 完成)
		HAL_Delay(delay_ms);
	}
	*/

	/* 向后兼容：调用带亮度控制的新实现，默认亮�?1.0f（原始峰值） */
	rainbow_effect_with_brightness(steps, delay_ms, 1.0f);
}


// 新实现：带亮度控制的彩虹效果
void rainbow_effect_with_brightness(uint8_t steps, uint16_t delay_ms, float brightness)
{
	float frequency = 0.1f;
	int center = 128;
	int width = 127;

	if (steps == 0) return; // 防止除零或无效循�?

	if (brightness < 0.0f) brightness = 0.0f;
	if (brightness > 1.0f) brightness = 1.0f;

	// 正序
	for (int i = 0; i < steps; i++)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			uint32_t color = rainbow_color(frequency, i + led_id * 2, center, width);
			uint8_t r, g, b;
			color_to_rgb(color, &r, &g, &b);

			// 线性缩放亮�?
			r = (uint8_t)((float)r * brightness);
			g = (uint8_t)((float)g * brightness);
			b = (uint8_t)((float)b * brightness);

			ws2812_set_rgb(led_id, r, g, b);
		}
		ws2812_update();
		/* 等待 DMA 完成，避免覆盖发送缓�?*/
		ws2812_wait_dma(100);
		HAL_Delay(delay_ms);
	}

	// 倒序（避免重复最后一帧）
	for (int i = steps - 2; i >= 0; i--)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			uint32_t color = rainbow_color(frequency, i + led_id * 2, center, width);
			uint8_t r, g, b;
			color_to_rgb(color, &r, &g, &b);

			r = (uint8_t)((float)r * brightness);
			g = (uint8_t)((float)g * brightness);
			b = (uint8_t)((float)b * brightness);

			ws2812_set_rgb(led_id, r, g, b);
		}
		ws2812_update();
		/* 等待 DMA 完成，避免覆盖发送缓�?*/
		ws2812_wait_dma(100);
		HAL_Delay(delay_ms);
	}

}
