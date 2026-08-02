#include "media_control.h"

#include "lcd_spi_154.h"
#include "main.h"

#define MEDIA_CONTROL_KEY_GUARD_MS     120U
#define MEDIA_CONTROL_REPEAT_START_MS  500U
#define MEDIA_CONTROL_REPEAT_STEP_MS   250U

static uint8_t s_paused = 0U;
static uint8_t s_ok_latched = 0U;
static uint32_t s_ok_action_tick = 0U;
static media_control_action_t s_seek_active = MEDIA_CONTROL_NONE;
static uint32_t s_seek_action_tick = 0U;
static uint32_t s_seek_press_tick = 0U;
static uint32_t s_seek_repeat_tick = 0U;

static uint8_t media_key_pressed(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

void MediaControl_Init(void)
{
    uint8_t left_pressed;
    uint8_t right_pressed;
    uint32_t now = HAL_GetTick();

    s_paused = 0U;
    s_ok_latched = media_key_pressed(Key_OK_GPIO_Port, Key_OK_Pin);
    s_ok_action_tick = now;
    left_pressed = media_key_pressed(Key_Left_GPIO_Port, Key_Left_Pin);
    right_pressed = media_key_pressed(Key_Right_GPIO_Port, Key_Right_Pin);

    if ((left_pressed != 0U) && (right_pressed == 0U))
    {
        s_seek_active = MEDIA_CONTROL_SEEK_BACK;
    }
    else if ((right_pressed != 0U) && (left_pressed == 0U))
    {
        s_seek_active = MEDIA_CONTROL_SEEK_FORWARD;
    }
    else
    {
        s_seek_active = MEDIA_CONTROL_NONE;
    }

    s_seek_action_tick = now;
    s_seek_press_tick = now;
    s_seek_repeat_tick = now;
}

media_control_action_t MediaControl_Poll(void)
{
    uint8_t ok_pressed;
    uint8_t left_pressed;
    uint8_t right_pressed;
    media_control_action_t seek_action;
    uint32_t now;

    if (media_key_pressed(Key2_GPIO_Port, Key2_Pin) != 0U)
    {
        return MEDIA_CONTROL_STOP;
    }

    if (media_key_pressed(Key3_GPIO_Port, Key3_Pin) != 0U)
    {
        return MEDIA_CONTROL_BACK;
    }

    ok_pressed = media_key_pressed(Key_OK_GPIO_Port, Key_OK_Pin);
    if (ok_pressed == 0U)
    {
        s_ok_latched = 0U;
    }
    else if ((s_ok_latched == 0U) &&
             ((HAL_GetTick() - s_ok_action_tick) >= MEDIA_CONTROL_KEY_GUARD_MS))
    {
        s_ok_latched = 1U;
        s_ok_action_tick = HAL_GetTick();
        s_paused = (s_paused == 0U) ? 1U : 0U;
        return MEDIA_CONTROL_PAUSE_CHANGED;
    }

    left_pressed = media_key_pressed(Key_Left_GPIO_Port, Key_Left_Pin);
    right_pressed = media_key_pressed(Key_Right_GPIO_Port, Key_Right_Pin);
    if ((left_pressed == 0U) && (right_pressed == 0U))
    {
        s_seek_active = MEDIA_CONTROL_NONE;
        return MEDIA_CONTROL_NONE;
    }

    if ((left_pressed != 0U) && (right_pressed == 0U))
    {
        seek_action = MEDIA_CONTROL_SEEK_BACK;
    }
    else if ((right_pressed != 0U) && (left_pressed == 0U))
    {
        seek_action = MEDIA_CONTROL_SEEK_FORWARD;
    }
    else
    {
        s_seek_active = MEDIA_CONTROL_NONE;
        return MEDIA_CONTROL_NONE;
    }

    now = HAL_GetTick();
    if (s_seek_active != seek_action)
    {
        s_seek_active = seek_action;
        s_seek_press_tick = now;
        s_seek_repeat_tick = now;
        if ((now - s_seek_action_tick) < MEDIA_CONTROL_KEY_GUARD_MS)
        {
            return MEDIA_CONTROL_NONE;
        }
        s_seek_action_tick = now;
        return seek_action;
    }

    if (((now - s_seek_press_tick) >= MEDIA_CONTROL_REPEAT_START_MS) &&
        ((now - s_seek_repeat_tick) >= MEDIA_CONTROL_REPEAT_STEP_MS))
    {
        s_seek_repeat_tick = now;
        s_seek_action_tick = now;
        return seek_action;
    }

    return MEDIA_CONTROL_NONE;
}

uint8_t MediaControl_IsPaused(void)
{
    return s_paused;
}

uint8_t MediaControl_IsSeekHeld(void)
{
    uint8_t left_pressed = media_key_pressed(Key_Left_GPIO_Port, Key_Left_Pin);
    uint8_t right_pressed = media_key_pressed(Key_Right_GPIO_Port, Key_Right_Pin);

    return (((left_pressed != 0U) && (right_pressed == 0U)) ||
            ((right_pressed != 0U) && (left_pressed == 0U))) ? 1U : 0U;
}

void MediaControl_ShowPausedHud(void)
{
    LCD_SetBackColor(DARK_BLUE);
    LCD_SetColor(LCD_WHITE);
    LCD_ClearRect(34U, 94U, 172U, 52U);
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_DisplayString(92U, 101U, "PAUSED");
    LCD_SetAsciiFont(&ASCII_Font12);
    LCD_DisplayString(70U, 126U, "OK PLAY   <</>>");
    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);
}
