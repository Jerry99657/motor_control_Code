#include "lv_port_indev.h"

#include "main.h"
#include "ui_settings.h"

static lv_indev_t *s_keypad = NULL;

#define LV_PORT_LR_REPEAT_START_MS 500U
#define LV_PORT_LR_REPEAT_STEP_MS  250U

static uint32_t s_lr_active_key = 0U;
static uint32_t s_lr_press_tick = 0U;
static uint32_t s_lr_last_repeat_tick = 0U;
static volatile uint8_t s_suppress_exit_keys = 0U;
static volatile uint8_t s_suppress_all_keys = 0U;
static uint16_t s_last_pressed_mask = 0U;

#define LV_PORT_KEY_MASK_UP       (1U << 0)
#define LV_PORT_KEY_MASK_RIGHT    (1U << 1)
#define LV_PORT_KEY_MASK_DOWN     (1U << 2)
#define LV_PORT_KEY_MASK_LEFT     (1U << 3)
#define LV_PORT_KEY_MASK_OK       (1U << 4)
#define LV_PORT_KEY_MASK_KEY1     (1U << 5)
#define LV_PORT_KEY_MASK_KEY2     (1U << 6)
#define LV_PORT_KEY_MASK_KEY3     (1U << 7)

static uint8_t lv_port_key_is_pressed(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint32_t lv_port_key_get_code(void)
{
    uint16_t pressed_mask = 0U;
    uint16_t new_press_mask;
    UI_SettingsDirection physical_direction;
    UI_SettingsDirection logical_direction;
    uint8_t direction_pressed = 0U;
    uint32_t lr_key;
    uint32_t now;

    if (lv_port_key_is_pressed(Key_Up_GPIO_Port, Key_Up_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_UP;
    }
    if (lv_port_key_is_pressed(Key_Right_GPIO_Port, Key_Right_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_RIGHT;
    }
    if (lv_port_key_is_pressed(Key_Down_GPIO_Port, Key_Down_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_DOWN;
    }
    if (lv_port_key_is_pressed(Key_Left_GPIO_Port, Key_Left_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_LEFT;
    }
    if (lv_port_key_is_pressed(Key_OK_GPIO_Port, Key_OK_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_OK;
    }
    if (lv_port_key_is_pressed(Key1_GPIO_Port, Key1_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_KEY1;
    }
    if (lv_port_key_is_pressed(Key2_GPIO_Port, Key2_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_KEY2;
    }
    if (lv_port_key_is_pressed(Key3_GPIO_Port, Key3_Pin) != 0U)
    {
        pressed_mask |= LV_PORT_KEY_MASK_KEY3;
    }

    new_press_mask = (uint16_t)(pressed_mask & (uint16_t)~s_last_pressed_mask);
    s_last_pressed_mask = pressed_mask;
    if (new_press_mask != 0U)
    {
        UI_Settings_NotifyKeyPress();
    }

    if (s_suppress_all_keys != 0U)
    {
        if (pressed_mask != 0U)
        {
            return 0U;
        }
        s_suppress_all_keys = 0U;
    }

    if (s_suppress_exit_keys != 0U)
    {
        if ((pressed_mask & (LV_PORT_KEY_MASK_KEY2 |
                             LV_PORT_KEY_MASK_KEY3)) != 0U)
        {
            return 0U;
        }

        s_suppress_exit_keys = 0U;
    }

    if ((pressed_mask & LV_PORT_KEY_MASK_KEY2) != 0U)
    {
        return LV_KEY_ESC;
    }

    /* KEY3 is the hierarchical back key for normal LVGL pages.  Blocking
     * media/NES paths also poll it directly and suppress this event until the
     * physical key is released after restoring their parent page. */
    if ((pressed_mask & LV_PORT_KEY_MASK_KEY3) != 0U)
    {
        return LV_KEY_ESC;
    }

    if ((pressed_mask & LV_PORT_KEY_MASK_OK) != 0U)
    {
        return LV_KEY_ENTER;
    }

    if ((pressed_mask & LV_PORT_KEY_MASK_UP) != 0U)
    {
        physical_direction = UI_SETTINGS_DIRECTION_UP;
        direction_pressed = 1U;
    }
    else if ((pressed_mask & LV_PORT_KEY_MASK_RIGHT) != 0U)
    {
        physical_direction = UI_SETTINGS_DIRECTION_RIGHT;
        direction_pressed = 1U;
    }
    else if ((pressed_mask & LV_PORT_KEY_MASK_DOWN) != 0U)
    {
        physical_direction = UI_SETTINGS_DIRECTION_DOWN;
        direction_pressed = 1U;
    }
    else if ((pressed_mask & LV_PORT_KEY_MASK_LEFT) != 0U)
    {
        physical_direction = UI_SETTINGS_DIRECTION_LEFT;
        direction_pressed = 1U;
    }

    if (direction_pressed == 0U)
    {
        s_lr_active_key = 0U;
        return 0U;
    }

    logical_direction = UI_Settings_MapDirection(physical_direction);
    if (logical_direction == UI_SETTINGS_DIRECTION_UP)
    {
        s_lr_active_key = 0U;
        return LV_KEY_PREV;
    }
    if (logical_direction == UI_SETTINGS_DIRECTION_DOWN)
    {
        s_lr_active_key = 0U;
        return LV_KEY_NEXT;
    }
    if (logical_direction == UI_SETTINGS_DIRECTION_LEFT)
    {
        lr_key = LV_KEY_LEFT;
    }
    else
    {
        lr_key = LV_KEY_RIGHT;
    }

    now = HAL_GetTick();

    if (s_lr_active_key != lr_key)
    {
        s_lr_active_key = lr_key;
        s_lr_press_tick = now;
        s_lr_last_repeat_tick = now;
        return lr_key;
    }

    if ((now - s_lr_press_tick) < LV_PORT_LR_REPEAT_START_MS)
    {
        return 0U;
    }

    if ((now - s_lr_last_repeat_tick) >= LV_PORT_LR_REPEAT_STEP_MS)
    {
        s_lr_last_repeat_tick = now;
        return lr_key;
    }

    return 0U;
}

static void lv_port_keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static uint32_t last_key = LV_KEY_ENTER;
    uint32_t key;

    (void)indev_drv;

    key = lv_port_key_get_code();
    if (key != 0U)
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = key;
        last_key = key;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
        data->key = last_key;
    }
}

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = lv_port_keypad_read;
    s_keypad = lv_indev_drv_register(&indev_drv);
}

lv_indev_t *lv_port_indev_get_keypad(void)
{
    return s_keypad;
}

void lv_port_indev_suppress_exit_keys_until_release(void)
{
    s_suppress_exit_keys = 1U;
}

void lv_port_indev_suppress_all_keys_until_release(void)
{
    s_lr_active_key = 0U;
    s_suppress_all_keys = 1U;
    if (s_keypad != NULL)
    {
        lv_indev_wait_release(s_keypad);
    }
}
