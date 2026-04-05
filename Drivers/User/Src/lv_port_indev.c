#include "lv_port_indev.h"

#include "main.h"

static lv_indev_t *s_keypad = NULL;

#define LV_PORT_LR_REPEAT_START_MS 500U
#define LV_PORT_LR_REPEAT_STEP_MS  300U

static uint32_t s_lr_active_key = 0U;
static uint32_t s_lr_press_tick = 0U;
static uint32_t s_lr_last_repeat_tick = 0U;

static uint8_t lv_port_key_is_pressed(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint32_t lv_port_key_get_code(void)
{
    uint8_t left_pressed;
    uint8_t right_pressed;
    uint32_t lr_key;
    uint32_t now;

    if (lv_port_key_is_pressed(Key3_GPIO_Port, Key3_Pin) != 0U)
    {
        return LV_KEY_ESC;
    }

    if (lv_port_key_is_pressed(Key_Up_GPIO_Port, Key_Up_Pin) != 0U)
    {
        return LV_KEY_PREV;
    }

    if (lv_port_key_is_pressed(Key_Down_GPIO_Port, Key_Down_Pin) != 0U)
    {
        return LV_KEY_NEXT;
    }

    if (lv_port_key_is_pressed(Key_OK_GPIO_Port, Key_OK_Pin) != 0U)
    {
        return LV_KEY_ENTER;
    }

    left_pressed = lv_port_key_is_pressed(Key_Left_GPIO_Port, Key_Left_Pin);
    right_pressed = lv_port_key_is_pressed(Key_Right_GPIO_Port, Key_Right_Pin);

    if ((left_pressed != 0U) && (right_pressed == 0U))
    {
        lr_key = LV_KEY_LEFT;
    }
    else if ((right_pressed != 0U) && (left_pressed == 0U))
    {
        lr_key = LV_KEY_RIGHT;
    }
    else
    {
        lr_key = 0U;
    }

    if (lr_key == 0U)
    {
        s_lr_active_key = 0U;
        return 0U;
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
