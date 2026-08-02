#ifndef LV_PORT_INDEV_H
#define LV_PORT_INDEV_H

#include "lvgl.h"

void lv_port_indev_init(void);
lv_indev_t *lv_port_indev_get_keypad(void);
void lv_port_indev_suppress_exit_keys_until_release(void);
void lv_port_indev_suppress_all_keys_until_release(void);

#endif /* LV_PORT_INDEV_H */
