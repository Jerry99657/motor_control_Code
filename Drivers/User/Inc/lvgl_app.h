#ifndef LVGL_APP_H
#define LVGL_APP_H

#include "ui_navigation.h"
#include <stdint.h>

void LVGL_App_Init(void);
void LVGL_App_Process(void);
void lvgl_app_com_rx_cb(uint8_t *buf, uint32_t len);
void lvgl_app_com_rx_channel_cb(uint8_t channel, uint8_t *buf, uint32_t len);
void LVGL_App_GetNavigationSnapshot(UiNavigationSnapshot *snapshot);

#endif /* LVGL_APP_H */
