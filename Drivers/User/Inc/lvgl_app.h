#ifndef LVGL_APP_H
#define LVGL_APP_H

#include <stdint.h>

void LVGL_App_Init(void);
void LVGL_App_Process(void);
uint8_t LVGL_App_IsCommandControlActive(void);
uint8_t LVGL_App_CommandSetMotorSpeed(uint8_t motor_index, int16_t speed_percent);
void LVGL_App_CommandStopMotors(void);
void lvgl_app_com_rx_cb(uint8_t *buf, uint32_t len);
void lvgl_app_com_rx_channel_cb(uint8_t channel, uint8_t *buf, uint32_t len);

#endif /* LVGL_APP_H */
