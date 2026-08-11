#ifndef UI_SETTINGS_H
#define UI_SETTINGS_H

#include <stdint.h>

typedef enum
{
  UI_SETTINGS_ROTATION_0 = 0,
  UI_SETTINGS_ROTATION_90,
  UI_SETTINGS_ROTATION_180,
  UI_SETTINGS_ROTATION_270,
  UI_SETTINGS_ROTATION_COUNT
} UI_SettingsRotation;

typedef enum
{
  UI_SETTINGS_DIRECTION_UP = 0,
  UI_SETTINGS_DIRECTION_RIGHT,
  UI_SETTINGS_DIRECTION_DOWN,
  UI_SETTINGS_DIRECTION_LEFT
} UI_SettingsDirection;

typedef enum
{
  UI_SETTINGS_OK = 0,
  UI_SETTINGS_ERR_QSPI = -1,
  UI_SETTINGS_ERR_VERIFY = -2
} UI_SettingsResult;

/* Call once after LCD and QSPI initialization, before showing direct media or
 * creating the first LVGL page.  qspi_ready may be zero; runtime settings will
 * still work, but persistence is then disabled for this boot. */
void UI_Settings_Init(uint8_t qspi_ready);

uint8_t UI_Settings_GetRotation(void);
uint16_t UI_Settings_GetRotationDegrees(void);
uint8_t UI_Settings_GetKeySoundEnabled(void);
uint8_t UI_Settings_GetChineseEnabled(void);

void UI_Settings_SetRotation(uint8_t rotation);
void UI_Settings_SetKeySoundEnabled(uint8_t enabled);
void UI_Settings_SetChineseEnabled(uint8_t enabled);
UI_SettingsDirection UI_Settings_MapDirection(UI_SettingsDirection physical);

/* Settings writes are delayed so key-repeat cannot wear one flash sector.
 * RequestSaveNow is used when leaving the settings page; Process performs the
 * actual dual-slot write from normal thread context. */
void UI_Settings_RequestSaveNow(void);
void UI_Settings_Process(void);
int8_t UI_Settings_GetLastSaveResult(void);

/* Non-blocking active-high buzzer service.  NotifyKeyPress starts one short
 * chirp; BuzzerTick1ms must be called by the existing 1 kHz TIM6 callback. */
void UI_Settings_NotifyKeyPress(void);
void UI_Settings_BuzzerTick1ms(void);

#endif /* UI_SETTINGS_H */
