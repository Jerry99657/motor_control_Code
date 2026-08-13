#include "ui_settings.h"

#include "lcd_spi_154.h"
#include "lv_port_disp.h"
#include "main.h"
#include "qspi_partition.h"
#include "qspi_w25q64.h"

#include <string.h>

#define UI_SETTINGS_MAGIC               0x54534955U /* "UIST" */
#define UI_SETTINGS_FORMAT_VERSION      1U
#define UI_SETTINGS_RECORD_SIZE         32U
#define UI_SETTINGS_CRC_OFFSET          28U
#define UI_SETTINGS_SAVE_DELAY_MS       800U
#define UI_SETTINGS_BUZZER_DURATION_MS  22U

static uint8_t s_rotation = UI_SETTINGS_ROTATION_0;
static uint8_t s_key_sound_enabled = 1U;
static uint8_t s_chinese_enabled = 0U;
static uint8_t s_low_battery_alarm_enabled = 0U;
static uint8_t s_storage_available = 0U;
static uint8_t s_dirty = 0U;
static uint8_t s_active_slot = 0xFFU;
static uint32_t s_sequence = 0U;
static uint32_t s_save_due_tick = 0U;
static int8_t s_last_save_result = UI_SETTINGS_OK;
static volatile uint16_t s_buzzer_ticks = 0U;
static volatile uint8_t s_low_battery_alert = 0U;

static uint16_t ui_settings_read_u16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t ui_settings_read_u32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void ui_settings_write_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static void ui_settings_write_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static uint32_t ui_settings_crc32(const uint8_t *data, uint32_t size)
{
  uint32_t crc = 0xFFFFFFFFU;
  uint32_t i;
  uint8_t bit;

  for (i = 0U; i < size; ++i)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xEDB88320U);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

static uint8_t ui_settings_record_valid(const uint8_t *record)
{
  if ((ui_settings_read_u32(&record[0]) != UI_SETTINGS_MAGIC) ||
      (ui_settings_read_u16(&record[4]) != UI_SETTINGS_FORMAT_VERSION) ||
      (ui_settings_read_u16(&record[6]) != UI_SETTINGS_RECORD_SIZE) ||
      (record[12] >= UI_SETTINGS_ROTATION_COUNT) ||
      (record[13] > 1U) ||
      ((record[14] != 0xFFU) && (record[14] > 1U)) ||
      ((record[15] != 0xFFU) && (record[15] > 1U)))
  {
    return 0U;
  }

  return (ui_settings_crc32(record, UI_SETTINGS_CRC_OFFSET) ==
          ui_settings_read_u32(&record[UI_SETTINGS_CRC_OFFSET])) ? 1U : 0U;
}

static void ui_settings_build_record(uint8_t *record, uint32_t sequence)
{
  memset(record, 0xFF, UI_SETTINGS_RECORD_SIZE);
  ui_settings_write_u32(&record[0], UI_SETTINGS_MAGIC);
  ui_settings_write_u16(&record[4], UI_SETTINGS_FORMAT_VERSION);
  ui_settings_write_u16(&record[6], UI_SETTINGS_RECORD_SIZE);
  ui_settings_write_u32(&record[8], sequence);
  record[12] = s_rotation;
  record[13] = s_key_sound_enabled;
  record[14] = s_chinese_enabled;
  record[15] = s_low_battery_alarm_enabled;
  ui_settings_write_u32(&record[UI_SETTINGS_CRC_OFFSET],
                        ui_settings_crc32(record, UI_SETTINGS_CRC_OFFSET));
}

static void ui_settings_apply_rotation(uint8_t wait_for_flush)
{
  static const uint8_t lcd_direction[UI_SETTINGS_ROTATION_COUNT] =
  {
    Direction_H_Flip, /* Board's original orientation. */
    Direction_V,
    Direction_H,
    Direction_V_Flip
  };

  if (wait_for_flush != 0U)
  {
    (void)lv_port_disp_wait_idle(250U);
  }
  LCD_SetDirection(lcd_direction[s_rotation]);
}

static int8_t ui_settings_save(void)
{
  uint8_t record[UI_SETTINGS_RECORD_SIZE];
  uint8_t verify[UI_SETTINGS_RECORD_SIZE];
  uint8_t was_mapped;
  uint8_t target_slot;
  uint32_t target_address;
  uint32_t next_sequence = s_sequence + 1U;
  int8_t result = UI_SETTINGS_ERR_QSPI;

  if (s_storage_available == 0U)
  {
    return UI_SETTINGS_ERR_QSPI;
  }

  target_slot = (s_active_slot == 0U) ? 1U : 0U;
  target_address = (target_slot == 0U) ?
                   QSPI_PARTITION_UI_SETTINGS_SLOT_A :
                   QSPI_PARTITION_UI_SETTINGS_SLOT_B;
  ui_settings_build_record(record, next_sequence);
  was_mapped = QSPI_W25Qxx_IsMemoryMapped();

  if ((was_mapped != 0U) &&
      (QSPI_W25Qxx_ExitMemoryMappedMode() != QSPI_W25QXX_OK))
  {
    return UI_SETTINGS_ERR_QSPI;
  }

  if ((QSPI_W25Qxx_SectorErase(target_address) == QSPI_W25QXX_OK) &&
      (QSPI_W25Qxx_WriteBuffer(record, target_address,
                               UI_SETTINGS_RECORD_SIZE) == QSPI_W25QXX_OK) &&
      (QSPI_W25Qxx_ReadBuffer(verify, target_address,
                              UI_SETTINGS_RECORD_SIZE) == QSPI_W25QXX_OK))
  {
    if ((memcmp(record, verify, UI_SETTINGS_RECORD_SIZE) == 0) &&
        (ui_settings_record_valid(verify) != 0U))
    {
      s_active_slot = target_slot;
      s_sequence = next_sequence;
      result = UI_SETTINGS_OK;
    }
    else
    {
      result = UI_SETTINGS_ERR_VERIFY;
    }
  }

  if ((was_mapped != 0U) &&
      (QSPI_W25Qxx_MemoryMappedMode() != QSPI_W25QXX_OK) &&
      (result == UI_SETTINGS_OK))
  {
    result = UI_SETTINGS_ERR_QSPI;
  }
  return result;
}

void UI_Settings_Init(uint8_t qspi_ready)
{
  uint8_t record_a[UI_SETTINGS_RECORD_SIZE];
  uint8_t record_b[UI_SETTINGS_RECORD_SIZE];
  uint8_t valid_a = 0U;
  uint8_t valid_b = 0U;
  const uint8_t *selected = NULL;

  s_rotation = UI_SETTINGS_ROTATION_0;
  s_key_sound_enabled = 1U;
  s_chinese_enabled = 0U;
  s_low_battery_alarm_enabled = 0U;
  s_storage_available = (qspi_ready != 0U) ? 1U : 0U;
  s_dirty = 0U;
  s_active_slot = 0xFFU;
  s_sequence = 0U;
  s_last_save_result = UI_SETTINGS_OK;
  s_buzzer_ticks = 0U;
  s_low_battery_alert = 0U;
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  if (s_storage_available != 0U)
  {
    valid_a = ((QSPI_W25Qxx_ReadBuffer(record_a,
                QSPI_PARTITION_UI_SETTINGS_SLOT_A,
                UI_SETTINGS_RECORD_SIZE) == QSPI_W25QXX_OK) &&
               (ui_settings_record_valid(record_a) != 0U)) ? 1U : 0U;
    valid_b = ((QSPI_W25Qxx_ReadBuffer(record_b,
                QSPI_PARTITION_UI_SETTINGS_SLOT_B,
                UI_SETTINGS_RECORD_SIZE) == QSPI_W25QXX_OK) &&
               (ui_settings_record_valid(record_b) != 0U)) ? 1U : 0U;

    if ((valid_a != 0U) && (valid_b != 0U))
    {
      if ((int32_t)(ui_settings_read_u32(&record_b[8]) -
                    ui_settings_read_u32(&record_a[8])) > 0)
      {
        selected = record_b;
        s_active_slot = 1U;
      }
      else
      {
        selected = record_a;
        s_active_slot = 0U;
      }
    }
    else if (valid_a != 0U)
    {
      selected = record_a;
      s_active_slot = 0U;
    }
    else if (valid_b != 0U)
    {
      selected = record_b;
      s_active_slot = 1U;
    }

    if (selected != NULL)
    {
      s_sequence = ui_settings_read_u32(&selected[8]);
      s_rotation = selected[12];
      s_key_sound_enabled = selected[13];
      /* Byte 14 was 0xFF in records written before the language option was
       * introduced. Preserve those valid records and default them to English. */
      s_chinese_enabled = (selected[14] <= 1U) ? selected[14] : 0U;
      /* Byte 15 is 0xFF in settings written before the battery-alarm option.
       * Preserve the record and use the safe default (alarm disabled). */
      s_low_battery_alarm_enabled =
          (selected[15] <= 1U) ? selected[15] : 0U;
    }
  }

  ui_settings_apply_rotation(0U);
}

uint8_t UI_Settings_GetRotation(void)
{
  return s_rotation;
}

uint16_t UI_Settings_GetRotationDegrees(void)
{
  return (uint16_t)s_rotation * 90U;
}

uint8_t UI_Settings_GetKeySoundEnabled(void)
{
  return s_key_sound_enabled;
}

uint8_t UI_Settings_GetChineseEnabled(void)
{
  return s_chinese_enabled;
}

uint8_t UI_Settings_GetLowBatteryAlarmEnabled(void)
{
  return s_low_battery_alarm_enabled;
}

void UI_Settings_SetRotation(uint8_t rotation)
{
  if ((rotation >= UI_SETTINGS_ROTATION_COUNT) || (rotation == s_rotation))
  {
    return;
  }

  s_rotation = rotation;
  ui_settings_apply_rotation(1U);
  s_dirty = 1U;
  s_save_due_tick = HAL_GetTick() + UI_SETTINGS_SAVE_DELAY_MS;
}

void UI_Settings_SetKeySoundEnabled(uint8_t enabled)
{
  enabled = (enabled != 0U) ? 1U : 0U;
  if (enabled == s_key_sound_enabled)
  {
    return;
  }

  s_key_sound_enabled = enabled;
  s_dirty = 1U;
  s_save_due_tick = HAL_GetTick() + UI_SETTINGS_SAVE_DELAY_MS;
  if (enabled == 0U)
  {
    s_buzzer_ticks = 0U;
    if (s_low_battery_alert == 0U)
    {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
  }
}

void UI_Settings_SetChineseEnabled(uint8_t enabled)
{
  enabled = (enabled != 0U) ? 1U : 0U;
  if (enabled == s_chinese_enabled)
  {
    return;
  }

  s_chinese_enabled = enabled;
  s_dirty = 1U;
  s_save_due_tick = HAL_GetTick() + UI_SETTINGS_SAVE_DELAY_MS;
}

void UI_Settings_SetLowBatteryAlarmEnabled(uint8_t enabled)
{
  enabled = (enabled != 0U) ? 1U : 0U;
  if (enabled == s_low_battery_alarm_enabled)
  {
    return;
  }

  s_low_battery_alarm_enabled = enabled;
  s_dirty = 1U;
  s_save_due_tick = HAL_GetTick() + UI_SETTINGS_SAVE_DELAY_MS;
  if (enabled == 0U)
  {
    /* Disabling the option must silence an active alarm immediately. */
    UI_Settings_SetLowBatteryAlert(0U);
  }
}

UI_SettingsDirection UI_Settings_MapDirection(UI_SettingsDirection physical)
{
  static const UI_SettingsDirection mapping[UI_SETTINGS_ROTATION_COUNT][4] =
  {
    {UI_SETTINGS_DIRECTION_UP,    UI_SETTINGS_DIRECTION_RIGHT,
     UI_SETTINGS_DIRECTION_DOWN,  UI_SETTINGS_DIRECTION_LEFT},
    {UI_SETTINGS_DIRECTION_LEFT,  UI_SETTINGS_DIRECTION_UP,
     UI_SETTINGS_DIRECTION_RIGHT, UI_SETTINGS_DIRECTION_DOWN},
    {UI_SETTINGS_DIRECTION_DOWN,  UI_SETTINGS_DIRECTION_LEFT,
     UI_SETTINGS_DIRECTION_UP,    UI_SETTINGS_DIRECTION_RIGHT},
    {UI_SETTINGS_DIRECTION_RIGHT, UI_SETTINGS_DIRECTION_DOWN,
     UI_SETTINGS_DIRECTION_LEFT,  UI_SETTINGS_DIRECTION_UP}
  };

  if ((physical > UI_SETTINGS_DIRECTION_LEFT) ||
      (s_rotation >= UI_SETTINGS_ROTATION_COUNT))
  {
    return physical;
  }
  return mapping[s_rotation][physical];
}

void UI_Settings_RequestSaveNow(void)
{
  if (s_dirty != 0U)
  {
    s_save_due_tick = HAL_GetTick();
  }
}

void UI_Settings_Process(void)
{
  if ((s_dirty == 0U) ||
      ((int32_t)(HAL_GetTick() - s_save_due_tick) < 0))
  {
    return;
  }

  s_last_save_result = ui_settings_save();
  if (s_last_save_result == UI_SETTINGS_OK)
  {
    s_dirty = 0U;
  }
  else
  {
    /* Keep the live setting, but throttle retries after a transient QSPI
     * failure instead of blocking every main-loop pass. */
    s_save_due_tick = HAL_GetTick() + 2000U;
  }
}

int8_t UI_Settings_GetLastSaveResult(void)
{
  return s_last_save_result;
}

void UI_Settings_NotifyKeyPress(void)
{
  if ((s_key_sound_enabled == 0U) || (s_low_battery_alert != 0U))
  {
    return;
  }

  s_buzzer_ticks = UI_SETTINGS_BUZZER_DURATION_MS;
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void UI_Settings_SetLowBatteryAlert(uint8_t active)
{
  if (s_low_battery_alarm_enabled == 0U)
  {
    active = 0U;
  }
  active = (active != 0U) ? 1U : 0U;
  if (active == s_low_battery_alert)
  {
    return;
  }

  s_low_battery_alert = active;
  if (active != 0U)
  {
    s_buzzer_ticks = 0U;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  }
}

void UI_Settings_BuzzerTick1ms(void)
{
  if (s_low_battery_alert != 0U)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    return;
  }

  if (s_buzzer_ticks == 0U)
  {
    return;
  }

  --s_buzzer_ticks;
  if (s_buzzer_ticks == 0U)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  }
}
