#ifndef NES_RUNTIME_H
#define NES_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NES_RUNTIME_RUNNING               0
#define NES_RUNTIME_STOP_KEY3             1
#define NES_RUNTIME_STOP_KEY2             2
#define NES_RUNTIME_ERR_CACHE             -1
#define NES_RUNTIME_ERR_MAPPER            -2
#define NES_RUNTIME_ERR_ROM               -3
#define NES_RUNTIME_ERR_MEMORY            -4
#define NES_RUNTIME_ERR_CPU               -5
#define NES_RUNTIME_ERR_DISPLAY           -6
#define NES_RUNTIME_ERR_SAVE              -7

#define NES_RUNTIME_SAVE_NONE              0
#define NES_RUNTIME_SAVE_WRITTEN           1
#define NES_RUNTIME_SAVE_LOADED            2

typedef struct
{
  uint32_t frame_count;
  uint32_t cpu_cycles;
  uint16_t pc;
  uint16_t scanline;
  uint8_t opcode;
  uint8_t mapper;
  uint8_t prg_bank;
  uint8_t chr_bank;
  uint8_t mmc1_control;
  uint8_t mmc1_prg;
  uint8_t save_loaded;
  uint8_t save_dirty;
  uint8_t last_fs_error;
} NES_RuntimeDiagnostics;

/* Phase 4 runtime scope: NTSC iNES 1.0, Mapper 0/1/2/3, no audio output. */
int8_t NES_Runtime_Start(void);
int8_t NES_Runtime_Process(void);
int8_t NES_Runtime_Stop(uint8_t persist_battery_save);
uint8_t NES_Runtime_IsActive(void);
uint32_t NES_Runtime_GetFrameCount(void);
uint32_t NES_Runtime_GetRenderedFrameCount(void);
void NES_Runtime_GetDiagnostics(NES_RuntimeDiagnostics *diagnostics);
/* Mobile input uses the native NES serial order: bit0 A, bit1 B, bit2 Select,
 * bit3 Start, bit4 Up, bit5 Down, bit6 Left, bit7 Right. */
void NES_Runtime_SetRemoteButtons(uint8_t buttons);
void NES_Runtime_ClearRemoteButtons(void);
void NES_Runtime_RequestRemoteReset(void);

#ifdef __cplusplus
}
#endif

#endif /* NES_RUNTIME_H */
