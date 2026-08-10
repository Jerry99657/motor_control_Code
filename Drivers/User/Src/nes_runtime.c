#include "nes_runtime.h"

#include "fatfs.h"
#include "lcd_spi_154.h"
#include "main.h"
#include "media_memory.h"
#include "nes_cpu.h"
#include "nes_rom_cache.h"
#include "sd_diskio.h"

#include <stddef.h>
#include <string.h>

#define NES_FRAME_WIDTH             256U
#define NES_FRAME_HEIGHT            240U
#define NES_LCD_CROP_X              8U
#define NES_LCD_WIDTH               240U
#define NES_LCD_HEIGHT              240U
#define NES_FRAMEBUFFER_BYTES       (NES_LCD_WIDTH * NES_LCD_HEIGHT * 2U)
#define NES_PRG_RAM_BYTES           8192U
#define NES_CHR_RAM_BYTES           8192U
#define NES_CPU_RAM_BYTES           2048U
#define NES_NAMETABLE_BYTES         4096U
#define NES_LCD_STOP_TIMEOUT_MS     500U
#define NES_FRAME_LATE_RESET_MS     100U
#define NES_SOFT_RESET_HOLD_MS      1200U
#define NES_SELECT_DOUBLE_CLICK_MS  350U
#define NES_SELECT_PULSE_FRAMES     4U
#define NES_REMOTE_TIMEOUT_MS       350U
#define NES_PRG_BANK_BYTES          16384U
#define NES_CHR_BANK_BYTES          8192U
#define NES_CHR_HALF_BANK_BYTES     4096U
#define NES_MMC1_MAX_PRG_BYTES      (512U * 1024U)
#define NES_APU_4STEP_CYCLES        29830U
#define NES_APU_5STEP_CYCLES        37282U
#define NES_SAVE_PATH_SIZE          512U
#define NES_SAVE_IO_CHUNK           4096U

#define NES_MIRROR_ONE_LOWER        0U
#define NES_MIRROR_ONE_UPPER        1U
#define NES_MIRROR_VERTICAL         2U
#define NES_MIRROR_HORIZONTAL       3U
#define NES_MIRROR_FOUR_SCREEN      4U

_Static_assert(LCD_Width == NES_LCD_WIDTH,
               "NES crop assumes a 240-pixel LCD width");
_Static_assert(LCD_Height == NES_LCD_HEIGHT,
               "NES runtime assumes a 240-pixel LCD height");

typedef struct
{
  uint8_t ctrl;
  uint8_t mask;
  uint8_t status;
  uint8_t oam_addr;
  uint16_t v;
  uint16_t t;
  uint8_t fine_x;
  uint8_t write_toggle;
  uint8_t read_buffer;
  uint8_t open_bus;
  uint8_t palette[32];
  uint8_t oam[256];
  uint8_t nametable[NES_NAMETABLE_BYTES];
} NES_PPU;

typedef struct
{
  uint32_t frame_cycle;
  uint8_t channel_enable;
  uint8_t mode_5step;
  uint8_t irq_inhibit;
  uint8_t frame_irq;
} NES_APU;

typedef struct
{
  NES_CPU cpu;
  NES_PPU ppu;
  NES_APU apu;
  uint8_t cpu_ram[NES_CPU_RAM_BYTES];
  uint8_t prg_ram[NES_PRG_RAM_BYTES];
  uint8_t chr_ram[NES_CHR_RAM_BYTES];
  uint8_t bg_opaque[NES_FRAME_WIDTH];
  uint8_t sprite_occupied[NES_FRAME_WIDTH];
  uint16_t line_pixels[NES_FRAME_WIDTH];
  NES_RomCacheMetadata metadata;
  const uint8_t *mapped_rom;
  const uint8_t *prg_rom;
  const uint8_t *chr_rom;
  uint16_t *framebuffer;
  uint32_t framebuffer_capacity;
  uint32_t frame_count;
  uint32_t rendered_frame_count;
  uint32_t next_frame_tick;
  uint32_t start_combo_tick;
  uint32_t key1_last_press_tick;
  uint16_t scanline_cycle_remainder;
  uint16_t cpu_cycle_debt;
  uint16_t cpu_stall_cycles;
  uint16_t prg_bank_count;
  uint16_t chr_bank_count;
  uint16_t chr_half_bank_count;
  uint16_t current_scanline;
  uint16_t mmc1_prg_bank_lo;
  uint16_t mmc1_prg_bank_hi;
  uint16_t mmc1_chr_bank_lo;
  uint16_t mmc1_chr_bank_hi;
  uint8_t controller_latch;
  uint8_t controller_shift;
  uint8_t controller_strobe;
  uint8_t prg_bank;
  uint8_t chr_bank;
  uint8_t chr_is_ram;
  uint8_t mirroring_mode;
  uint8_t mmc1_control;
  uint8_t mmc1_chr0;
  uint8_t mmc1_chr1;
  uint8_t mmc1_prg;
  uint8_t mmc1_shift;
  uint8_t mmc1_last_write_valid;
  uint8_t prg_ram_enabled;
  uint8_t prg_ram_dirty;
  uint8_t battery_backed;
  uint8_t save_loaded;
  uint8_t last_fs_error;
  uint8_t exit_armed;
  uint8_t frame_tick_phase;
  uint8_t start_combo_active;
  uint8_t start_combo_reset;
  uint8_t key1_last_pressed;
  uint8_t key1_click_armed;
  uint8_t select_pulse_frames;
  uint8_t force_draw;
  uint32_t mmc1_last_write_cycle;
} NES_RuntimeContext;

/* DTCM is CPU-only, so the interpreter state is fast and never needs cache
 * maintenance.  The LCD framebuffer remains in the shared AXI SRAM pool. */
static NES_RuntimeContext s_nes
  __attribute__((section(".ram_dtcm"), aligned(32)));
/* .ram_dtcm is NOLOAD and therefore undefined at reset.  Keep the ownership
 * flag in normal zero-initialized BSS so an uninitialized DTCM byte can never
 * make the first Start() look as if the emulator were already active. */
static volatile uint8_t s_nes_active = 0U;
static char s_save_path[NES_SAVE_PATH_SIZE];
static char s_save_temp_path[NES_SAVE_PATH_SIZE];
static char s_save_backup_path[NES_SAVE_PATH_SIZE];
static uint8_t s_save_io_buffer[NES_SAVE_IO_CHUNK]
  __attribute__((section(".ram_d2"), aligned(32)));
static volatile uint8_t s_nes_remote_buttons = 0U;
static volatile uint8_t s_nes_remote_valid = 0U;
static volatile uint8_t s_nes_remote_reset_pending = 0U;
static volatile uint32_t s_nes_remote_tick = 0U;

/* RGB565 palette retained from the supplied ye781205/ALIENTEK emulator
 * reference.  Its readme permits reuse with attribution. */
static const uint16_t s_nes_palette[64] =
{
  0x73AE,0x20D1,0x0015,0x4013,0x880E,0xA802,0xA000,0x7840,
  0x4160,0x0220,0x0280,0x01E2,0x19EB,0x0000,0x0000,0x0000,
  0xBDF7,0x039D,0x21DD,0x801E,0xB817,0xE00B,0xD940,0xCA61,
  0x8B80,0x04A0,0x0540,0x0487,0x0411,0x0000,0x0000,0x0000,
  0xF79E,0x3DFF,0x5CBF,0xA45F,0xF3DF,0xFBB6,0xFBAC,0xFCC7,
  0xF5E7,0x8682,0x4EE9,0x5FD3,0x075B,0x0000,0x0000,0x0000,
  0xF79E,0xAF3F,0xC6BF,0xD65F,0xFE3F,0xFE3B,0xFDF6,0xFED5,
  0xFF34,0xE7F4,0xAF97,0xB7F9,0x9FFE,0x0000,0x0000,0x0000
};

static uint8_t nes_key_pressed(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

void NES_Runtime_SetRemoteButtons(uint8_t buttons)
{
  s_nes_remote_tick = HAL_GetTick();
  s_nes_remote_buttons = buttons;
  __DMB();
  s_nes_remote_valid = 1U;
}

void NES_Runtime_ClearRemoteButtons(void)
{
  s_nes_remote_valid = 0U;
  __DMB();
  s_nes_remote_buttons = 0U;
  s_nes_remote_tick = 0U;
}

void NES_Runtime_RequestRemoteReset(void)
{
  if (s_nes_active != 0U)
  {
    s_nes_remote_reset_pending = 1U;
    __DMB();
  }
}

static uint8_t nes_remote_buttons_sample(void)
{
  if (s_nes_remote_valid == 0U)
  {
    return 0U;
  }
  if ((HAL_GetTick() - s_nes_remote_tick) >= NES_REMOTE_TIMEOUT_MS)
  {
    NES_Runtime_ClearRemoteButtons();
    return 0U;
  }
  return s_nes_remote_buttons;
}

static uint8_t nes_controller_sample(void)
{
  uint8_t buttons = 0U;
  uint8_t ok = nes_key_pressed(Key_OK_GPIO_Port, Key_OK_Pin);
  uint8_t key1 = nes_key_pressed(Key1_GPIO_Port, Key1_Pin);
  uint8_t up = nes_key_pressed(Key_Up_GPIO_Port, Key_Up_Pin);
  uint8_t down = nes_key_pressed(Key_Down_GPIO_Port, Key_Down_Pin);
  uint8_t left = nes_key_pressed(Key_Left_GPIO_Port, Key_Left_Pin);
  uint8_t right = nes_key_pressed(Key_Right_GPIO_Port, Key_Right_Pin);

  /* Serial order: A, B, Select, Start, Up, Down, Left, Right.
   * OK belongs to the five-way switch, while KEY1 is independent, so only
   * that cross-device pair is used for Start. */
  if ((ok != 0U) && (key1 != 0U))
  {
    buttons |= 0x08U; /* Start */
  }
  else
  {
    if (ok != 0U) buttons |= 0x01U;   /* A */
    if (key1 != 0U) buttons |= 0x02U; /* B */
  }
  if (s_nes.select_pulse_frames != 0U)
  {
    buttons = (uint8_t)((buttons & (uint8_t)~0x02U) | 0x04U);
  }
  if (up != 0U) buttons |= 0x10U;
  if (down != 0U) buttons |= 0x20U;
  if (left != 0U) buttons |= 0x40U;
  if (right != 0U) buttons |= 0x80U;
  buttons |= nes_remote_buttons_sample();

  /* Neutralize impossible/opposed directions after merging phone and board
   * input. This prevents a held phone direction fighting the five-way key. */
  if ((buttons & 0x30U) == 0x30U) buttons &= (uint8_t)~0x30U;
  if ((buttons & 0xC0U) == 0xC0U) buttons &= (uint8_t)~0xC0U;
  return buttons;
}

static void nes_mmc1_apply_mirroring(NES_RuntimeContext *ctx)
{
  if ((ctx->metadata.flags6 & 0x08U) != 0U)
  {
    ctx->mirroring_mode = NES_MIRROR_FOUR_SCREEN;
    return;
  }

  switch (ctx->mmc1_control & 3U)
  {
    case 0U: ctx->mirroring_mode = NES_MIRROR_ONE_LOWER; break;
    case 1U: ctx->mirroring_mode = NES_MIRROR_ONE_UPPER; break;
    case 2U: ctx->mirroring_mode = NES_MIRROR_VERTICAL; break;
    default: ctx->mirroring_mode = NES_MIRROR_HORIZONTAL; break;
  }
}

static void nes_mmc1_update_banks(NES_RuntimeContext *ctx)
{
  uint16_t region_base = 0U;
  uint8_t prg_mode = (ctx->mmc1_control >> 2U) & 3U;

  if (ctx->prg_bank_count > 16U)
  {
    region_base = ((ctx->mmc1_chr0 & 0x10U) != 0U) ? 16U : 0U;
  }

  if (prg_mode <= 1U)
  {
    ctx->mmc1_prg_bank_lo =
      (uint16_t)(region_base + (ctx->mmc1_prg & 0x0EU));
    ctx->mmc1_prg_bank_hi = (uint16_t)(ctx->mmc1_prg_bank_lo + 1U);
  }
  else if (prg_mode == 2U)
  {
    ctx->mmc1_prg_bank_lo = region_base;
    ctx->mmc1_prg_bank_hi =
      (uint16_t)(region_base + (ctx->mmc1_prg & 0x0FU));
  }
  else
  {
    ctx->mmc1_prg_bank_lo =
      (uint16_t)(region_base + (ctx->mmc1_prg & 0x0FU));
    ctx->mmc1_prg_bank_hi = (uint16_t)(region_base + 15U);
    if (ctx->mmc1_prg_bank_hi >= ctx->prg_bank_count)
    {
      ctx->mmc1_prg_bank_hi = (uint16_t)(ctx->prg_bank_count - 1U);
    }
  }
  ctx->mmc1_prg_bank_lo =
    (uint16_t)(ctx->mmc1_prg_bank_lo % ctx->prg_bank_count);
  ctx->mmc1_prg_bank_hi =
    (uint16_t)(ctx->mmc1_prg_bank_hi % ctx->prg_bank_count);

  if ((ctx->mmc1_control & 0x10U) != 0U)
  {
    ctx->mmc1_chr_bank_lo = ctx->mmc1_chr0;
    ctx->mmc1_chr_bank_hi = ctx->mmc1_chr1;
  }
  else
  {
    ctx->mmc1_chr_bank_lo = ctx->mmc1_chr0 & 0x1EU;
    ctx->mmc1_chr_bank_hi = (uint16_t)(ctx->mmc1_chr_bank_lo + 1U);
  }
  ctx->mmc1_chr_bank_lo =
    (uint16_t)(ctx->mmc1_chr_bank_lo % ctx->chr_half_bank_count);
  ctx->mmc1_chr_bank_hi =
    (uint16_t)(ctx->mmc1_chr_bank_hi % ctx->chr_half_bank_count);
}

static void nes_mmc1_reset(NES_RuntimeContext *ctx)
{
  ctx->mmc1_control = 0x0CU;
  ctx->mmc1_chr0 = 0U;
  ctx->mmc1_chr1 = 0U;
  ctx->mmc1_prg = 0U;
  ctx->mmc1_shift = 0x10U;
  ctx->mmc1_last_write_valid = 0U;
  ctx->mmc1_last_write_cycle = 0U;
  ctx->prg_ram_enabled = 1U;
  nes_mmc1_apply_mirroring(ctx);
  nes_mmc1_update_banks(ctx);
}

static void nes_mmc1_write(NES_RuntimeContext *ctx,
                           uint16_t address, uint8_t value)
{
  uint8_t complete;
  uint8_t reg;

  /* MMC1 ignores the second bus write of an RMW instruction.  This CPU core
   * updates total_cycles after an instruction, so both writes share a cycle
   * stamp and can be filtered without making the bus callback cycle-aware. */
  if ((ctx->mmc1_last_write_valid != 0U) &&
      (ctx->mmc1_last_write_cycle == ctx->cpu.total_cycles))
  {
    return;
  }
  ctx->mmc1_last_write_valid = 1U;
  ctx->mmc1_last_write_cycle = ctx->cpu.total_cycles;

  if ((value & 0x80U) != 0U)
  {
    ctx->mmc1_shift = 0x10U;
    ctx->mmc1_control |= 0x0CU;
    nes_mmc1_apply_mirroring(ctx);
    nes_mmc1_update_banks(ctx);
    return;
  }

  complete = ctx->mmc1_shift & 1U;
  ctx->mmc1_shift = (uint8_t)((ctx->mmc1_shift >> 1U) |
                              ((value & 1U) << 4U));
  if (complete == 0U)
  {
    return;
  }

  reg = (uint8_t)((address >> 13U) & 3U);
  switch (reg)
  {
    case 0U:
      ctx->mmc1_control = ctx->mmc1_shift & 0x1FU;
      nes_mmc1_apply_mirroring(ctx);
      break;
    case 1U:
      ctx->mmc1_chr0 = ctx->mmc1_shift & 0x1FU;
      break;
    case 2U:
      ctx->mmc1_chr1 = ctx->mmc1_shift & 0x1FU;
      break;
    default:
      ctx->mmc1_prg = ctx->mmc1_shift & 0x1FU;
      ctx->prg_ram_enabled = ((ctx->mmc1_prg & 0x10U) == 0U) ? 1U : 0U;
      break;
  }
  ctx->mmc1_shift = 0x10U;
  nes_mmc1_update_banks(ctx);
}

static uint32_t nes_mmc1_prg_offset(const NES_RuntimeContext *ctx,
                                    uint16_t address)
{
  uint16_t bank = (address < 0xC000U) ?
                    ctx->mmc1_prg_bank_lo : ctx->mmc1_prg_bank_hi;
  return ((uint32_t)bank * NES_PRG_BANK_BYTES) +
         (uint32_t)(address & (NES_PRG_BANK_BYTES - 1U));
}

static uint32_t nes_chr_offset(const NES_RuntimeContext *ctx,
                               uint16_t address)
{
  uint16_t bank;

  if (ctx->metadata.mapper == 1U)
  {
    bank = (address < 0x1000U) ?
             ctx->mmc1_chr_bank_lo : ctx->mmc1_chr_bank_hi;
    return ((uint32_t)bank * NES_CHR_HALF_BANK_BYTES) +
           (uint32_t)(address & (NES_CHR_HALF_BANK_BYTES - 1U));
  }

  bank = (ctx->metadata.mapper == 3U) ? ctx->chr_bank : 0U;
  return ((uint32_t)bank * NES_CHR_BANK_BYTES) + address;
}

static uint16_t nes_nametable_index(const NES_RuntimeContext *ctx,
                                    uint16_t address)
{
  uint16_t logical = (uint16_t)((address - 0x2000U) & 0x0FFFU);
  uint16_t table = logical >> 10;
  uint16_t offset = logical & 0x03FFU;
  uint16_t physical;

  switch (ctx->mirroring_mode)
  {
    case NES_MIRROR_ONE_LOWER: physical = 0U; break;
    case NES_MIRROR_ONE_UPPER: physical = 1U; break;
    case NES_MIRROR_VERTICAL: physical = table & 1U; break;
    case NES_MIRROR_FOUR_SCREEN: physical = table; break;
    default: physical = table >> 1; break;
  }
  return (uint16_t)((physical << 10) | offset);
}

static uint8_t nes_palette_index(uint16_t address)
{
  uint8_t index = (uint8_t)(address & 0x1FU);
  if ((index & 0x13U) == 0x10U)
  {
    index &= 0x0FU;
  }
  return index;
}

static uint8_t nes_ppu_read(NES_RuntimeContext *ctx, uint16_t address)
{
  address &= 0x3FFFU;
  if (address < 0x2000U)
  {
    if (ctx->chr_is_ram != 0U)
    {
      return ctx->chr_ram[nes_chr_offset(ctx, address)];
    }
    return ctx->chr_rom[nes_chr_offset(ctx, address)];
  }
  if (address < 0x3F00U)
  {
    if (address >= 0x3000U)
    {
      address = (uint16_t)(address - 0x1000U);
    }
    return ctx->ppu.nametable[nes_nametable_index(ctx, address)];
  }
  return ctx->ppu.palette[nes_palette_index(address)];
}

static void nes_ppu_write(NES_RuntimeContext *ctx, uint16_t address, uint8_t value)
{
  address &= 0x3FFFU;
  if (address < 0x2000U)
  {
    if (ctx->chr_is_ram != 0U)
    {
      ctx->chr_ram[nes_chr_offset(ctx, address)] = value;
    }
    return;
  }
  if (address < 0x3F00U)
  {
    if (address >= 0x3000U)
    {
      address = (uint16_t)(address - 0x1000U);
    }
    ctx->ppu.nametable[nes_nametable_index(ctx, address)] = value;
    return;
  }
  ctx->ppu.palette[nes_palette_index(address)] = value & 0x3FU;
}

static uint8_t nes_ppu_register_read(NES_RuntimeContext *ctx, uint16_t address)
{
  NES_PPU *p = &ctx->ppu;
  uint8_t result = p->open_bus;
  address = (uint16_t)(0x2000U | (address & 7U));

  switch (address)
  {
    case 0x2002U:
      result = (uint8_t)((p->status & 0xE0U) | (p->open_bus & 0x1FU));
      p->status &= (uint8_t)~0x80U;
      p->write_toggle = 0U;
      break;
    case 0x2004U:
      result = p->oam[p->oam_addr];
      break;
    case 0x2007U:
    {
      uint8_t value = nes_ppu_read(ctx, p->v);
      if ((p->v & 0x3FFFU) >= 0x3F00U)
      {
        result = value;
        p->read_buffer = nes_ppu_read(ctx, (uint16_t)(p->v - 0x1000U));
      }
      else
      {
        result = p->read_buffer;
        p->read_buffer = value;
      }
      p->v = (uint16_t)((p->v + ((p->ctrl & 0x04U) ? 32U : 1U)) & 0x7FFFU);
      break;
    }
    default:
      break;
  }
  p->open_bus = result;
  return result;
}

static void nes_ppu_register_write(NES_RuntimeContext *ctx,
                                   uint16_t address, uint8_t value)
{
  NES_PPU *p = &ctx->ppu;
  uint8_t old_ctrl;
  address = (uint16_t)(0x2000U | (address & 7U));
  p->open_bus = value;

  switch (address)
  {
    case 0x2000U:
      old_ctrl = p->ctrl;
      p->ctrl = value;
      p->t = (uint16_t)((p->t & 0xF3FFU) | (((uint16_t)value & 3U) << 10));
      if (((old_ctrl & 0x80U) == 0U) && ((value & 0x80U) != 0U) &&
          ((p->status & 0x80U) != 0U))
      {
        NES_CPU_RequestNmi(&ctx->cpu);
      }
      break;
    case 0x2001U:
      p->mask = value;
      break;
    case 0x2003U:
      p->oam_addr = value;
      break;
    case 0x2004U:
      p->oam[p->oam_addr++] = value;
      break;
    case 0x2005U:
      if (p->write_toggle == 0U)
      {
        p->fine_x = value & 7U;
        p->t = (uint16_t)((p->t & 0xFFE0U) | (value >> 3));
        p->write_toggle = 1U;
      }
      else
      {
        p->t = (uint16_t)((p->t & 0x8C1FU) |
                          (((uint16_t)value & 0xF8U) << 2) |
                          (((uint16_t)value & 7U) << 12));
        p->write_toggle = 0U;
      }
      break;
    case 0x2006U:
      if (p->write_toggle == 0U)
      {
        p->t = (uint16_t)((p->t & 0x00FFU) | (((uint16_t)value & 0x3FU) << 8));
        p->write_toggle = 1U;
      }
      else
      {
        p->t = (uint16_t)((p->t & 0xFF00U) | value);
        p->v = p->t;
        p->write_toggle = 0U;
      }
      break;
    case 0x2007U:
      nes_ppu_write(ctx, p->v, value);
      p->v = (uint16_t)((p->v + ((p->ctrl & 0x04U) ? 32U : 1U)) & 0x7FFFU);
      break;
    default:
      break;
  }
}

static uint8_t nes_apu_status_read(NES_RuntimeContext *ctx)
{
  uint8_t result = (ctx->apu.frame_irq != 0U) ? 0x40U : 0U;

  /* Audio channels are intentionally not synthesized.  Frame IRQ behavior
   * is still modeled because games use $4015/$4017 for timing. */
  ctx->apu.frame_irq = 0U;
  NES_CPU_ClearIrq(&ctx->cpu);
  return result;
}

static void nes_apu_frame_control_write(NES_RuntimeContext *ctx, uint8_t value)
{
  ctx->apu.mode_5step = ((value & 0x80U) != 0U) ? 1U : 0U;
  ctx->apu.irq_inhibit = ((value & 0x40U) != 0U) ? 1U : 0U;
  ctx->apu.frame_cycle = 0U;
  if (ctx->apu.irq_inhibit != 0U)
  {
    ctx->apu.frame_irq = 0U;
    NES_CPU_ClearIrq(&ctx->cpu);
  }
}

static void nes_apu_advance(NES_RuntimeContext *ctx, uint32_t cpu_cycles)
{
  uint32_t period = (ctx->apu.mode_5step != 0U) ?
                      NES_APU_5STEP_CYCLES : NES_APU_4STEP_CYCLES;

  ctx->apu.frame_cycle += cpu_cycles;
  while (ctx->apu.frame_cycle >= period)
  {
    ctx->apu.frame_cycle -= period;
    if ((ctx->apu.mode_5step == 0U) && (ctx->apu.irq_inhibit == 0U))
    {
      ctx->apu.frame_irq = 1U;
      NES_CPU_RequestIrq(&ctx->cpu);
    }
  }
}

static uint8_t nes_cpu_bus_read(void *user, uint16_t address)
{
  NES_RuntimeContext *ctx = (NES_RuntimeContext *)user;
  if (address < 0x2000U)
  {
    return ctx->cpu_ram[address & 0x07FFU];
  }
  if (address < 0x4000U)
  {
    return nes_ppu_register_read(ctx, address);
  }
  if (address == 0x4015U)
  {
    return nes_apu_status_read(ctx);
  }
  if (address == 0x4016U)
  {
    uint8_t result;
    if (ctx->controller_strobe != 0U)
    {
      result = nes_controller_sample() & 1U;
    }
    else
    {
      result = ctx->controller_shift & 1U;
      ctx->controller_shift = (uint8_t)((ctx->controller_shift >> 1) | 0x80U);
    }
    return (uint8_t)(0x40U | result);
  }
  if ((address >= 0x6000U) && (address < 0x8000U))
  {
    if ((ctx->metadata.mapper == 1U) && (ctx->prg_ram_enabled == 0U))
    {
      return 0xFFU;
    }
    return ctx->prg_ram[address - 0x6000U];
  }
  if (address >= 0x8000U)
  {
    uint32_t offset;
    if (ctx->metadata.mapper == 1U)
    {
      offset = nes_mmc1_prg_offset(ctx, address);
    }
    else if (ctx->metadata.mapper == 2U)
    {
      if (address < 0xC000U)
      {
        offset = ((uint32_t)ctx->prg_bank * NES_PRG_BANK_BYTES) +
                 (address - 0x8000U);
      }
      else
      {
        offset = ((uint32_t)(ctx->prg_bank_count - 1U) * NES_PRG_BANK_BYTES) +
                 (address - 0xC000U);
      }
    }
    else
    {
      offset = address - 0x8000U;
      if (ctx->metadata.prg_size == NES_PRG_BANK_BYTES)
      {
        offset &= (NES_PRG_BANK_BYTES - 1U);
      }
    }
    return ctx->prg_rom[offset];
  }
  return 0U;
}

static void nes_cpu_bus_write(void *user, uint16_t address, uint8_t value)
{
  NES_RuntimeContext *ctx = (NES_RuntimeContext *)user;
  if (address < 0x2000U)
  {
    ctx->cpu_ram[address & 0x07FFU] = value;
    return;
  }
  if (address == 0x4015U)
  {
    ctx->apu.channel_enable = value & 0x1FU;
    return;
  }
  if (address == 0x4017U)
  {
    nes_apu_frame_control_write(ctx, value);
    return;
  }
  if (address < 0x4000U)
  {
    nes_ppu_register_write(ctx, address, value);
    return;
  }
  if (address == 0x4014U)
  {
    uint16_t source = (uint16_t)value << 8;
    uint16_t i;
    for (i = 0U; i < 256U; i++)
    {
      ctx->ppu.oam[ctx->ppu.oam_addr++] = nes_cpu_bus_read(ctx, (uint16_t)(source + i));
    }
    ctx->cpu_stall_cycles = (uint16_t)(ctx->cpu_stall_cycles + 513U);
    return;
  }
  if (address == 0x4016U)
  {
    uint8_t new_strobe = value & 1U;
    if ((ctx->controller_strobe != 0U) || (new_strobe != 0U))
    {
      ctx->controller_latch = nes_controller_sample();
      ctx->controller_shift = ctx->controller_latch;
    }
    ctx->controller_strobe = new_strobe;
    return;
  }
  if ((address >= 0x6000U) && (address < 0x8000U))
  {
    if ((ctx->metadata.mapper != 1U) || (ctx->prg_ram_enabled != 0U))
    {
      uint16_t offset = address - 0x6000U;
      if (ctx->prg_ram[offset] != value)
      {
        ctx->prg_ram[offset] = value;
        if (ctx->battery_backed != 0U)
        {
          ctx->prg_ram_dirty = 1U;
        }
      }
    }
    return;
  }
  if (address >= 0x8000U)
  {
    if (ctx->metadata.mapper == 1U)
    {
      nes_mmc1_write(ctx, address, value);
    }
    else if ((ctx->metadata.mapper == 2U) && (ctx->prg_bank_count != 0U))
    {
      ctx->prg_bank = (uint8_t)((uint16_t)value % ctx->prg_bank_count);
    }
    else if ((ctx->metadata.mapper == 3U) && (ctx->chr_bank_count != 0U))
    {
      ctx->chr_bank = (uint8_t)((uint16_t)value % ctx->chr_bank_count);
    }
  }
  /* APU registers and Mapper-0 ROM writes are intentionally ignored. */
}

static void nes_run_cpu_scanline(NES_RuntimeContext *ctx)
{
  int32_t budget;
  int32_t used;
  uint16_t elapsed;

  ctx->scanline_cycle_remainder = (uint16_t)(ctx->scanline_cycle_remainder + 341U);
  budget = (int32_t)(ctx->scanline_cycle_remainder / 3U);
  ctx->scanline_cycle_remainder %= 3U;
  elapsed = (uint16_t)budget;

  if (ctx->cpu_stall_cycles != 0U)
  {
    uint16_t consumed = (ctx->cpu_stall_cycles > (uint16_t)budget) ?
                        (uint16_t)budget : ctx->cpu_stall_cycles;
    ctx->cpu_stall_cycles = (uint16_t)(ctx->cpu_stall_cycles - consumed);
    budget -= consumed;
  }
  if (ctx->cpu_cycle_debt >= (uint16_t)budget)
  {
    ctx->cpu_cycle_debt = (uint16_t)(ctx->cpu_cycle_debt - (uint16_t)budget);
    nes_apu_advance(ctx, elapsed);
    return;
  }
  budget -= ctx->cpu_cycle_debt;
  ctx->cpu_cycle_debt = 0U;
  if (budget <= 0)
  {
    nes_apu_advance(ctx, elapsed);
    return;
  }
  used = NES_CPU_Run(&ctx->cpu, budget);
  if (used > budget)
  {
    ctx->cpu_cycle_debt = (uint16_t)(used - budget);
  }
  nes_apu_advance(ctx, elapsed);
}

static uint16_t nes_color(NES_RuntimeContext *ctx, uint16_t palette_address)
{
  uint8_t index = nes_ppu_read(ctx, palette_address) & 0x3FU;
  return s_nes_palette[index];
}

static void nes_render_background(NES_RuntimeContext *ctx)
{
  NES_PPU *p = &ctx->ppu;
  uint16_t universal = nes_color(ctx, 0x3F00U);
  uint16_t x;

  memset(ctx->bg_opaque, 0, sizeof(ctx->bg_opaque));
  for (x = 0U; x < NES_FRAME_WIDTH; x++)
  {
    ctx->line_pixels[x] = universal;
  }
  if ((p->mask & 0x08U) == 0U)
  {
    return;
  }

  for (x = 0U; x < NES_FRAME_WIDTH; x++)
  {
    uint16_t total_x;
    uint8_t coarse_x;
    uint8_t coarse_y;
    uint8_t fine_y;
    uint8_t nt_x;
    uint8_t nt_y;
    uint16_t nt_address;
    uint8_t tile;
    uint8_t attr;
    uint8_t shift;
    uint16_t pattern;
    uint8_t bit;
    uint8_t pixel;
    uint8_t palette_select;

    if ((x < 8U) && ((p->mask & 0x02U) == 0U))
    {
      continue;
    }
    total_x = (uint16_t)(((p->v & 0x001FU) << 3) + p->fine_x + x);
    coarse_x = (uint8_t)((total_x >> 3) & 31U);
    coarse_y = (uint8_t)((p->v >> 5) & 31U);
    fine_y = (uint8_t)((p->v >> 12) & 7U);
    nt_x = (uint8_t)(((p->v >> 10) & 1U) ^ ((total_x >> 8) & 1U));
    nt_y = (uint8_t)((p->v >> 11) & 1U);
    nt_address = (uint16_t)(0x2000U | ((uint16_t)nt_y << 11) |
                            ((uint16_t)nt_x << 10) |
                            ((uint16_t)coarse_y << 5) | coarse_x);
    tile = nes_ppu_read(ctx, nt_address);
    attr = nes_ppu_read(ctx, (uint16_t)(0x23C0U |
                        ((uint16_t)nt_y << 11) | ((uint16_t)nt_x << 10) |
                        (((uint16_t)coarse_y >> 2) << 3) | (coarse_x >> 2)));
    shift = (uint8_t)(((coarse_y & 2U) << 1) | (coarse_x & 2U));
    palette_select = (uint8_t)((attr >> shift) & 3U);
    pattern = (uint16_t)(((p->ctrl & 0x10U) ? 0x1000U : 0U) +
                         ((uint16_t)tile << 4) + fine_y);
    bit = (uint8_t)(7U - (total_x & 7U));
    pixel = (uint8_t)(((nes_ppu_read(ctx, pattern) >> bit) & 1U) |
                      (((nes_ppu_read(ctx, (uint16_t)(pattern + 8U)) >> bit) & 1U) << 1));
    if (pixel != 0U)
    {
      ctx->bg_opaque[x] = 1U;
      ctx->line_pixels[x] = nes_color(ctx, (uint16_t)(0x3F00U +
                                  ((uint16_t)palette_select << 2) + pixel));
    }
  }
}

static void nes_render_sprites(NES_RuntimeContext *ctx, uint16_t line)
{
  NES_PPU *p = &ctx->ppu;
  uint8_t selected[8];
  uint8_t count = 0U;
  uint8_t sprite;
  uint8_t height = (p->ctrl & 0x20U) ? 16U : 8U;

  memset(ctx->sprite_occupied, 0, sizeof(ctx->sprite_occupied));
  if ((p->mask & 0x10U) == 0U)
  {
    return;
  }
  for (sprite = 0U; sprite < 64U; sprite++)
  {
    int16_t row = (int16_t)line - ((int16_t)p->oam[sprite * 4U] + 1);
    if ((row >= 0) && (row < height))
    {
      if (count < 8U)
      {
        selected[count++] = sprite;
      }
      else
      {
        p->status |= 0x20U;
        break;
      }
    }
  }

  for (uint8_t n = 0U; n < count; n++)
  {
    uint8_t index = selected[n];
    uint8_t *oam = &p->oam[index * 4U];
    uint8_t tile = oam[1];
    uint8_t attr = oam[2];
    uint8_t sx = oam[3];
    uint8_t row = (uint8_t)((int16_t)line - ((int16_t)oam[0] + 1));
    uint16_t pattern;

    if ((attr & 0x80U) != 0U)
    {
      row = (uint8_t)(height - 1U - row);
    }
    if (height == 16U)
    {
      pattern = (uint16_t)(((tile & 1U) << 12) |
                          ((uint16_t)(tile & 0xFEU) << 4));
      if (row >= 8U)
      {
        pattern = (uint16_t)(pattern + 16U);
        row -= 8U;
      }
      pattern = (uint16_t)(pattern + row);
    }
    else
    {
      pattern = (uint16_t)(((p->ctrl & 0x08U) ? 0x1000U : 0U) +
                           ((uint16_t)tile << 4) + row);
    }

    for (uint8_t column = 0U; column < 8U; column++)
    {
      uint16_t screen_x = (uint16_t)sx + column;
      uint8_t bit;
      uint8_t pixel;

      if (screen_x >= NES_FRAME_WIDTH)
      {
        continue;
      }
      if ((screen_x < 8U) && ((p->mask & 0x04U) == 0U))
      {
        continue;
      }
      bit = ((attr & 0x40U) != 0U) ? column : (uint8_t)(7U - column);
      pixel = (uint8_t)(((nes_ppu_read(ctx, pattern) >> bit) & 1U) |
                        (((nes_ppu_read(ctx, (uint16_t)(pattern + 8U)) >> bit) & 1U) << 1));
      if (pixel == 0U)
      {
        continue;
      }
      if ((index == 0U) && (ctx->bg_opaque[screen_x] != 0U) &&
          (screen_x < 255U))
      {
        p->status |= 0x40U;
      }
      if (ctx->sprite_occupied[screen_x] != 0U)
      {
        continue;
      }
      ctx->sprite_occupied[screen_x] = 1U;
      if (((attr & 0x20U) != 0U) && (ctx->bg_opaque[screen_x] != 0U))
      {
        continue;
      }
      ctx->line_pixels[screen_x] = nes_color(ctx, (uint16_t)(0x3F10U +
                                  ((uint16_t)(attr & 3U) << 2) + pixel));
    }
  }
}

static void nes_increment_scroll_y(NES_PPU *p)
{
  uint16_t coarse_y;
  if ((p->v & 0x7000U) != 0x7000U)
  {
    p->v += 0x1000U;
    return;
  }
  p->v &= (uint16_t)~0x7000U;
  coarse_y = (p->v & 0x03E0U) >> 5;
  if (coarse_y == 29U)
  {
    coarse_y = 0U;
    p->v ^= 0x0800U;
  }
  else if (coarse_y == 31U)
  {
    coarse_y = 0U;
  }
  else
  {
    coarse_y++;
  }
  p->v = (uint16_t)((p->v & (uint16_t)~0x03E0U) | (coarse_y << 5));
}

static void nes_render_scanline(NES_RuntimeContext *ctx, uint16_t line,
                                uint8_t write_framebuffer)
{
  NES_PPU *p = &ctx->ppu;

  if ((p->mask & 0x18U) != 0U)
  {
    p->v = (uint16_t)((p->v & 0xFBE0U) | (p->t & 0x041FU));
  }
  nes_render_background(ctx);
  nes_render_sprites(ctx, line);
  if (write_framebuffer != 0U)
  {
    memcpy(&ctx->framebuffer[line * NES_LCD_WIDTH],
           &ctx->line_pixels[NES_LCD_CROP_X], NES_LCD_WIDTH * sizeof(uint16_t));
  }
  if ((p->mask & 0x18U) != 0U)
  {
    nes_increment_scroll_y(p);
  }
}

static void nes_emulate_frame(NES_RuntimeContext *ctx, uint8_t draw)
{
  uint16_t line;
  NES_PPU *p = &ctx->ppu;

  /* Approximate the pre-render scanline, including vertical scroll reload. */
  ctx->current_scanline = 261U;
  p->status &= (uint8_t)~0xE0U;
  if ((p->mask & 0x18U) != 0U)
  {
    p->v = p->t;
  }
  nes_run_cpu_scanline(ctx);

  for (line = 0U; line < NES_FRAME_HEIGHT; line++)
  {
    ctx->current_scanline = line;
    nes_run_cpu_scanline(ctx);
    nes_render_scanline(ctx, line, draw);
  }

  /* One post-render line, then twenty NTSC vblank scanlines. */
  ctx->current_scanline = 240U;
  nes_run_cpu_scanline(ctx);
  p->status |= 0x80U;
  if ((p->ctrl & 0x80U) != 0U)
  {
    NES_CPU_RequestNmi(&ctx->cpu);
  }
  for (line = 0U; line < 20U; line++)
  {
    ctx->current_scanline = (uint16_t)(241U + line);
    nes_run_cpu_scanline(ctx);
  }
}

static void nes_advance_frame_deadline(NES_RuntimeContext *ctx, uint32_t now)
{
  /* 16, 17, 17 ms gives exactly 50 ms for three NTSC-like frames. */
  static const uint8_t increments[3] = {16U, 17U, 17U};
  ctx->next_frame_tick += increments[ctx->frame_tick_phase];
  ctx->frame_tick_phase++;
  if (ctx->frame_tick_phase >= 3U)
  {
    ctx->frame_tick_phase = 0U;
  }
  if ((int32_t)(now - ctx->next_frame_tick) > (int32_t)NES_FRAME_LATE_RESET_MS)
  {
    ctx->next_frame_tick = now;
  }
}

static uint8_t nes_rom_layout_supported(const NES_RomCacheMetadata *metadata)
{
  if (metadata == NULL)
  {
    return 0U;
  }
  switch (metadata->mapper)
  {
    case 0U: /* NROM */
      return (((metadata->prg_size == NES_PRG_BANK_BYTES) ||
               (metadata->prg_size == (2U * NES_PRG_BANK_BYTES))) &&
              ((metadata->chr_size == 0U) ||
               (metadata->chr_size == NES_CHR_BANK_BYTES))) ? 1U : 0U;
    case 2U: /* UxROM: switchable 16 KiB + fixed final 16 KiB */
      return (((metadata->prg_size % NES_PRG_BANK_BYTES) == 0U) &&
              (metadata->prg_size >= (2U * NES_PRG_BANK_BYTES)) &&
              ((metadata->chr_size == 0U) ||
               (metadata->chr_size == NES_CHR_BANK_BYTES))) ? 1U : 0U;
    case 1U: /* MMC1: 16/32 KiB PRG modes and 4/8 KiB CHR modes */
      return (((metadata->prg_size % NES_PRG_BANK_BYTES) == 0U) &&
              (metadata->prg_size >= (2U * NES_PRG_BANK_BYTES)) &&
              (metadata->prg_size <= NES_MMC1_MAX_PRG_BYTES) &&
              ((metadata->chr_size == 0U) ||
               ((metadata->chr_size % NES_CHR_HALF_BANK_BYTES) == 0U))) ?
               1U : 0U;
    case 3U: /* CNROM: fixed NROM PRG + switchable 8 KiB CHR */
      return (((metadata->prg_size == NES_PRG_BANK_BYTES) ||
               (metadata->prg_size == (2U * NES_PRG_BANK_BYTES))) &&
              (metadata->chr_size >= NES_CHR_BANK_BYTES) &&
              ((metadata->chr_size % NES_CHR_BANK_BYTES) == 0U)) ? 1U : 0U;
    default:
      return 0U;
  }
}

static void nes_soft_reset(NES_RuntimeContext *ctx, uint32_t now)
{
  NES_PPU *p = &ctx->ppu;

  ctx->prg_bank = 0U;
  ctx->chr_bank = 0U;
  memset(&ctx->apu, 0, sizeof(ctx->apu));
  if (ctx->metadata.mapper == 1U)
  {
    nes_mmc1_reset(ctx);
  }
  ctx->scanline_cycle_remainder = 0U;
  ctx->cpu_cycle_debt = 0U;
  ctx->cpu_stall_cycles = 0U;
  ctx->controller_strobe = 0U;
  ctx->controller_latch = nes_controller_sample();
  ctx->controller_shift = ctx->controller_latch;
  ctx->frame_tick_phase = 0U;
  ctx->select_pulse_frames = 0U;
  ctx->force_draw = 1U;
  ctx->next_frame_tick = now;

  p->ctrl = 0U;
  p->mask = 0U;
  p->status = 0U;
  p->oam_addr = 0U;
  p->v = 0U;
  p->t = 0U;
  p->fine_x = 0U;
  p->write_toggle = 0U;
  p->read_buffer = 0U;
  p->open_bus = 0U;
  NES_CPU_Reset(&ctx->cpu);
}

static void nes_process_special_inputs(NES_RuntimeContext *ctx, uint32_t now)
{
  uint8_t ok = nes_key_pressed(Key_OK_GPIO_Port, Key_OK_Pin);
  uint8_t key1 = nes_key_pressed(Key1_GPIO_Port, Key1_Pin);

  if ((ok != 0U) && (key1 != 0U))
  {
    if (ctx->start_combo_active == 0U)
    {
      ctx->start_combo_active = 1U;
      ctx->start_combo_reset = 0U;
      ctx->start_combo_tick = now;
      ctx->key1_click_armed = 0U;
    }
    else if ((ctx->start_combo_reset == 0U) &&
             ((now - ctx->start_combo_tick) >= NES_SOFT_RESET_HOLD_MS))
    {
      ctx->start_combo_reset = 1U;
      nes_soft_reset(ctx, now);
    }
  }
  else
  {
    ctx->start_combo_active = 0U;
    ctx->start_combo_reset = 0U;
  }

  if ((key1 != 0U) && (ctx->key1_last_pressed == 0U) &&
      (ok == 0U) && (ctx->start_combo_active == 0U))
  {
    if ((ctx->key1_click_armed != 0U) &&
        ((now - ctx->key1_last_press_tick) <= NES_SELECT_DOUBLE_CLICK_MS))
    {
      ctx->select_pulse_frames = NES_SELECT_PULSE_FRAMES;
      ctx->key1_click_armed = 0U;
    }
    else
    {
      ctx->key1_last_press_tick = now;
      ctx->key1_click_armed = 1U;
    }
  }
  ctx->key1_last_pressed = key1;
}

static uint8_t nes_build_save_paths(const char *rom_path)
{
  const char *slash;
  const char *dot;
  size_t path_length;
  size_t base_length;

  s_save_path[0] = '\0';
  s_save_temp_path[0] = '\0';
  s_save_backup_path[0] = '\0';
  if (rom_path == NULL)
  {
    return 0U;
  }

  path_length = strlen(rom_path);
  slash = strrchr(rom_path, '/');
  dot = strrchr(rom_path, '.');
  if ((dot == NULL) || ((slash != NULL) && (dot < slash)))
  {
    base_length = path_length;
  }
  else
  {
    base_length = (size_t)(dot - rom_path);
  }

  if ((base_length + sizeof(".sav.tmp")) > NES_SAVE_PATH_SIZE)
  {
    return 0U;
  }

  memcpy(s_save_path, rom_path, base_length);
  memcpy(&s_save_path[base_length], ".sav", sizeof(".sav"));
  memcpy(s_save_temp_path, rom_path, base_length);
  memcpy(&s_save_temp_path[base_length], ".sav.tmp", sizeof(".sav.tmp"));
  memcpy(s_save_backup_path, rom_path, base_length);
  memcpy(&s_save_backup_path[base_length], ".sav.bak", sizeof(".sav.bak"));
  return 1U;
}

static void nes_save_unmount(void)
{
  SD_SetReadPollingMode(0U);
  (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
}

static FRESULT nes_open_valid_save(FIL *file)
{
  const char *candidates[3] =
  {
    s_save_path,
    s_save_backup_path,
    s_save_temp_path
  };
  FRESULT fr;
  uint8_t invalid_image = 0U;
  uint8_t i;

  for (i = 0U; i < 3U; ++i)
  {
    fr = f_open(file, candidates[i], FA_READ);
    if (fr == FR_OK)
    {
      if ((uint32_t)f_size(file) == NES_PRG_RAM_BYTES)
      {
        return FR_OK;
      }
      invalid_image = 1U;
      (void)f_close(file);
      continue;
    }
    if (fr != FR_NO_FILE)
    {
      return fr;
    }
  }

  return (invalid_image != 0U) ? FR_INT_ERR : FR_NO_FILE;
}

static int8_t nes_load_battery_save(NES_RuntimeContext *ctx)
{
  FIL file;
  FRESULT fr;
  UINT bytes_read;
  uint32_t offset = 0U;

  if ((ctx->battery_backed == 0U) || (s_save_path[0] == '\0'))
  {
    return NES_RUNTIME_SAVE_NONE;
  }

  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    return NES_RUNTIME_ERR_SAVE;
  }

  SD_SetReadPollingMode(1U);
  /* A reset between atomic rename steps can leave the valid image under its
   * .bak or fully-synced .tmp name. */
  fr = nes_open_valid_save(&file);
  if (fr == FR_NO_FILE)
  {
    nes_save_unmount();
    return NES_RUNTIME_SAVE_NONE;
  }
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    nes_save_unmount();
    return NES_RUNTIME_ERR_SAVE;
  }

  while (offset < NES_PRG_RAM_BYTES)
  {
    uint32_t chunk = NES_PRG_RAM_BYTES - offset;
    if (chunk > sizeof(s_save_io_buffer))
    {
      chunk = sizeof(s_save_io_buffer);
    }
    bytes_read = 0U;
    fr = f_read(&file, s_save_io_buffer, (UINT)chunk, &bytes_read);
    if ((fr != FR_OK) || (bytes_read != chunk))
    {
      ctx->last_fs_error = (uint8_t)((fr != FR_OK) ? fr : FR_DISK_ERR);
      (void)f_close(&file);
      nes_save_unmount();
      return NES_RUNTIME_ERR_SAVE;
    }
    memcpy(&ctx->prg_ram[offset], s_save_io_buffer, chunk);
    offset += chunk;
  }

  fr = f_close(&file);
  nes_save_unmount();
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    return NES_RUNTIME_ERR_SAVE;
  }

  ctx->save_loaded = 1U;
  ctx->prg_ram_dirty = 0U;
  ctx->last_fs_error = 0U;
  return NES_RUNTIME_SAVE_LOADED;
}

static int8_t nes_write_battery_save(NES_RuntimeContext *ctx)
{
  FIL file;
  FILINFO file_info;
  FRESULT fr;
  FRESULT close_result;
  UINT bytes_written;
  uint32_t offset = 0U;
  uint8_t backup_created = 0U;

  if ((ctx->battery_backed == 0U) || (ctx->prg_ram_dirty == 0U) ||
      (s_save_path[0] == '\0'))
  {
    return NES_RUNTIME_SAVE_NONE;
  }

  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    return NES_RUNTIME_ERR_SAVE;
  }

  fr = f_open(&file, s_save_temp_path, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    nes_save_unmount();
    return NES_RUNTIME_ERR_SAVE;
  }

  while (offset < NES_PRG_RAM_BYTES)
  {
    uint32_t chunk = NES_PRG_RAM_BYTES - offset;
    if (chunk > sizeof(s_save_io_buffer))
    {
      chunk = sizeof(s_save_io_buffer);
    }
    memcpy(s_save_io_buffer, &ctx->prg_ram[offset], chunk);
    bytes_written = 0U;
    fr = f_write(&file, s_save_io_buffer, (UINT)chunk, &bytes_written);
    if ((fr != FR_OK) || (bytes_written != chunk))
    {
      fr = (fr != FR_OK) ? fr : FR_DISK_ERR;
      break;
    }
    offset += chunk;
  }
  if (fr == FR_OK)
  {
    fr = f_sync(&file);
  }
  close_result = f_close(&file);
  if ((fr == FR_OK) && (close_result != FR_OK))
  {
    fr = close_result;
  }
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    (void)f_unlink(s_save_temp_path);
    nes_save_unmount();
    return NES_RUNTIME_ERR_SAVE;
  }

  (void)memset(&file_info, 0, sizeof(file_info));
  fr = f_stat(s_save_path, &file_info);
  if (fr == FR_OK)
  {
    fr = f_unlink(s_save_backup_path);
    if ((fr != FR_OK) && (fr != FR_NO_FILE))
    {
      ctx->last_fs_error = (uint8_t)fr;
      (void)f_unlink(s_save_temp_path);
      nes_save_unmount();
      return NES_RUNTIME_ERR_SAVE;
    }
    fr = f_rename(s_save_path, s_save_backup_path);
    if (fr != FR_OK)
    {
      ctx->last_fs_error = (uint8_t)fr;
      (void)f_unlink(s_save_temp_path);
      nes_save_unmount();
      return NES_RUNTIME_ERR_SAVE;
    }
    backup_created = 1U;
  }
  else if (fr != FR_NO_FILE)
  {
    ctx->last_fs_error = (uint8_t)fr;
    (void)f_unlink(s_save_temp_path);
    nes_save_unmount();
    return NES_RUNTIME_ERR_SAVE;
  }

  fr = f_rename(s_save_temp_path, s_save_path);
  if (fr != FR_OK)
  {
    ctx->last_fs_error = (uint8_t)fr;
    if (backup_created != 0U)
    {
      (void)f_rename(s_save_backup_path, s_save_path);
    }
    (void)f_unlink(s_save_temp_path);
    nes_save_unmount();
    return NES_RUNTIME_ERR_SAVE;
  }

  if (backup_created != 0U)
  {
    (void)f_unlink(s_save_backup_path);
  }
  nes_save_unmount();
  ctx->prg_ram_dirty = 0U;
  ctx->last_fs_error = 0U;
  return NES_RUNTIME_SAVE_WRITTEN;
}

int8_t NES_Runtime_Start(void)
{
  NES_RomCacheMetadata metadata;
  const uint8_t *rom = NULL;
  const uint8_t *prg;
  const uint8_t *chr;
  uint32_t trainer_size;
  uint32_t minimum_size;
  uint32_t capacity = 0U;
  uint16_t *framebuffer;

  if (s_nes_active != 0U)
  {
    return NES_RUNTIME_RUNNING;
  }
  NES_Runtime_ClearRemoteButtons();
  s_nes_remote_reset_pending = 0U;
  if (NES_RomCache_Map(&rom, &metadata) != NES_ROM_CACHE_OK)
  {
    return NES_RUNTIME_ERR_CACHE;
  }
  if ((metadata.mapper != 0U) && (metadata.mapper != 1U) &&
      (metadata.mapper != 2U) &&
      (metadata.mapper != 3U))
  {
    (void)NES_RomCache_Unmap();
    return NES_RUNTIME_ERR_MAPPER;
  }
  if (nes_rom_layout_supported(&metadata) == 0U)
  {
    (void)NES_RomCache_Unmap();
    return NES_RUNTIME_ERR_ROM;
  }
  trainer_size = ((metadata.flags6 & 0x04U) != 0U) ? 512U : 0U;
  minimum_size = 16U + trainer_size + metadata.prg_size + metadata.chr_size;
  if ((rom == NULL) || (metadata.rom_size < minimum_size))
  {
    (void)NES_RomCache_Unmap();
    return NES_RUNTIME_ERR_ROM;
  }
  prg = rom + 16U + trainer_size;
  chr = prg + metadata.prg_size;

  framebuffer = (uint16_t *)MediaMemory_Acquire(
      MEDIA_MEMORY_OWNER_NES, NES_FRAMEBUFFER_BYTES, &capacity);
  if ((framebuffer == NULL) || (capacity < NES_FRAMEBUFFER_BYTES))
  {
    (void)NES_RomCache_Unmap();
    return NES_RUNTIME_ERR_MEMORY;
  }

  memset(&s_nes, 0, sizeof(s_nes));
  s_nes.metadata = metadata;
  s_nes.mapped_rom = rom;
  s_nes.prg_rom = prg;
  s_nes.chr_rom = chr;
  s_nes.framebuffer = framebuffer;
  s_nes.framebuffer_capacity = capacity;
  s_nes.prg_bank_count = (uint16_t)(metadata.prg_size / NES_PRG_BANK_BYTES);
  s_nes.chr_bank_count = (metadata.chr_size == 0U) ? 1U :
                         (uint16_t)(metadata.chr_size / NES_CHR_BANK_BYTES);
  s_nes.chr_half_bank_count = (metadata.chr_size == 0U) ? 2U :
                         (uint16_t)(metadata.chr_size / NES_CHR_HALF_BANK_BYTES);
  s_nes.chr_is_ram = (metadata.chr_size == 0U) ? 1U : 0U;
  s_nes.mirroring_mode = ((metadata.flags6 & 0x08U) != 0U) ?
                           NES_MIRROR_FOUR_SCREEN :
                           (((metadata.flags6 & 0x01U) != 0U) ?
                              NES_MIRROR_VERTICAL : NES_MIRROR_HORIZONTAL);
  s_nes.prg_ram_enabled = 1U;
  s_nes.battery_backed = ((metadata.flags6 & 0x02U) != 0U) ? 1U : 0U;
  if (metadata.mapper == 1U)
  {
    nes_mmc1_reset(&s_nes);
  }
  if (trainer_size != 0U)
  {
    memcpy(&s_nes.prg_ram[0x1000U], rom + 16U, trainer_size);
  }
  if (s_nes.battery_backed != 0U)
  {
    if (nes_build_save_paths(NES_RomCache_GetPath()) != 0U)
    {
      (void)nes_load_battery_save(&s_nes);
    }
    else
    {
      s_nes.last_fs_error = (uint8_t)FR_INVALID_NAME;
    }
  }
  else
  {
    (void)nes_build_save_paths(NULL);
  }
  memset(framebuffer, 0, NES_FRAMEBUFFER_BYTES);

  NES_CPU_Init(&s_nes.cpu, nes_cpu_bus_read, nes_cpu_bus_write, &s_nes);
  NES_CPU_Reset(&s_nes.cpu);
  if ((s_nes.cpu.pc < 0x8000U) || (s_nes.cpu.jammed != 0U))
  {
    MediaMemory_Release(MEDIA_MEMORY_OWNER_NES);
    (void)NES_RomCache_Unmap();
    memset(&s_nes, 0, sizeof(s_nes));
    return NES_RUNTIME_ERR_CPU;
  }

  s_nes.controller_latch = nes_controller_sample();
  s_nes.controller_shift = s_nes.controller_latch;
  s_nes.exit_armed = ((nes_key_pressed(Key2_GPIO_Port, Key2_Pin) == 0U) &&
                      (nes_key_pressed(Key3_GPIO_Port, Key3_Pin) == 0U)) ? 1U : 0U;
  s_nes.next_frame_tick = HAL_GetTick();
  __DMB();
  s_nes_active = 1U;
  return NES_RUNTIME_RUNNING;
}

int8_t NES_Runtime_Process(void)
{
  uint32_t now;
  uint8_t draw;

  if (s_nes_active == 0U)
  {
    return NES_RUNTIME_ERR_CACHE;
  }
  LCD_TransferService();

  if (s_nes.exit_armed == 0U)
  {
    if ((nes_key_pressed(Key2_GPIO_Port, Key2_Pin) == 0U) &&
        (nes_key_pressed(Key3_GPIO_Port, Key3_Pin) == 0U))
    {
      s_nes.exit_armed = 1U;
    }
  }
  else
  {
    if (nes_key_pressed(Key3_GPIO_Port, Key3_Pin) != 0U)
    {
      return NES_RUNTIME_STOP_KEY3;
    }
    if (nes_key_pressed(Key2_GPIO_Port, Key2_Pin) != 0U)
    {
      return NES_RUNTIME_STOP_KEY2;
    }
  }

  now = HAL_GetTick();
  if (s_nes_remote_reset_pending != 0U)
  {
    s_nes_remote_reset_pending = 0U;
    __DMB();
    nes_soft_reset(&s_nes, now);
  }
  nes_process_special_inputs(&s_nes, now);
  if ((int32_t)(now - s_nes.next_frame_tick) < 0)
  {
    return NES_RUNTIME_RUNNING;
  }
  draw = ((s_nes.force_draw != 0U) ||
          ((s_nes.frame_count & 1U) == 0U)) ? 1U : 0U;
  if ((draw != 0U) && (LCD_IsTransmitBusy() != 0U))
  {
    return NES_RUNTIME_RUNNING;
  }

  nes_emulate_frame(&s_nes, draw);
  if (s_nes.cpu.jammed != 0U)
  {
    return NES_RUNTIME_ERR_CPU;
  }
  s_nes.frame_count++;
  if (s_nes.select_pulse_frames != 0U)
  {
    s_nes.select_pulse_frames--;
  }
  if (draw != 0U)
  {
    if (LCD_CopyBufferAsync(0U, 0U, NES_LCD_WIDTH, NES_LCD_HEIGHT,
                            s_nes.framebuffer) != HAL_OK)
    {
      return NES_RUNTIME_ERR_DISPLAY;
    }
    s_nes.rendered_frame_count++;
    s_nes.force_draw = 0U;
  }
  nes_advance_frame_deadline(&s_nes, now);
  return NES_RUNTIME_RUNNING;
}

int8_t NES_Runtime_Stop(uint8_t persist_battery_save)
{
  int8_t save_result = NES_RUNTIME_SAVE_NONE;

  NES_Runtime_ClearRemoteButtons();
  s_nes_remote_reset_pending = 0U;
  if (s_nes_active == 0U)
  {
    return NES_RUNTIME_SAVE_NONE;
  }
  if (LCD_IsTransmitBusy() != 0U)
  {
    if (LCD_WaitTransmitDone(NES_LCD_STOP_TIMEOUT_MS) != HAL_OK)
    {
      LCD_ResetTransferState();
    }
  }
  if (persist_battery_save != 0U)
  {
    save_result = nes_write_battery_save(&s_nes);
  }
  s_nes_active = 0U;
  __DMB();
  MediaMemory_Release(MEDIA_MEMORY_OWNER_NES);
  (void)NES_RomCache_Unmap();
  s_nes.framebuffer = NULL;
  s_nes.mapped_rom = NULL;
  s_nes.prg_rom = NULL;
  s_nes.chr_rom = NULL;
  return save_result;
}

uint8_t NES_Runtime_IsActive(void)
{
  return s_nes_active;
}

uint32_t NES_Runtime_GetFrameCount(void)
{
  return s_nes.frame_count;
}

uint32_t NES_Runtime_GetRenderedFrameCount(void)
{
  return s_nes.rendered_frame_count;
}

void NES_Runtime_GetDiagnostics(NES_RuntimeDiagnostics *diagnostics)
{
  if (diagnostics == NULL)
  {
    return;
  }

  diagnostics->frame_count = s_nes.frame_count;
  diagnostics->cpu_cycles = s_nes.cpu.total_cycles;
  diagnostics->pc = s_nes.cpu.last_pc;
  diagnostics->scanline = s_nes.current_scanline;
  diagnostics->opcode = s_nes.cpu.last_opcode;
  diagnostics->mapper = (uint8_t)s_nes.metadata.mapper;
  diagnostics->prg_bank = (s_nes.metadata.mapper == 1U) ?
                            s_nes.mmc1_prg : s_nes.prg_bank;
  diagnostics->chr_bank = (s_nes.metadata.mapper == 1U) ?
                            s_nes.mmc1_chr0 : s_nes.chr_bank;
  diagnostics->mmc1_control = s_nes.mmc1_control;
  diagnostics->mmc1_prg = s_nes.mmc1_prg;
  diagnostics->save_loaded = s_nes.save_loaded;
  diagnostics->save_dirty = s_nes.prg_ram_dirty;
  diagnostics->last_fs_error = s_nes.last_fs_error;
}
