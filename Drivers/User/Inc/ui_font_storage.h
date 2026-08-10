#ifndef UI_FONT_STORAGE_H
#define UI_FONT_STORAGE_H

#include "lvgl.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_FONT_STORAGE_OK                 0
#define UI_FONT_STORAGE_ERR_NOT_READY     -1
#define UI_FONT_STORAGE_ERR_QSPI          -2
#define UI_FONT_STORAGE_ERR_SD            -3
#define UI_FONT_STORAGE_ERR_PACKAGE       -4
#define UI_FONT_STORAGE_ERR_CRC           -5

/* Check W25Q64 first; if the font is absent, install /GB2312.FNT from SD. */
int8_t UI_FontStorage_Bootstrap(void);
int8_t UI_FontStorage_Init(void);
int8_t UI_FontStorage_InstallFromSd(const char *path);
uint8_t UI_FontStorage_IsReady(void);

/* Keep normal UI reads in the cached QSPI window.  Calls are cheap while the
 * window is already mapped and internally throttled after a mapping error.
 * NES cache writes may temporarily leave QSPI in indirect mode; in that case
 * the glyph callback remains functional through QSPI_W25Qxx_ReadBuffer(). */
void UI_FontStorage_MaintainMappedRead(void);

/* LVGL callbacks used by the generated font descriptor. */
bool UI_FontStorage_GetGlyphDsc(const lv_font_t *font,
                               lv_font_glyph_dsc_t *dsc_out,
                               uint32_t unicode_letter,
                               uint32_t unicode_letter_next);
const uint8_t *UI_FontStorage_GetBitmap(const lv_font_t *font,
                                        uint32_t unicode_letter);

#ifdef __cplusplus
}
#endif

#endif /* UI_FONT_STORAGE_H */
