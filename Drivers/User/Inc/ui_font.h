#ifndef UI_FONT_H
#define UI_FONT_H

#include "lvgl.h"
#include "ui_font_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

LV_FONT_DECLARE(lv_font_gb2312_16)

#define UI_FONT_CJK (&lv_font_gb2312_16)

#ifdef __cplusplus
}
#endif

#endif /* UI_FONT_H */
