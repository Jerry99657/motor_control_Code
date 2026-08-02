#ifndef UI_THEME_H
#define UI_THEME_H

#include "lvgl.h"

typedef enum
{
    UI_VISUAL_NORMAL = 0,
    UI_VISUAL_FOCUSED,
    UI_VISUAL_EDITING,
    UI_VISUAL_RUNNING,
    UI_VISUAL_FAULT,
    UI_VISUAL_DISABLED
} ui_visual_state_t;

typedef enum
{
    UI_NOTICE_INFO = 0,
    UI_NOTICE_SUCCESS,
    UI_NOTICE_WARNING,
    UI_NOTICE_ERROR
} ui_notice_level_t;

void UI_Theme_Init(lv_disp_t *disp);
void UI_Theme_ApplyPageRoot(lv_obj_t *obj);
void UI_Theme_ApplyHeader(lv_obj_t *obj);
void UI_Theme_ApplyContent(lv_obj_t *obj);
void UI_Theme_ApplyFooter(lv_obj_t *obj);
void UI_Theme_ApplyTitle(lv_obj_t *obj);
void UI_Theme_ApplyStatus(lv_obj_t *obj);
void UI_Theme_ApplyPanel(lv_obj_t *obj);
void UI_Theme_ApplyControlRow(lv_obj_t *obj);
void UI_Theme_ApplyDataCard(lv_obj_t *obj);
void UI_Theme_ApplyValueBar(lv_obj_t *obj);
void UI_Theme_ApplyValueFill(lv_obj_t *obj);
void UI_Theme_ApplySlider(lv_obj_t *obj);
void UI_Theme_SetVisualState(lv_obj_t *obj, lv_obj_t *label, ui_visual_state_t state);
void UI_Theme_SetValueBarColor(lv_obj_t *bar, int32_t signed_value);
void UI_Theme_SetValueFillColor(lv_obj_t *bar, int32_t signed_value);
void UI_Theme_ApplyToast(lv_obj_t *toast, lv_obj_t *label, ui_notice_level_t level);
void UI_Theme_SetFooterFault(lv_obj_t *footer, lv_obj_t *label, uint8_t active);
lv_color_t UI_Theme_GetVisualColor(ui_visual_state_t state);

#endif /* UI_THEME_H */
