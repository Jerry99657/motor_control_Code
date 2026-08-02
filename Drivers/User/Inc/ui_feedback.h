#ifndef UI_FEEDBACK_H
#define UI_FEEDBACK_H

#include "lvgl.h"
#include "ui_theme.h"

#include <stdint.h>

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *footer;
    lv_obj_t *status_label;
    lv_obj_t *toast;
    lv_obj_t *toast_label;
    uint32_t toast_deadline;
    uint32_t fault_mask;
    uint8_t fault_active;
} ui_feedback_t;

void UI_Feedback_Init(ui_feedback_t *feedback);
void UI_Feedback_Attach(ui_feedback_t *feedback,
                        lv_obj_t *root,
                        lv_obj_t *footer,
                        lv_obj_t *status_label);
void UI_Feedback_Detach(ui_feedback_t *feedback);
void UI_Feedback_SetStatus(ui_feedback_t *feedback, const char *text);
void UI_Feedback_ShowToast(ui_feedback_t *feedback,
                           ui_notice_level_t level,
                           const char *text,
                           uint32_t duration_ms);
void UI_Feedback_SetFault(ui_feedback_t *feedback,
                          uint32_t fault_mask,
                          const char *normal_status);
void UI_Feedback_Process(ui_feedback_t *feedback);

#endif /* UI_FEEDBACK_H */
