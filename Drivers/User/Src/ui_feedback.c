#include "ui_feedback.h"

#include "ui_animation.h"
#include "ui_page.h"

#include <stdio.h>
#include <string.h>

void UI_Feedback_Init(ui_feedback_t *feedback)
{
    if (feedback != NULL)
    {
        (void)memset(feedback, 0, sizeof(*feedback));
    }
}

void UI_Feedback_Detach(ui_feedback_t *feedback)
{
    if (feedback == NULL)
    {
        return;
    }

    feedback->root = NULL;
    feedback->footer = NULL;
    feedback->status_label = NULL;
    feedback->toast = NULL;
    feedback->toast_label = NULL;
    feedback->toast_deadline = 0U;
    feedback->fault_mask = 0U;
    feedback->fault_active = 0U;
}

void UI_Feedback_Attach(ui_feedback_t *feedback,
                        lv_obj_t *root,
                        lv_obj_t *footer,
                        lv_obj_t *status_label)
{
    if ((feedback == NULL) || (root == NULL) || (footer == NULL) || (status_label == NULL))
    {
        return;
    }

    UI_Feedback_Detach(feedback);
    feedback->root = root;
    feedback->footer = footer;
    feedback->status_label = status_label;

    feedback->toast = lv_obj_create(root);
    lv_obj_remove_style_all(feedback->toast);
    lv_obj_set_size(feedback->toast, 216, 28);
    lv_obj_align(feedback->toast, LV_ALIGN_BOTTOM_MID, 0, -(UI_PAGE_FOOTER_HEIGHT + 5));
    lv_obj_clear_flag(feedback->toast, LV_OBJ_FLAG_SCROLLABLE);

    feedback->toast_label = lv_label_create(feedback->toast);
    lv_label_set_long_mode(feedback->toast_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(feedback->toast_label, 204);
    lv_obj_center(feedback->toast_label);
    lv_obj_add_flag(feedback->toast, LV_OBJ_FLAG_HIDDEN);
}

void UI_Feedback_SetStatus(ui_feedback_t *feedback, const char *text)
{
    if ((feedback == NULL) || (text == NULL) || (feedback->fault_active != 0U) ||
        (feedback->status_label == NULL) || (lv_obj_is_valid(feedback->status_label) == false))
    {
        return;
    }

    if (UI_LabelSetTextIfChanged(feedback->status_label, text) != 0U)
    {
        UI_Anim_PulseOpacity(feedback->status_label);
    }
}

void UI_Feedback_ShowToast(ui_feedback_t *feedback,
                           ui_notice_level_t level,
                           const char *text,
                           uint32_t duration_ms)
{
    if ((feedback == NULL) || (text == NULL) || (feedback->toast == NULL) ||
        (lv_obj_is_valid(feedback->toast) == false))
    {
        return;
    }

    UI_Theme_ApplyToast(feedback->toast, feedback->toast_label, level);
    lv_label_set_text(feedback->toast_label, text);
    lv_obj_move_foreground(feedback->toast);
    UI_Anim_ToastShow(feedback->toast);
    feedback->toast_deadline = lv_tick_get() + ((duration_ms == 0U) ? 1500U : duration_ms);
}

void UI_Feedback_SetFault(ui_feedback_t *feedback,
                          uint32_t fault_mask,
                          const char *normal_status)
{
    char fault_text[40];
    uint8_t state_changed;

    if ((feedback == NULL) || (feedback->footer == NULL) ||
        (lv_obj_is_valid(feedback->footer) == false))
    {
        return;
    }

    state_changed = (feedback->fault_mask != fault_mask) ? 1U : 0U;
    feedback->fault_mask = fault_mask;
    feedback->fault_active = (fault_mask != 0U) ? 1U : 0U;
    UI_Theme_SetFooterFault(feedback->footer, feedback->status_label, feedback->fault_active);

    if (feedback->fault_active != 0U)
    {
        (void)snprintf(fault_text, sizeof(fault_text), "SAFETY FAULT 0x%02lX",
                       (unsigned long)fault_mask);
        lv_label_set_text(feedback->status_label, fault_text);
    }
    else if (normal_status != NULL)
    {
        lv_label_set_text(feedback->status_label, normal_status);
    }

    if (state_changed != 0U)
    {
        UI_Anim_StateBounce(feedback->footer);
        UI_Anim_PulseOpacity(feedback->status_label);
    }
}

void UI_Feedback_Process(ui_feedback_t *feedback)
{
    if ((feedback == NULL) || (feedback->toast == NULL) ||
        (lv_obj_is_valid(feedback->toast) == false) ||
        lv_obj_has_flag(feedback->toast, LV_OBJ_FLAG_HIDDEN) ||
        (feedback->toast_deadline == 0U))
    {
        return;
    }

    if ((int32_t)(lv_tick_get() - feedback->toast_deadline) >= 0)
    {
        feedback->toast_deadline = 0U;
        UI_Anim_ToastHide(feedback->toast);
    }
}
