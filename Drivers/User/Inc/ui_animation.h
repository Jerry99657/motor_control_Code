#ifndef UI_ANIMATION_H
#define UI_ANIMATION_H

#include "lvgl.h"

#include <stdint.h>

typedef enum
{
    UI_TRANSITION_NONE = 0,
    UI_TRANSITION_FORWARD,
    UI_TRANSITION_BACKWARD
} ui_transition_t;

typedef struct
{
    lv_obj_t *outgoing;
    lv_obj_t *incoming;
    lv_obj_t *shared_overlay;
    lv_area_t shared_start;
    lv_color_t shared_color;
    ui_transition_t pending;
    uint8_t shared_pending;
} ui_transition_manager_t;

typedef struct
{
    int32_t value;
    int32_t target;
    uint32_t last_tick;
    uint8_t initialized;
} ui_value_follower_t;

void UI_TransitionManager_Init(ui_transition_manager_t *manager);
void UI_TransitionManager_Prepare(ui_transition_manager_t *manager,
                                  lv_obj_t *outgoing,
                                  lv_obj_t *incoming,
                                  ui_transition_t transition);
void UI_TransitionManager_Start(ui_transition_manager_t *manager);
void UI_TransitionManager_Cancel(ui_transition_manager_t *manager);
void UI_TransitionManager_CaptureShared(ui_transition_manager_t *manager,
                                       lv_obj_t *source);
void UI_Anim_AttachFocus(lv_obj_t *obj);
void UI_Anim_SetFocusColor(lv_obj_t *obj, lv_color_t color);
void UI_Anim_PulseOpacity(lv_obj_t *obj);
void UI_Anim_StateBounce(lv_obj_t *obj);
void UI_Anim_ToastShow(lv_obj_t *obj);
void UI_Anim_ToastHide(lv_obj_t *obj);
void UI_Anim_SetBarValue(lv_obj_t *bar, int32_t value);
void UI_Anim_StaggerIn(lv_obj_t *obj, uint8_t index);
void UI_Anim_CarouselIn(lv_obj_t *obj, int8_t direction);
void UI_Anim_IconSpin(lv_obj_t *obj, uint8_t enable);
void UI_ValueFollower_Reset(ui_value_follower_t *follower, int32_t value);
void UI_ValueFollower_SetTarget(ui_value_follower_t *follower, int32_t target);
int32_t UI_ValueFollower_Update(ui_value_follower_t *follower,
                                uint32_t now,
                                uint32_t response_ms);
uint8_t UI_LabelSetTextIfChanged(lv_obj_t *label, const char *text);

#endif /* UI_ANIMATION_H */
