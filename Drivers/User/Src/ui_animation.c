#include "ui_animation.h"

#include <string.h>

#define UI_PAGE_ENTER_DISTANCE  38
#define UI_PAGE_EXIT_DISTANCE   12
#define UI_PAGE_ENTER_TIME_MS   230U
#define UI_PAGE_EXIT_TIME_MS    180U
#define UI_FOCUS_IN_TIME_MS     145U
#define UI_FOCUS_OUT_TIME_MS     90U
#define UI_PRESS_IN_TIME_MS      55U
#define UI_PRESS_BOUNCE_TIME_MS  70U
#define UI_PRESS_OPA_TIME_MS     95U
#define UI_STATUS_ANIM_TIME_MS  140U
#define UI_STATE_LIFT_TIME_MS     70U
#define UI_STATE_SETTLE_TIME_MS   90U
#define UI_TOAST_ENTER_TIME_MS   180U
#define UI_TOAST_EXIT_TIME_MS    120U
#define UI_VALUE_FOLLOW_TIME_MS  160U
#define UI_STAGGER_MAX_OBJECTS      4U
#define UI_STAGGER_STEP_MS         20U
#define UI_STAGGER_TIME_MS        170U
#define UI_STAGGER_START_Y          8
#define UI_CAROUSEL_DISTANCE       18
#define UI_CAROUSEL_TIME_MS       180U
#define UI_SHARED_TIME_MS         190U
#define UI_FOCUS_INDICATOR_WIDTH 4
#define UI_FOCUS_SHIFT_X          2
#define UI_PRESS_SHIFT_Y          2
#define UI_PRESS_BOUNCE_Y        -1
#define UI_PRESS_OPA             LV_OPA_90
#define UI_STATE_LIFT_Y           -2
#define UI_TOAST_START_Y          12
#define UI_TOAST_EXIT_Y           10

static void ui_anim_set_translate_x(void *var, int32_t value)
{
    lv_obj_set_style_translate_x((lv_obj_t *)var, (lv_coord_t)value, LV_PART_MAIN);
}

static void ui_anim_page_ready_cb(lv_anim_t *anim)
{
    ui_transition_manager_t *manager;
    lv_obj_t *page;

    page = (lv_obj_t *)anim->var;
    if ((page != NULL) && (lv_obj_is_valid(page) != false))
    {
        lv_obj_set_style_translate_x(page, 0, LV_PART_MAIN);
    }

    manager = (ui_transition_manager_t *)lv_anim_get_user_data(anim);
    if ((manager == NULL) || (manager->incoming != page))
    {
        return;
    }

    if ((manager->outgoing != NULL) &&
        (manager->outgoing != manager->incoming) &&
        (lv_obj_is_valid(manager->outgoing) != false))
    {
        lv_anim_del(manager->outgoing, ui_anim_set_translate_x);
        lv_obj_del(manager->outgoing);
    }

    manager->outgoing = NULL;
    manager->incoming = NULL;
    manager->pending = UI_TRANSITION_NONE;
}

static void ui_anim_set_focus_indicator(void *var, int32_t value)
{
    lv_obj_t *obj = (lv_obj_t *)var;

    lv_obj_set_style_border_width(obj, (lv_coord_t)value, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, (lv_coord_t)value,
                                  LV_PART_MAIN | LV_STATE_FOCUSED);
}

static void ui_anim_set_opa(void *var, int32_t value)
{
    lv_obj_set_style_opa((lv_obj_t *)var, (lv_opa_t)value, LV_PART_MAIN);
}

static void ui_anim_set_translate_y(void *var, int32_t value)
{
    lv_obj_set_style_translate_y((lv_obj_t *)var, (lv_coord_t)value, LV_PART_MAIN);
}

static void ui_anim_set_bar_value(void *var, int32_t value)
{
    lv_obj_t *bar = (lv_obj_t *)var;

    if ((bar != NULL) && (lv_obj_is_valid(bar) != false))
    {
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
    }
}

static void ui_anim_set_angle(void *var, int32_t value)
{
    lv_obj_set_style_transform_angle((lv_obj_t *)var, (int16_t)value, LV_PART_MAIN);
}

static void ui_anim_set_zoom(void *var, int32_t value)
{
    lv_obj_set_style_transform_zoom((lv_obj_t *)var, (uint16_t)value, LV_PART_MAIN);
}

static void ui_anim_shared_exec(void *var, int32_t value)
{
    ui_transition_manager_t *manager = (ui_transition_manager_t *)var;
    lv_area_t end_area;
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;

    if ((manager == NULL) || (manager->shared_overlay == NULL) ||
        (lv_obj_is_valid(manager->shared_overlay) == false) ||
        (manager->incoming == NULL) ||
        (lv_obj_is_valid(manager->incoming) == false))
    {
        return;
    }

    lv_obj_get_coords(manager->incoming, &end_area);
    end_area.x1 += 10;
    end_area.x2 -= 10;
    end_area.y1 += 4;
    end_area.y2 = end_area.y1 + 27;

    x = manager->shared_start.x1 +
        (((int32_t)end_area.x1 - manager->shared_start.x1) * value) / 256;
    y = manager->shared_start.y1 +
        (((int32_t)end_area.y1 - manager->shared_start.y1) * value) / 256;
    width = lv_area_get_width(&manager->shared_start) +
            ((lv_area_get_width(&end_area) - lv_area_get_width(&manager->shared_start)) * value) / 256;
    height = lv_area_get_height(&manager->shared_start) +
             ((lv_area_get_height(&end_area) - lv_area_get_height(&manager->shared_start)) * value) / 256;

    lv_obj_set_pos(manager->shared_overlay, (lv_coord_t)x, (lv_coord_t)y);
    lv_obj_set_size(manager->shared_overlay, (lv_coord_t)width, (lv_coord_t)height);
}

static void ui_anim_shared_ready_cb(lv_anim_t *anim)
{
    ui_transition_manager_t *manager = (ui_transition_manager_t *)anim->var;

    if (manager == NULL)
    {
        return;
    }

    if ((manager->shared_overlay != NULL) &&
        (lv_obj_is_valid(manager->shared_overlay) != false))
    {
        lv_obj_del(manager->shared_overlay);
    }
    manager->shared_overlay = NULL;
    manager->shared_pending = 0U;
}

static void ui_anim_shared_start(ui_transition_manager_t *manager)
{
    lv_anim_t anim;
    lv_obj_t *root;

    if ((manager == NULL) || (manager->shared_pending == 0U) ||
        (manager->incoming == NULL) ||
        (lv_obj_is_valid(manager->incoming) == false) ||
        (manager->pending != UI_TRANSITION_FORWARD))
    {
        if (manager != NULL)
        {
            manager->shared_pending = 0U;
        }
        return;
    }

    root = lv_obj_get_parent(lv_obj_get_parent(manager->incoming));
    if ((root == NULL) || (lv_obj_is_valid(root) == false))
    {
        manager->shared_pending = 0U;
        return;
    }

    manager->shared_overlay = lv_obj_create(root);
    lv_obj_remove_style_all(manager->shared_overlay);
    lv_obj_set_style_bg_color(manager->shared_overlay, manager->shared_color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(manager->shared_overlay, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_radius(manager->shared_overlay, 6, LV_PART_MAIN);
    lv_obj_set_pos(manager->shared_overlay, manager->shared_start.x1,
                   manager->shared_start.y1);
    lv_obj_set_size(manager->shared_overlay,
                    lv_area_get_width(&manager->shared_start),
                    lv_area_get_height(&manager->shared_start));
    lv_obj_clear_flag(manager->shared_overlay,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(manager->shared_overlay);

    lv_anim_init(&anim);
    lv_anim_set_var(&anim, manager);
    lv_anim_set_exec_cb(&anim, ui_anim_shared_exec);
    lv_anim_set_values(&anim, 0, 256);
    lv_anim_set_time(&anim, UI_SHARED_TIME_MS);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&anim, ui_anim_shared_ready_cb);
    (void)lv_anim_start(&anim);
}

static void ui_anim_start_value(lv_obj_t *obj,
                                lv_anim_exec_xcb_t exec_cb,
                                int32_t current,
                                int32_t target,
                                uint32_t time_ms,
                                lv_anim_path_cb_t path_cb,
                                lv_anim_ready_cb_t ready_cb)
{
    lv_anim_t anim;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    lv_anim_del(obj, exec_cb);
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, obj);
    lv_anim_set_exec_cb(&anim, exec_cb);
    lv_anim_set_values(&anim, current, target);
    lv_anim_set_time(&anim, time_ms);
    lv_anim_set_path_cb(&anim, path_cb);
    if (ready_cb != NULL)
    {
        lv_anim_set_ready_cb(&anim, ready_cb);
    }
    (void)lv_anim_start(&anim);
}

static void ui_anim_focus_to(lv_obj_t *obj, uint8_t focused)
{
    uint32_t time_ms;
    lv_anim_path_cb_t path_cb;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    time_ms = (focused != 0U) ? UI_FOCUS_IN_TIME_MS : UI_FOCUS_OUT_TIME_MS;
    path_cb = (focused != 0U) ? lv_anim_path_overshoot : lv_anim_path_ease_out;

    ui_anim_start_value(obj,
                        ui_anim_set_focus_indicator,
                        lv_obj_get_style_border_width(obj, LV_PART_MAIN),
                        (focused != 0U) ? UI_FOCUS_INDICATOR_WIDTH : 0,
                        time_ms,
                        path_cb,
                        NULL);
    ui_anim_start_value(obj,
                        ui_anim_set_translate_x,
                        lv_obj_get_style_translate_x(obj, LV_PART_MAIN),
                        (focused != 0U) ? UI_FOCUS_SHIFT_X : 0,
                        time_ms,
                        path_cb,
                        NULL);
}

static void ui_anim_press_settle_ready_cb(lv_anim_t *anim)
{
    lv_obj_t *obj = (lv_obj_t *)anim->var;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        UI_PRESS_BOUNCE_Y,
                        0,
                        UI_PRESS_BOUNCE_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
}

static void ui_anim_press_down(lv_obj_t *obj)
{
    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        lv_obj_get_style_translate_y(obj, LV_PART_MAIN),
                        UI_PRESS_SHIFT_Y,
                        UI_PRESS_IN_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
    ui_anim_start_value(obj,
                        ui_anim_set_opa,
                        lv_obj_get_style_opa(obj, LV_PART_MAIN),
                        UI_PRESS_OPA,
                        UI_PRESS_IN_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
}

static void ui_anim_press_release(lv_obj_t *obj)
{
    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        lv_obj_get_style_translate_y(obj, LV_PART_MAIN),
                        UI_PRESS_BOUNCE_Y,
                        UI_PRESS_BOUNCE_TIME_MS,
                        lv_anim_path_ease_out,
                        ui_anim_press_settle_ready_cb);
    ui_anim_start_value(obj,
                        ui_anim_set_opa,
                        lv_obj_get_style_opa(obj, LV_PART_MAIN),
                        LV_OPA_COVER,
                        UI_PRESS_OPA_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
}

static void ui_anim_press_reset(lv_obj_t *obj)
{
    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        lv_obj_get_style_translate_y(obj, LV_PART_MAIN),
                        0,
                        UI_FOCUS_OUT_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
    ui_anim_start_value(obj,
                        ui_anim_set_opa,
                        lv_obj_get_style_opa(obj, LV_PART_MAIN),
                        LV_OPA_COVER,
                        UI_FOCUS_OUT_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
}

static void ui_anim_state_settle_ready_cb(lv_anim_t *anim)
{
    lv_obj_t *obj = (lv_obj_t *)anim->var;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        UI_STATE_LIFT_Y,
                        0,
                        UI_STATE_SETTLE_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
}

static void ui_anim_toast_hidden_ready_cb(lv_anim_t *anim)
{
    lv_obj_t *obj = (lv_obj_t *)anim->var;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_transform_zoom(obj, 256U, LV_PART_MAIN);
}

static void ui_anim_focus_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_FOCUSED)
    {
        ui_anim_focus_to(obj, 1U);
    }
    else if (code == LV_EVENT_DEFOCUSED)
    {
        ui_anim_focus_to(obj, 0U);
        ui_anim_press_reset(obj);
    }
    else if (code == LV_EVENT_PRESSED)
    {
        ui_anim_press_down(obj);
    }
    else if ((code == LV_EVENT_RELEASED) || (code == LV_EVENT_PRESS_LOST))
    {
        ui_anim_press_release(obj);
    }
    else if (code == LV_EVENT_DELETE)
    {
        lv_anim_del(obj, ui_anim_set_focus_indicator);
        lv_anim_del(obj, ui_anim_set_translate_x);
        lv_anim_del(obj, ui_anim_set_translate_y);
        lv_anim_del(obj, ui_anim_set_opa);
    }
}

/*
 * A page can leave while the focused object's short defocus animation is
 * still running.  Neutralize only objects registered by UI_Anim_AttachFocus
 * so the outgoing frame does not carry a blue selection indicator through
 * the page transition.
 */
static void ui_anim_clear_focus_tree(lv_obj_t *root)
{
    uint32_t i;
    uint32_t child_count;

    if ((root == NULL) || (lv_obj_is_valid(root) == false))
    {
        return;
    }

    if (lv_obj_has_flag(root, LV_OBJ_FLAG_USER_1) != false)
    {
        lv_anim_del(root, ui_anim_set_focus_indicator);
        lv_anim_del(root, ui_anim_set_translate_x);
        lv_anim_del(root, ui_anim_set_translate_y);
        lv_anim_del(root, ui_anim_set_opa);
        lv_obj_clear_state(root, LV_STATE_FOCUSED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(root, 0,
                                      LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_translate_x(root, 0, LV_PART_MAIN);
        lv_obj_set_style_translate_y(root, 0, LV_PART_MAIN);
        lv_obj_set_style_opa(root, LV_OPA_COVER, LV_PART_MAIN);
    }

    child_count = lv_obj_get_child_cnt(root);
    for (i = 0U; i < child_count; ++i)
    {
        ui_anim_clear_focus_tree(lv_obj_get_child(root, (int32_t)i));
    }
}

static void ui_anim_page_enter(ui_transition_manager_t *manager)
{
    lv_anim_t incoming_anim;
    lv_anim_t outgoing_anim;
    lv_obj_t *incoming;
    lv_obj_t *outgoing;
    int32_t start_x;
    int32_t exit_x;
    ui_transition_t transition;

    if ((manager == NULL) || (manager->incoming == NULL) ||
        (lv_obj_is_valid(manager->incoming) == false))
    {
        return;
    }

    incoming = manager->incoming;
    outgoing = manager->outgoing;
    transition = manager->pending;

    lv_anim_del(incoming, ui_anim_set_translate_x);
    if (transition == UI_TRANSITION_NONE)
    {
        lv_obj_set_style_translate_x(incoming, 0, LV_PART_MAIN);
        if ((outgoing != NULL) && (outgoing != incoming) &&
            (lv_obj_is_valid(outgoing) != false))
        {
            lv_anim_del(outgoing, ui_anim_set_translate_x);
            lv_obj_del(outgoing);
        }
        manager->outgoing = NULL;
        manager->incoming = NULL;
        manager->shared_overlay = NULL;
        manager->pending = UI_TRANSITION_NONE;
        manager->shared_pending = 0U;
        return;
    }

    start_x = (transition == UI_TRANSITION_FORWARD) ? UI_PAGE_ENTER_DISTANCE : -UI_PAGE_ENTER_DISTANCE;
    exit_x = (transition == UI_TRANSITION_FORWARD) ? -UI_PAGE_EXIT_DISTANCE : UI_PAGE_EXIT_DISTANCE;
    lv_obj_set_style_translate_x(incoming, (lv_coord_t)start_x, LV_PART_MAIN);

    if ((outgoing != NULL) && (outgoing != incoming) &&
        (lv_obj_is_valid(outgoing) != false))
    {
        lv_anim_del(outgoing, ui_anim_set_translate_x);
        lv_obj_set_style_translate_x(outgoing, 0, LV_PART_MAIN);
        lv_anim_init(&outgoing_anim);
        lv_anim_set_var(&outgoing_anim, outgoing);
        lv_anim_set_exec_cb(&outgoing_anim, ui_anim_set_translate_x);
        lv_anim_set_values(&outgoing_anim, 0, exit_x);
        lv_anim_set_time(&outgoing_anim, UI_PAGE_EXIT_TIME_MS);
        lv_anim_set_path_cb(&outgoing_anim, lv_anim_path_ease_out);
        (void)lv_anim_start(&outgoing_anim);
    }

    lv_anim_init(&incoming_anim);
    lv_anim_set_var(&incoming_anim, incoming);
    lv_anim_set_exec_cb(&incoming_anim, ui_anim_set_translate_x);
    lv_anim_set_values(&incoming_anim, start_x, 0);
    lv_anim_set_time(&incoming_anim, UI_PAGE_ENTER_TIME_MS);
    lv_anim_set_path_cb(&incoming_anim, lv_anim_path_overshoot);
    lv_anim_set_ready_cb(&incoming_anim, ui_anim_page_ready_cb);
    lv_anim_set_user_data(&incoming_anim, manager);
    (void)lv_anim_start(&incoming_anim);
}

void UI_TransitionManager_Init(ui_transition_manager_t *manager)
{
    if (manager != NULL)
    {
        manager->outgoing = NULL;
        manager->incoming = NULL;
        manager->shared_overlay = NULL;
        manager->pending = UI_TRANSITION_NONE;
        manager->shared_pending = 0U;
    }
}

void UI_TransitionManager_Prepare(ui_transition_manager_t *manager,
                                  lv_obj_t *outgoing,
                                  lv_obj_t *incoming,
                                  ui_transition_t transition)
{
    if (manager == NULL)
    {
        return;
    }

    UI_TransitionManager_Cancel(manager);
    manager->outgoing = outgoing;
    manager->incoming = incoming;
    manager->pending = transition;

    if ((incoming != NULL) && (lv_obj_is_valid(incoming) != false))
    {
        lv_obj_set_style_translate_x(incoming, 0, LV_PART_MAIN);
    }
}

void UI_TransitionManager_Start(ui_transition_manager_t *manager)
{
    if ((manager == NULL) || (manager->incoming == NULL) ||
        (lv_obj_is_valid(manager->incoming) == false))
    {
        return;
    }

    lv_obj_update_layout(manager->incoming);
    ui_anim_clear_focus_tree(manager->outgoing);
    ui_anim_shared_start(manager);
    ui_anim_page_enter(manager);
}

void UI_TransitionManager_Cancel(ui_transition_manager_t *manager)
{
    if (manager == NULL)
    {
        return;
    }

    lv_anim_del(manager, ui_anim_shared_exec);
    if (manager->shared_overlay != NULL)
    {
        if (lv_obj_is_valid(manager->shared_overlay) != false)
        {
            lv_obj_del(manager->shared_overlay);
        }
        manager->shared_pending = 0U;
    }
    manager->shared_overlay = NULL;

    if ((manager->outgoing != NULL) &&
        (manager->outgoing != manager->incoming) &&
        (lv_obj_is_valid(manager->outgoing) != false))
    {
        lv_anim_del(manager->outgoing, ui_anim_set_translate_x);
        lv_obj_del(manager->outgoing);
    }

    if ((manager->incoming != NULL) &&
        (lv_obj_is_valid(manager->incoming) != false))
    {
        lv_anim_del(manager->incoming, ui_anim_set_translate_x);
        lv_obj_set_style_translate_x(manager->incoming, 0, LV_PART_MAIN);
    }

    manager->outgoing = NULL;
    manager->incoming = NULL;
    manager->pending = UI_TRANSITION_NONE;
}

void UI_TransitionManager_CaptureShared(ui_transition_manager_t *manager,
                                       lv_obj_t *source)
{
    if ((manager == NULL) || (source == NULL) ||
        (lv_obj_is_valid(source) == false))
    {
        return;
    }

    lv_obj_get_coords(source, &manager->shared_start);
    manager->shared_color = lv_obj_get_style_border_color(source, LV_PART_MAIN);
    manager->shared_pending = 1U;
}

void UI_Anim_AttachFocus(lv_obj_t *obj)
{
    if (obj == NULL)
    {
        return;
    }

    lv_obj_add_flag(obj, LV_OBJ_FLAG_USER_1);
    lv_obj_set_style_border_side(obj, LV_BORDER_SIDE_LEFT, LV_PART_MAIN);
    lv_obj_set_style_border_side(obj, LV_BORDER_SIDE_LEFT,
                                 LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x2563EB), LV_PART_MAIN);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x2563EB),
                                  LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER,
                                LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_translate_x(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(obj, ui_anim_focus_event_cb, LV_EVENT_ALL, NULL);
}

void UI_Anim_SetFocusColor(lv_obj_t *obj, lv_color_t color)
{
    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    lv_obj_set_style_border_color(obj, color, LV_PART_MAIN);
    lv_obj_set_style_border_color(obj, color, LV_PART_MAIN | LV_STATE_FOCUSED);
}

void UI_Anim_PulseOpacity(lv_obj_t *obj)
{
    lv_anim_t anim;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    lv_anim_del(obj, ui_anim_set_opa);
    lv_obj_set_style_opa(obj, LV_OPA_80, LV_PART_MAIN);
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, obj);
    lv_anim_set_exec_cb(&anim, ui_anim_set_opa);
    lv_anim_set_values(&anim, LV_OPA_80, LV_OPA_COVER);
    lv_anim_set_time(&anim, UI_STATUS_ANIM_TIME_MS);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
    (void)lv_anim_start(&anim);
}

void UI_Anim_StateBounce(lv_obj_t *obj)
{
    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        lv_obj_get_style_translate_y(obj, LV_PART_MAIN),
                        UI_STATE_LIFT_Y,
                        UI_STATE_LIFT_TIME_MS,
                        lv_anim_path_ease_out,
                        ui_anim_state_settle_ready_cb);
}

void UI_Anim_ToastShow(lv_obj_t *obj)
{
    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    lv_anim_del(obj, ui_anim_set_translate_y);
    lv_anim_del(obj, ui_anim_set_opa);
    lv_anim_del(obj, ui_anim_set_zoom);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_translate_y(obj, UI_TOAST_START_Y, LV_PART_MAIN);
    lv_obj_set_style_opa(obj, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_transform_zoom(obj, 250U, LV_PART_MAIN);

    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        UI_TOAST_START_Y,
                        0,
                        UI_TOAST_ENTER_TIME_MS,
                        lv_anim_path_overshoot,
                        NULL);
    ui_anim_start_value(obj,
                        ui_anim_set_opa,
                        LV_OPA_60,
                        LV_OPA_COVER,
                        UI_TOAST_EXIT_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
    ui_anim_start_value(obj,
                        ui_anim_set_zoom,
                        250,
                        256,
                        UI_TOAST_ENTER_TIME_MS,
                        lv_anim_path_overshoot,
                        NULL);
}

void UI_Anim_ToastHide(lv_obj_t *obj)
{
    if ((obj == NULL) || (lv_obj_is_valid(obj) == false) ||
        lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN))
    {
        return;
    }

    ui_anim_start_value(obj,
                        ui_anim_set_translate_y,
                        lv_obj_get_style_translate_y(obj, LV_PART_MAIN),
                        UI_TOAST_EXIT_Y,
                        UI_TOAST_EXIT_TIME_MS,
                        lv_anim_path_ease_in,
                        ui_anim_toast_hidden_ready_cb);
    ui_anim_start_value(obj,
                        ui_anim_set_opa,
                        lv_obj_get_style_opa(obj, LV_PART_MAIN),
                        LV_OPA_TRANSP,
                        UI_TOAST_EXIT_TIME_MS,
                        lv_anim_path_ease_in,
                        NULL);
    ui_anim_start_value(obj,
                        ui_anim_set_zoom,
                        lv_obj_get_style_transform_zoom(obj, LV_PART_MAIN),
                        250,
                        UI_TOAST_EXIT_TIME_MS,
                        lv_anim_path_ease_in,
                        NULL);
}

void UI_Anim_SetBarValue(lv_obj_t *bar, int32_t value)
{
    int32_t current;

    if ((bar == NULL) || (lv_obj_is_valid(bar) == false))
    {
        return;
    }

    current = lv_bar_get_value(bar);
    if (current == value)
    {
        return;
    }

    ui_anim_start_value(bar,
                        ui_anim_set_bar_value,
                        current,
                        value,
                        UI_VALUE_FOLLOW_TIME_MS,
                        lv_anim_path_ease_out,
                        NULL);
}

void UI_Anim_StaggerIn(lv_obj_t *obj, uint8_t index)
{
    lv_anim_t anim;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false) ||
        (index >= UI_STAGGER_MAX_OBJECTS))
    {
        return;
    }

    lv_anim_del(obj, ui_anim_set_translate_y);
    lv_anim_del(obj, ui_anim_set_opa);
    lv_obj_set_style_translate_y(obj, UI_STAGGER_START_Y, LV_PART_MAIN);
    lv_obj_set_style_opa(obj, LV_OPA_70, LV_PART_MAIN);

    lv_anim_init(&anim);
    lv_anim_set_var(&anim, obj);
    lv_anim_set_exec_cb(&anim, ui_anim_set_translate_y);
    lv_anim_set_values(&anim, UI_STAGGER_START_Y, 0);
    lv_anim_set_delay(&anim, (uint32_t)index * UI_STAGGER_STEP_MS);
    lv_anim_set_time(&anim, UI_STAGGER_TIME_MS);
    lv_anim_set_path_cb(&anim, lv_anim_path_overshoot);
    (void)lv_anim_start(&anim);

    lv_anim_init(&anim);
    lv_anim_set_var(&anim, obj);
    lv_anim_set_exec_cb(&anim, ui_anim_set_opa);
    lv_anim_set_values(&anim, LV_OPA_70, LV_OPA_COVER);
    lv_anim_set_delay(&anim, (uint32_t)index * UI_STAGGER_STEP_MS);
    lv_anim_set_time(&anim, UI_STAGGER_TIME_MS);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
    (void)lv_anim_start(&anim);
}

void UI_Anim_CarouselIn(lv_obj_t *obj, int8_t direction)
{
    int32_t start_x;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false) || (direction == 0))
    {
        return;
    }

    start_x = (direction > 0) ? UI_CAROUSEL_DISTANCE : -UI_CAROUSEL_DISTANCE;
    lv_anim_del(obj, ui_anim_set_translate_x);
    lv_anim_del(obj, ui_anim_set_opa);
    lv_obj_set_style_translate_x(obj, (lv_coord_t)start_x, LV_PART_MAIN);
    lv_obj_set_style_opa(obj, LV_OPA_70, LV_PART_MAIN);

    ui_anim_start_value(obj, ui_anim_set_translate_x, start_x, 0,
                        UI_CAROUSEL_TIME_MS, lv_anim_path_overshoot, NULL);
    ui_anim_start_value(obj, ui_anim_set_opa, LV_OPA_70, LV_OPA_COVER,
                        UI_CAROUSEL_TIME_MS, lv_anim_path_ease_out, NULL);
}

void UI_Anim_IconSpin(lv_obj_t *obj, uint8_t enable)
{
    lv_anim_t anim;

    if ((obj == NULL) || (lv_obj_is_valid(obj) == false))
    {
        return;
    }

    lv_anim_del(obj, ui_anim_set_angle);
    if (enable == 0U)
    {
        lv_obj_set_style_transform_angle(obj, 0, LV_PART_MAIN);
        return;
    }

    lv_anim_init(&anim);
    lv_anim_set_var(&anim, obj);
    lv_anim_set_exec_cb(&anim, ui_anim_set_angle);
    lv_anim_set_values(&anim, 0, 3600);
    lv_anim_set_time(&anim, 900U);
    lv_anim_set_path_cb(&anim, lv_anim_path_linear);
    lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
    (void)lv_anim_start(&anim);
}

void UI_ValueFollower_Reset(ui_value_follower_t *follower, int32_t value)
{
    if (follower == NULL)
    {
        return;
    }

    follower->value = value;
    follower->target = value;
    follower->last_tick = lv_tick_get();
    follower->initialized = 1U;
}

void UI_ValueFollower_SetTarget(ui_value_follower_t *follower, int32_t target)
{
    if (follower != NULL)
    {
        if (follower->initialized == 0U)
        {
            UI_ValueFollower_Reset(follower, target);
        }
        follower->target = target;
    }
}

int32_t UI_ValueFollower_Update(ui_value_follower_t *follower,
                                uint32_t now,
                                uint32_t response_ms)
{
    uint32_t elapsed;
    int32_t delta;
    int32_t step;

    if ((follower == NULL) || (follower->initialized == 0U))
    {
        return 0;
    }

    elapsed = now - follower->last_tick;
    follower->last_tick = now;
    delta = follower->target - follower->value;
    if ((delta == 0) || (elapsed == 0U))
    {
        return follower->value;
    }

    if ((response_ms == 0U) || (elapsed >= response_ms))
    {
        follower->value = follower->target;
        return follower->value;
    }

    step = (int32_t)(((int64_t)delta * (int64_t)elapsed) / (int64_t)response_ms);
    if (step == 0)
    {
        step = (delta > 0) ? 1 : -1;
    }
    follower->value += step;
    if (((delta > 0) && (follower->value > follower->target)) ||
        ((delta < 0) && (follower->value < follower->target)))
    {
        follower->value = follower->target;
    }
    return follower->value;
}

uint8_t UI_LabelSetTextIfChanged(lv_obj_t *label, const char *text)
{
    const char *current;

    if ((label == NULL) || (text == NULL) || (lv_obj_is_valid(label) == false))
    {
        return 0U;
    }

    current = lv_label_get_text(label);
    if ((current != NULL) && (strcmp(current, text) == 0))
    {
        return 0U;
    }

    lv_label_set_text(label, text);
    return 1U;
}
