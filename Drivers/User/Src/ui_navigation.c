#include "ui_navigation.h"

#include <stddef.h>
#include <string.h>

void UiNavigation_Init(UiNavigation *navigation)
{
    if (navigation != NULL)
    {
        memset(navigation, 0, sizeof(*navigation));
    }
}

void UiNavigation_SetLifecycle(UiNavigation *navigation,
                               UiNavigationLifecycleFn on_leave,
                               UiNavigationLifecycleFn on_enter,
                               void *context)
{
    if (navigation == NULL)
    {
        return;
    }
    navigation->on_leave = on_leave;
    navigation->on_enter = on_enter;
    navigation->lifecycle_context = context;
}

void UiNavigation_Request(UiNavigation *navigation,
                          lvgl_app_screen_req_t target)
{
    if ((navigation == NULL) || (target == LVGL_APP_SCREEN_REQ_NONE))
    {
        return;
    }

    if ((navigation->pending != LVGL_APP_SCREEN_REQ_NONE) &&
        (navigation->pending != target))
    {
        navigation->replaced_request_count++;
    }
    if ((navigation->pending == LVGL_APP_SCREEN_REQ_NONE) &&
        (navigation->current != LVGL_APP_SCREEN_REQ_NONE) &&
        (navigation->current != target) &&
        (navigation->on_leave != NULL))
    {
        navigation->on_leave(navigation->current, target,
                             navigation->lifecycle_context);
    }
    navigation->pending = target;
    navigation->request_count++;
}

uint8_t UiNavigation_TakePending(UiNavigation *navigation,
                                 lvgl_app_screen_req_t *target)
{
    if ((navigation == NULL) || (target == NULL) ||
        (navigation->pending == LVGL_APP_SCREEN_REQ_NONE))
    {
        return 0U;
    }

    *target = navigation->pending;
    navigation->pending = LVGL_APP_SCREEN_REQ_NONE;
    return 1U;
}

void UiNavigation_ClearPending(UiNavigation *navigation)
{
    if (navigation != NULL)
    {
        navigation->pending = LVGL_APP_SCREEN_REQ_NONE;
    }
}

void UiNavigation_Commit(UiNavigation *navigation,
                         lvgl_app_screen_req_t target)
{
    lvgl_app_screen_req_t previous;

    if ((navigation == NULL) || (target == LVGL_APP_SCREEN_REQ_NONE))
    {
        return;
    }

    previous = navigation->current;
    navigation->current = target;
    if (previous != target)
    {
        navigation->transition_count++;
        if (navigation->on_enter != NULL)
        {
            navigation->on_enter(previous, target,
                                 navigation->lifecycle_context);
        }
    }
}

lvgl_app_screen_req_t UiNavigation_GetCurrent(const UiNavigation *navigation)
{
    return (navigation != NULL) ? navigation->current :
                                  LVGL_APP_SCREEN_REQ_NONE;
}

uint8_t UiNavigation_GetDepth(lvgl_app_screen_req_t screen)
{
    if (screen == LVGL_APP_SCREEN_REQ_MAIN)
    {
        return 0U;
    }
    if ((screen == LVGL_APP_SCREEN_REQ_MOTOR_SPEED) ||
        (screen == LVGL_APP_SCREEN_REQ_SERVO_ANGLE) ||
        (screen == LVGL_APP_SCREEN_REQ_NES_CACHE) ||
        (screen == LVGL_APP_SCREEN_REQ_NES_PLAYER) ||
        (screen == LVGL_APP_SCREEN_REQ_PHOTO) ||
        (screen == LVGL_APP_SCREEN_REQ_GIF))
    {
        return 2U;
    }
    return (screen == LVGL_APP_SCREEN_REQ_NONE) ? 0U : 1U;
}

void UiNavigation_GetSnapshot(const UiNavigation *navigation,
                              UiNavigationSnapshot *snapshot)
{
    if ((navigation == NULL) || (snapshot == NULL))
    {
        return;
    }

    snapshot->current = navigation->current;
    snapshot->pending = navigation->pending;
    snapshot->request_count = navigation->request_count;
    snapshot->transition_count = navigation->transition_count;
    snapshot->replaced_request_count = navigation->replaced_request_count;
}
