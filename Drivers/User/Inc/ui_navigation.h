#ifndef UI_NAVIGATION_H
#define UI_NAVIGATION_H

#include <stdint.h>

typedef enum
{
    LVGL_APP_SCREEN_REQ_NONE = 0,
    LVGL_APP_SCREEN_REQ_MAIN,
    LVGL_APP_SCREEN_REQ_MOTOR_MENU,
    LVGL_APP_SCREEN_REQ_MOTOR_SPEED,
    LVGL_APP_SCREEN_REQ_SERVO_ANGLE,
    LVGL_APP_SCREEN_REQ_COMMAND,
    LVGL_APP_SCREEN_REQ_SD_BROWSER,
    LVGL_APP_SCREEN_REQ_NES_CACHE,
    LVGL_APP_SCREEN_REQ_NES_PLAYER,
    LVGL_APP_SCREEN_REQ_MECANUM,
    LVGL_APP_SCREEN_REQ_MPU6500,
    LVGL_APP_SCREEN_REQ_WS2812,
    LVGL_APP_SCREEN_REQ_FOC,
    LVGL_APP_SCREEN_REQ_DIAGNOSTICS,
    LVGL_APP_SCREEN_REQ_CAMERA,
    LVGL_APP_SCREEN_REQ_DISPLAY_SETTINGS,
    LVGL_APP_SCREEN_REQ_GIF
} lvgl_app_screen_req_t;

typedef void (*UiNavigationLifecycleFn)(lvgl_app_screen_req_t current,
                                        lvgl_app_screen_req_t target,
                                        void *context);

typedef struct
{
    lvgl_app_screen_req_t current;
    lvgl_app_screen_req_t pending;
    UiNavigationLifecycleFn on_leave;
    UiNavigationLifecycleFn on_enter;
    void *lifecycle_context;
    uint32_t request_count;
    uint32_t transition_count;
    uint32_t replaced_request_count;
} UiNavigation;

typedef struct
{
    lvgl_app_screen_req_t current;
    lvgl_app_screen_req_t pending;
    uint32_t request_count;
    uint32_t transition_count;
    uint32_t replaced_request_count;
} UiNavigationSnapshot;

void UiNavigation_Init(UiNavigation *navigation);
void UiNavigation_SetLifecycle(UiNavigation *navigation,
                               UiNavigationLifecycleFn on_leave,
                               UiNavigationLifecycleFn on_enter,
                               void *context);
void UiNavigation_Request(UiNavigation *navigation,
                          lvgl_app_screen_req_t target);
uint8_t UiNavigation_TakePending(UiNavigation *navigation,
                                 lvgl_app_screen_req_t *target);
void UiNavigation_ClearPending(UiNavigation *navigation);
void UiNavigation_Commit(UiNavigation *navigation,
                         lvgl_app_screen_req_t target);
lvgl_app_screen_req_t UiNavigation_GetCurrent(const UiNavigation *navigation);
uint8_t UiNavigation_GetDepth(lvgl_app_screen_req_t screen);
void UiNavigation_GetSnapshot(const UiNavigation *navigation,
                              UiNavigationSnapshot *snapshot);

#endif /* UI_NAVIGATION_H */
