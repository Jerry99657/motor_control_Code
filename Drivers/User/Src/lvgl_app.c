#include "lvgl_app.h"
#include "ws2812.h"

#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "ui_animation.h"
#include "ui_feedback.h"
#include "ui_page.h"
#include "ui_perf_diag.h"
#include "ui_theme.h"
#include "dc_motor_ol.h"
#include "mjpeg_player.h"
#include "sd_start_anim.h"
#include "mecanum.h"
#include "main.h"
#include "lvgl.h"
#include "src/extra/libs/gif/gifdec.h"
#include "fatfs.h"
#include "ff.h"
#include "mpu6500.h"
#include "imu.h"
#include "imu_service.h"
#include "comm_service.h"
#include "safety_manager.h"
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static lv_obj_t *s_status_label = NULL;
static lv_obj_t *s_adc_label = NULL;
volatile uint8_t g_adc_update_flag = 0;
volatile float g_adc_voltage = 0.0f;
static lv_group_t *s_group = NULL;
static char s_status_text[640] = "Up/Down move, Right enter, Left back, OK play";
static uint32_t s_last_safety_faults = SAFETY_FAULT_NONE;

#define LVGL_APP_MAX_BROWSER_ENTRIES 48U
#define LVGL_APP_ENTRY_NAME_LEN      256U
#define LVGL_APP_BROWSER_PATH_LEN    512U
#define LVGL_APP_LVFS_PATH_LEN       (LVGL_APP_BROWSER_PATH_LEN + 3U)

#define LVGL_APP_MENU_ID_MANUAL      1U
#define LVGL_APP_MENU_ID_COMMAND     2U
#define LVGL_APP_MENU_ID_SD_BROWSER  3U
#define LVGL_APP_MENU_ID_MECANUM     4U
#define LVGL_APP_MENU_ID_MPU6500     5U
#define LVGL_APP_MENU_ID_WS2812      6U
#define LVGL_APP_MENU_ID_DIAGNOSTICS 7U
#define LVGL_APP_MOTOR_SUB_ID_BACK   0U
#define LVGL_APP_MOTOR_SUB_ID_SPEED  1U
#define LVGL_APP_MOTOR_SUB_ID_SERVO  2U
#define LVGL_APP_SD_ID_BACK          0U
#define LVGL_APP_SD_ID_UP            1U
#define LVGL_APP_SD_ID_BASE          2U

#define LVGL_APP_MOTOR_COUNT         4U
#define LVGL_APP_SERVO_COUNT         2U
#define LVGL_APP_SPEED_MIN           (-100)
#define LVGL_APP_SPEED_MAX           100
#define LVGL_APP_SPEED_STEP          10
#define LVGL_APP_SERVO_MIN           0
#define LVGL_APP_SERVO_MAX           270
#define LVGL_APP_SERVO_STEP          10
#define LVGL_APP_CTRL_REFRESH_MS     50U
#define LVGL_APP_MEDIA_FLUSH_WAIT_MS 250U

typedef enum
{
    LVGL_APP_ENTRY_DIR = 0,
    LVGL_APP_ENTRY_BIN,
    LVGL_APP_ENTRY_GIF,
    LVGL_APP_ENTRY_MJPEG,
    LVGL_APP_ENTRY_FILE
} lvgl_app_entry_type_t;

typedef struct
{
    char name[LVGL_APP_ENTRY_NAME_LEN];
    lvgl_app_entry_type_t type;
} lvgl_app_browser_entry_t;

typedef struct
{
    FIL fil;
} lvgl_app_lvfs_file_t;

typedef enum
{
    LVGL_APP_CTRL_PAGE_NONE = 0,
    LVGL_APP_CTRL_PAGE_MOTOR_SPEED,
    LVGL_APP_CTRL_PAGE_SERVO_ANGLE,
    LVGL_APP_CTRL_PAGE_COMMAND,
    LVGL_APP_CTRL_PAGE_MECANUM,
    LVGL_APP_CTRL_PAGE_MPU6500,
    LVGL_APP_CTRL_PAGE_WS2812
} lvgl_app_ctrl_page_t;

typedef enum
{
    LVGL_APP_SCREEN_REQ_NONE = 0,
    LVGL_APP_SCREEN_REQ_MAIN,
    LVGL_APP_SCREEN_REQ_MOTOR_MENU,
    LVGL_APP_SCREEN_REQ_MOTOR_SPEED,
    LVGL_APP_SCREEN_REQ_SERVO_ANGLE,
    LVGL_APP_SCREEN_REQ_COMMAND,
    LVGL_APP_SCREEN_REQ_SD_BROWSER,
    LVGL_APP_SCREEN_REQ_MECANUM,
    LVGL_APP_SCREEN_REQ_MPU6500,
    LVGL_APP_SCREEN_REQ_WS2812,
    LVGL_APP_SCREEN_REQ_DIAGNOSTICS,
    LVGL_APP_SCREEN_REQ_GIF
} lvgl_app_screen_req_t;

typedef enum
{
    LVGL_APP_ACTIVITY_NONE = 0,
    LVGL_APP_ACTIVITY_RUNNING,
    LVGL_APP_ACTIVITY_PAUSED,
    LVGL_APP_ACTIVITY_STORAGE,
    LVGL_APP_ACTIVITY_COMMAND,
    LVGL_APP_ACTIVITY_DIAGNOSTIC,
    LVGL_APP_ACTIVITY_FAULT
} lvgl_app_activity_t;

static uint16_t s_browser_entry_count = 0U;
static lvgl_app_browser_entry_t s_browser_entries[LVGL_APP_MAX_BROWSER_ENTRIES];
static char s_browser_path[LVGL_APP_BROWSER_PATH_LEN] = "/";
static FRESULT s_browser_scan_result = FR_OK;

static gd_GIF *s_gif = NULL;
static lv_obj_t *s_gif_obj = NULL;
static lv_timer_t *s_gif_timer = NULL;
static lv_img_dsc_t s_gif_imgdsc;
static uint32_t s_gif_last_call = 0U;
static uint16_t s_gif_current_frame = 0U;
static uint8_t s_gif_playing = 0U;
static uint8_t s_gif_paused = 0U;
static uint8_t s_gif_seek_key_latched = 0U;
static lv_obj_t *s_gif_control_card = NULL;
static lv_obj_t *s_gif_control_label = NULL;
static uint8_t s_key2_latched = 0U;
static uint8_t s_key3_latched = 0U;
static char s_gif_lvfs_path[LVGL_APP_LVFS_PATH_LEN] = "S:/";

static lv_fs_drv_t s_lvfs_drv;
static uint8_t s_lvfs_registered = 0U;

static lvgl_app_ctrl_page_t s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
static lvgl_app_screen_req_t s_pending_screen_req = LVGL_APP_SCREEN_REQ_NONE;
static lvgl_app_screen_req_t s_current_screen = LVGL_APP_SCREEN_REQ_NONE;
static ui_page_t s_page;
static ui_feedback_t s_feedback;
static lv_obj_t *s_page_root = NULL;
static lv_obj_t *s_page_content = NULL;
static ui_transition_manager_t s_transition_manager;
static uint8_t s_ctrl_selected_row = 0U;
static uint8_t s_ctrl_editing = 0U;
static int16_t s_motor_speed_preset[LVGL_APP_MOTOR_COUNT] = {0, 0, 0, 0};
static int16_t s_command_motor_speed_setpoint[LVGL_APP_MOTOR_COUNT] = {0, 0, 0, 0};
static int32_t s_motor_speed_actual[LVGL_APP_MOTOR_COUNT] = {0, 0, 0, 0};
static int32_t s_motor_speed_display[LVGL_APP_MOTOR_COUNT] = {0, 0, 0, 0};
static ui_value_follower_t s_motor_speed_followers[LVGL_APP_MOTOR_COUNT];
static int16_t s_servo_angle_preset[LVGL_APP_SERVO_COUNT] = {0, 0};
static int16_t s_mec_trans_x = 0;
static int16_t s_mec_trans_y = 0;
static int16_t s_mec_rot_z = 0;
static int16_t s_mec_speed_x = 0;
static int16_t s_mec_speed_y = 0;
static int16_t s_mec_speed_z = 0;
static uint8_t s_mecanum_executing = 0;
static lv_timer_t *s_mecanum_timer = NULL;

static lv_obj_t *s_ctrl_row_btns[7] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static lv_obj_t *s_ctrl_row_labels[7] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static lv_obj_t *s_ctrl_row_bars[7] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static uint8_t s_ctrl_row_visual_state[7] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static int16_t s_ctrl_row_bar_values[7] = {-1, -1, -1, -1, -1, -1, -1};
static int8_t s_ctrl_row_bar_directions[7] = {2, 2, 2, 2, 2, 2, 2};
static uint32_t s_ctrl_last_confirm_tick = 0U;
static uint32_t s_ctrl_last_actual_refresh_tick = 0U;
static lv_obj_t *s_diag_status_card = NULL;
static lv_obj_t *s_diag_page_label = NULL;
static lv_obj_t *s_diag_data_label = NULL;
static lv_obj_t *s_diag_data_panel = NULL;
static lv_obj_t *s_diag_chart = NULL;
static lv_chart_series_t *s_diag_fps_series = NULL;
static lv_chart_series_t *s_diag_heap_series = NULL;
static ui_perf_status_t s_diag_last_status = (ui_perf_status_t)0xFFU;
static uint8_t s_diag_page_index = 0U;
static uint32_t s_diag_last_refresh_tick = 0U;
static lvgl_app_activity_t s_header_activity = (lvgl_app_activity_t)0xFFU;
static uint32_t s_header_activity_tick = 0U;

static void lvgl_app_update_header_activity(void)
{
    lvgl_app_activity_t activity = LVGL_APP_ACTIVITY_NONE;
    const char *symbol = "";
    lv_color_t color = lv_color_white();
    uint8_t spin = 0U;
    uint8_t i;
    uint32_t now = HAL_GetTick();

    if ((now - s_header_activity_tick) < 50U)
    {
        return;
    }
    s_header_activity_tick = now;

    if ((s_page.status_icon == NULL) ||
        (lv_obj_is_valid(s_page.status_icon) == false))
    {
        return;
    }

    if (s_last_safety_faults != SAFETY_FAULT_NONE)
    {
        activity = LVGL_APP_ACTIVITY_FAULT;
        symbol = LV_SYMBOL_WARNING;
        color = lv_color_hex(0xFCA5A5);
    }
    else if (s_current_screen == LVGL_APP_SCREEN_REQ_GIF)
    {
        activity = (s_gif_paused != 0U) ?
                   LVGL_APP_ACTIVITY_PAUSED : LVGL_APP_ACTIVITY_RUNNING;
        symbol = (s_gif_paused != 0U) ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY;
        color = (s_gif_paused != 0U) ? lv_color_hex(0xFCD34D) : lv_color_hex(0x86EFAC);
        spin = 0U;
    }
    else if (s_current_screen == LVGL_APP_SCREEN_REQ_SD_BROWSER)
    {
        activity = LVGL_APP_ACTIVITY_STORAGE;
        symbol = LV_SYMBOL_SD_CARD;
        color = lv_color_hex(0x93C5FD);
    }
    else if (s_current_screen == LVGL_APP_SCREEN_REQ_COMMAND)
    {
        activity = LVGL_APP_ACTIVITY_COMMAND;
        symbol = LV_SYMBOL_USB;
        color = lv_color_hex(0x93C5FD);
    }
    else if (s_current_screen == LVGL_APP_SCREEN_REQ_DIAGNOSTICS)
    {
        activity = LVGL_APP_ACTIVITY_DIAGNOSTIC;
        symbol = LV_SYMBOL_EYE_OPEN;
        color = lv_color_hex(0xC4B5FD);
    }
    else if ((s_current_screen == LVGL_APP_SCREEN_REQ_MECANUM) &&
             ((s_mecanum_executing != 0U) || (Mecanum_IsMotionActive() != 0U)))
    {
        activity = LVGL_APP_ACTIVITY_RUNNING;
        symbol = LV_SYMBOL_REFRESH;
        color = lv_color_hex(0x86EFAC);
        spin = 1U;
    }
    else if (s_current_screen == LVGL_APP_SCREEN_REQ_MOTOR_SPEED)
    {
        for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
        {
            if ((s_motor_speed_actual[i] > 20) || (s_motor_speed_actual[i] < -20))
            {
                activity = LVGL_APP_ACTIVITY_RUNNING;
                symbol = LV_SYMBOL_REFRESH;
                color = lv_color_hex(0x86EFAC);
                spin = 1U;
                break;
            }
        }
    }

    if (activity == s_header_activity)
    {
        return;
    }

    s_header_activity = activity;
    UI_Anim_IconSpin(s_page.status_icon, 0U);
    if (activity == LVGL_APP_ACTIVITY_NONE)
    {
        lv_obj_add_flag(s_page.status_icon, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_label_set_text(s_page.status_icon, symbol);
    lv_obj_set_style_text_color(s_page.status_icon, color, LV_PART_MAIN);
    lv_obj_clear_flag(s_page.status_icon, LV_OBJ_FLAG_HIDDEN);
    UI_Anim_StateBounce(s_page.status_icon);
    UI_Anim_IconSpin(s_page.status_icon, spin);
}

static void lvgl_app_control_clear_row_refs(void)
{
    uint8_t i;

    for (i = 0U; i < 7U; ++i)
    {
        s_ctrl_row_btns[i] = NULL;
        s_ctrl_row_labels[i] = NULL;
        s_ctrl_row_bars[i] = NULL;
        s_ctrl_row_visual_state[i] = 0xFFU;
        s_ctrl_row_bar_values[i] = -1;
        s_ctrl_row_bar_directions[i] = 2;
    }
}

static void lvgl_app_show_main_menu(void);
static void lvgl_app_show_motor_control_menu(void);
static void lvgl_app_show_motor_speed_control(void);
static void lvgl_app_show_servo_angle_control(void);
static void lvgl_app_show_command_control(void);
static void lvgl_app_show_mecanum_control(void);
static void lvgl_app_show_sd_browser(void);
static void lvgl_app_show_mpu6500_data(void);
static void lvgl_app_show_ws2812_control(void);
static void lvgl_app_show_diagnostics(void);
static void lvgl_app_diagnostics_refresh(void);
static void lvgl_app_show_gif_player(const char *full_path, const char *name);
static void lvgl_app_exit_gif_player(const char *reason);
static void lvgl_app_motor_menu_event_cb(lv_event_t *e);
static void lvgl_app_control_event_cb(lv_event_t *e);
static void lvgl_app_request_screen(lvgl_app_screen_req_t req);
static void lvgl_app_process_pending_screen(void);

static uint8_t lvgl_app_screen_depth(lvgl_app_screen_req_t screen)
{
    if (screen == LVGL_APP_SCREEN_REQ_MAIN)
    {
        return 0U;
    }

    if ((screen == LVGL_APP_SCREEN_REQ_MOTOR_SPEED) ||
        (screen == LVGL_APP_SCREEN_REQ_SERVO_ANGLE) ||
        (screen == LVGL_APP_SCREEN_REQ_GIF))
    {
        return 2U;
    }

    return 1U;
}

static void lvgl_app_page_begin(lvgl_app_screen_req_t target, const char *title)
{
    ui_transition_t transition = UI_TRANSITION_NONE;
    lv_obj_t *outgoing = NULL;

    if ((s_current_screen != LVGL_APP_SCREEN_REQ_NONE) && (s_current_screen != target))
    {
        transition = (lvgl_app_screen_depth(target) < lvgl_app_screen_depth(s_current_screen))
                         ? UI_TRANSITION_BACKWARD
                         : UI_TRANSITION_FORWARD;
    }

    /* Finish an interrupted transition before allocating another layer. */
    UI_TransitionManager_Cancel(&s_transition_manager);

    if ((s_page.root == NULL) || (lv_obj_is_valid(s_page.root) == false))
    {
        UI_Feedback_Detach(&s_feedback);
        lv_obj_clean(lv_scr_act());
        if (UI_Page_Create(&s_page, lv_scr_act(), title) == 0U)
        {
            s_page_root = NULL;
            s_page_content = NULL;
            s_status_label = NULL;
            return;
        }

        UI_Feedback_Attach(&s_feedback, s_page.root, s_page.footer,
                           s_page.status_label);
        s_header_activity = (lvgl_app_activity_t)0xFFU;
    }
    else
    {
        outgoing = s_page_content;
        lv_label_set_text(s_page.title_label, title);
    }

    s_page_root = s_page.root;
    s_page_content = UI_Page_CreateContentLayer(&s_page);
    s_status_label = s_page.status_label;
    if (s_page_content == NULL)
    {
        return;
    }

    UI_Feedback_SetStatus(&s_feedback, s_status_text);
    UI_Feedback_SetFault(&s_feedback, s_last_safety_faults, s_status_text);
    s_current_screen = target;
    UI_TransitionManager_Prepare(&s_transition_manager, outgoing,
                                 s_page_content, transition);
}

static void lvgl_app_page_finish(void)
{
    if ((s_page_root != NULL) && (lv_obj_is_valid(s_page_root) != false) &&
        (s_page_content != NULL) && (lv_obj_is_valid(s_page_content) != false))
    {
        /* Resolve all child alignment before animating only the content region. */
        lv_obj_update_layout(s_page_root);
        UI_TransitionManager_Start(&s_transition_manager);
    }
}

static void lvgl_app_group_add_obj(lv_obj_t *obj)
{
    if ((s_group == NULL) || (obj == NULL))
    {
        return;
    }

    UI_Anim_AttachFocus(obj);
    lv_group_add_obj(s_group, obj);
}

static void lvgl_app_set_status(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
           (void)vsnprintf(s_status_text, sizeof(s_status_text), fmt, args);
    va_end(args);

    UI_Feedback_SetStatus(&s_feedback, s_status_text);
}

static void lvgl_app_show_toast(ui_notice_level_t level, const char *fmt, ...)
{
    va_list args;
    char text[128];

    va_start(args, fmt);
    (void)vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    UI_Feedback_ShowToast(&s_feedback, level, text,
                          (level == UI_NOTICE_ERROR) ? 2400U : 1500U);
}

static void lvgl_app_group_reset(void)
{
    lv_indev_t *keypad;

    if (s_group != NULL)
    {
        lv_group_del(s_group);
        s_group = NULL;
    }

    s_group = lv_group_create();
    lv_group_set_wrap(s_group, true);
    lv_group_set_editing(s_group, false);

    keypad = lv_port_indev_get_keypad();
    if (keypad != NULL)
    {
        lv_indev_set_group(keypad, s_group);
    }
}

static uint8_t lvgl_app_is_ext_file(const char *name, const char *ext3)
{
    size_t len;

    if ((name == NULL) || (ext3 == NULL))
    {
        return 0U;
    }

    len = strlen(name);
    if (len < 5U)
    {
        return 0U;
    }

    if (name[len - 4U] != '.')
    {
        return 0U;
    }

    if ((char)toupper((unsigned char)name[len - 3U]) != ext3[0])
    {
        return 0U;
    }

    if ((char)toupper((unsigned char)name[len - 2U]) != ext3[1])
    {
        return 0U;
    }

    if ((char)toupper((unsigned char)name[len - 1U]) != ext3[2])
    {
        return 0U;
    }

    return 1U;
}

static uint8_t lvgl_app_is_bin_file(const char *name)
{
    return lvgl_app_is_ext_file(name, "BIN");
}

static uint8_t lvgl_app_is_gif_file(const char *name)
{
    return lvgl_app_is_ext_file(name, "GIF");
}

static uint8_t lvgl_app_is_avi_file(const char *name)
{
    return lvgl_app_is_ext_file(name, "AVI");
}

static uint8_t lvgl_app_is_mjpeg_file(const char *name)
{
    size_t len;

    if (name == NULL)
    {
        return 0U;
    }

    len = strlen(name);
    if ((len >= 7U) && (name[len - 7U] == '.'))
    {
        if (((char)toupper((unsigned char)name[len - 6U]) == 'M') &&
            ((char)toupper((unsigned char)name[len - 5U]) == 'J') &&
            ((char)toupper((unsigned char)name[len - 4U]) == 'P') &&
            ((char)toupper((unsigned char)name[len - 3U]) == 'E') &&
            ((char)toupper((unsigned char)name[len - 2U]) == 'G'))
        {
            return 1U;
        }
    }

    if ((len >= 6U) && (name[len - 6U] == '.'))
    {
        if (((char)toupper((unsigned char)name[len - 5U]) == 'M') &&
            ((char)toupper((unsigned char)name[len - 4U]) == 'J') &&
            ((char)toupper((unsigned char)name[len - 3U]) == 'P') &&
            ((char)toupper((unsigned char)name[len - 2U]) == 'G'))
        {
            return 1U;
        }
    }

    return 0U;
}

static void lvgl_app_browser_reset_path(void)
{
    s_browser_path[0] = '/';
    s_browser_path[1] = '\0';
}

static uint8_t lvgl_app_browser_enter_dir(const char *name)
{
    size_t base_len;
    size_t name_len;

    if ((name == NULL) || (name[0] == '\0'))
    {
        return 0U;
    }

    base_len = strlen(s_browser_path);
    name_len = strlen(name);

    if (strcmp(s_browser_path, "/") == 0)
    {
        if ((1U + name_len + 1U) > sizeof(s_browser_path))
        {
            return 0U;
        }

        s_browser_path[0] = '/';
        (void)snprintf(&s_browser_path[1], sizeof(s_browser_path) - 1U, "%s", name);
    }
    else
    {
        if ((base_len + 1U + name_len + 1U) > sizeof(s_browser_path))
        {
            return 0U;
        }

        s_browser_path[base_len] = '/';
        s_browser_path[base_len + 1U] = '\0';
        (void)snprintf(&s_browser_path[base_len + 1U], sizeof(s_browser_path) - base_len - 1U, "%s", name);
    }

    return 1U;
}

static uint8_t lvgl_app_browser_go_parent(void)
{
    size_t len;

    if (strcmp(s_browser_path, "/") == 0)
    {
        return 0U;
    }

    len = strlen(s_browser_path);
    while ((len > 0U) && (s_browser_path[len - 1U] != '/'))
    {
        len--;
    }

    if (len <= 1U)
    {
        lvgl_app_browser_reset_path();
    }
    else
    {
        s_browser_path[len - 1U] = '\0';
    }

    return 1U;
}

static uint8_t lvgl_app_browser_make_file_path(const char *name, char *out, size_t out_size)
{
    int n;

    if ((name == NULL) || (out == NULL) || (out_size == 0U))
    {
        return 0U;
    }

    if (strcmp(s_browser_path, "/") == 0)
    {
        n = snprintf(out, out_size, "/%s", name);
    }
    else
    {
        n = snprintf(out, out_size, "%s/%s", s_browser_path, name);
    }

    if ((n <= 0) || ((size_t)n >= out_size))
    {
        return 0U;
    }

    return 1U;
}

static uint16_t lvgl_app_scan_browser_entries(void)
{
    DIR dir;
    FILINFO fno;
    FRESULT fr;

    s_browser_entry_count = 0U;
    s_browser_scan_result = FR_OK;

    fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (fr != FR_OK)
    {
        s_browser_scan_result = fr;
        lvgl_app_set_status("SD mount failed (%d)", (int)fr);
        return 0U;
    }

    fr = f_opendir(&dir, s_browser_path);
    if (fr == FR_OK)
    {
        while (1)
        {
            lvgl_app_entry_type_t file_type;

            fr = f_readdir(&dir, &fno);
            if (fr != FR_OK)
            {
                s_browser_scan_result = fr;
                lvgl_app_set_status("Read dir fail (%d)", (int)fr);
                break;
            }

            if (fno.fname[0] == '\0')
            {
                break;
            }

            if ((strcmp(fno.fname, ".") == 0) || (strcmp(fno.fname, "..") == 0))
            {
                continue;
            }

            if ((fno.fattrib & AM_DIR) != 0U)
            {
                file_type = LVGL_APP_ENTRY_DIR;
            }
            else if (lvgl_app_is_bin_file(fno.fname) != 0U)
            {
                file_type = LVGL_APP_ENTRY_BIN;
            }
            else if (lvgl_app_is_gif_file(fno.fname) != 0U)
            {
                file_type = LVGL_APP_ENTRY_GIF;
            }
            else if ((lvgl_app_is_avi_file(fno.fname) != 0U) || (lvgl_app_is_mjpeg_file(fno.fname) != 0U))
            {
                file_type = LVGL_APP_ENTRY_MJPEG;
            }
            else
            {
                file_type = LVGL_APP_ENTRY_FILE;
            }

            (void)snprintf(
                s_browser_entries[s_browser_entry_count].name,
                sizeof(s_browser_entries[s_browser_entry_count].name),
                "%s",
                fno.fname
            );
            s_browser_entries[s_browser_entry_count].type = file_type;
            s_browser_entry_count++;

            if (s_browser_entry_count >= LVGL_APP_MAX_BROWSER_ENTRIES)
            {
                lvgl_app_set_status("Too many files");
                break;
            }
        }

        (void)f_closedir(&dir);
    }
    else
    {
        s_browser_scan_result = fr;
        lvgl_app_set_status("Open dir fail (%d)", (int)fr);
    }

    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return s_browser_entry_count;
}

static uint32_t lvgl_app_event_get_key(lv_event_t *e)
{
    const uint32_t *key_param;

    key_param = (const uint32_t *)lv_event_get_param(e);
    if (key_param == NULL)
    {
        return 0U;
    }

    return *key_param;
}

static lv_fs_res_t lvgl_app_fs_res_from_fr(FRESULT fr)
{
    switch (fr)
    {
        case FR_OK:
            return LV_FS_RES_OK;
        case FR_NO_FILE:
        case FR_NO_PATH:
            return LV_FS_RES_NOT_EX;
        case FR_DENIED:
            return LV_FS_RES_DENIED;
        default:
            return LV_FS_RES_UNKNOWN;
    }
}

static void *lvgl_app_fs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
    lvgl_app_lvfs_file_t *file_ctx;
    BYTE ff_mode;
    FRESULT fr;

    (void)drv;

    ff_mode = FA_READ;
    if ((mode & LV_FS_MODE_WR) != 0U)
    {
        ff_mode |= FA_WRITE;
    }

    file_ctx = (lvgl_app_lvfs_file_t *)lv_mem_alloc(sizeof(*file_ctx));
    if (file_ctx == NULL)
    {
        return NULL;
    }

    fr = f_open(&file_ctx->fil, path, ff_mode);
    if (fr != FR_OK)
    {
        lv_mem_free(file_ctx);
        return NULL;
    }

    return file_ctx;
}

static lv_fs_res_t lvgl_app_fs_close(lv_fs_drv_t *drv, void *file_p)
{
    lvgl_app_lvfs_file_t *file_ctx = (lvgl_app_lvfs_file_t *)file_p;
    FRESULT fr;

    (void)drv;

    if (file_ctx == NULL)
    {
        return LV_FS_RES_INV_PARAM;
    }

    fr = f_close(&file_ctx->fil);
    lv_mem_free(file_ctx);
    return lvgl_app_fs_res_from_fr(fr);
}

static lv_fs_res_t lvgl_app_fs_read(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br)
{
    lvgl_app_lvfs_file_t *file_ctx = (lvgl_app_lvfs_file_t *)file_p;
    UINT read_len = 0U;
    FRESULT fr;

    (void)drv;

    if ((file_ctx == NULL) || (buf == NULL) || (br == NULL))
    {
        return LV_FS_RES_INV_PARAM;
    }

    fr = f_read(&file_ctx->fil, buf, (UINT)btr, &read_len);
    *br = (uint32_t)read_len;
    return lvgl_app_fs_res_from_fr(fr);
}

static lv_fs_res_t lvgl_app_fs_seek(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence)
{
    lvgl_app_lvfs_file_t *file_ctx = (lvgl_app_lvfs_file_t *)file_p;
    DWORD target;
    DWORD size;
    DWORD current;
    FRESULT fr;

    (void)drv;

    if (file_ctx == NULL)
    {
        return LV_FS_RES_INV_PARAM;
    }

    current = f_tell(&file_ctx->fil);
    size = f_size(&file_ctx->fil);

    if (whence == LV_FS_SEEK_SET)
    {
        target = (DWORD)pos;
    }
    else if (whence == LV_FS_SEEK_CUR)
    {
        target = (DWORD)(current + (DWORD)pos);
    }
    else
    {
        target = (DWORD)(size + (DWORD)pos);
    }

    fr = f_lseek(&file_ctx->fil, target);
    return lvgl_app_fs_res_from_fr(fr);
}

static lv_fs_res_t lvgl_app_fs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p)
{
    lvgl_app_lvfs_file_t *file_ctx = (lvgl_app_lvfs_file_t *)file_p;

    (void)drv;

    if ((file_ctx == NULL) || (pos_p == NULL))
    {
        return LV_FS_RES_INV_PARAM;
    }

    *pos_p = (uint32_t)f_tell(&file_ctx->fil);
    return LV_FS_RES_OK;
}

static void lvgl_app_fs_init(void)
{
    if (s_lvfs_registered != 0U)
    {
        return;
    }

    lv_fs_drv_init(&s_lvfs_drv);
    s_lvfs_drv.letter = 'S';
    s_lvfs_drv.cache_size = 0U;
    s_lvfs_drv.open_cb = lvgl_app_fs_open;
    s_lvfs_drv.close_cb = lvgl_app_fs_close;
    s_lvfs_drv.read_cb = lvgl_app_fs_read;
    s_lvfs_drv.seek_cb = lvgl_app_fs_seek;
    s_lvfs_drv.tell_cb = lvgl_app_fs_tell;
    lv_fs_drv_register(&s_lvfs_drv);

    s_lvfs_registered = 1U;
}

static void lvgl_app_motor_speed_send_cmd(uint8_t motor_index, int16_t speed)
{
    Mecanum_CancelControl();
    DCMotor_OL_RequestSpeed(motor_index, speed);
}

static void lvgl_app_motor_speed_sync_actual(void)
{
    uint8_t i;

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        s_motor_speed_actual[i] = DCMotor_OL_GetSpeedRpm((uint8_t)(i + 1U));
    }
}

static void lvgl_app_motor_speed_force_clear_all(void)
{
    uint8_t i;

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        s_motor_speed_preset[i] = 0;
        lvgl_app_motor_speed_send_cmd((uint8_t)(i + 1U), 0);
    }
}

extern TIM_HandleTypeDef htim8;

static void lvgl_app_servo_angle_send_cmd(uint8_t servo_index, int16_t angle)
{
    uint32_t ccr;
    if (angle < LVGL_APP_SERVO_MIN) { angle = LVGL_APP_SERVO_MIN; }
    if (angle > LVGL_APP_SERVO_MAX) { angle = LVGL_APP_SERVO_MAX; }

    /* 0 degrees -> 500us, 270 degrees -> 2500us. Timer resolution: 1us */
    ccr = 500U + ((uint32_t)angle * 2000U) / 270U;

    if (servo_index == 1U)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ccr);
    }
    else if (servo_index == 2U)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ccr);
    }
}

static void lvgl_app_request_screen(lvgl_app_screen_req_t req)
{
    s_pending_screen_req = req;
}

static void lvgl_app_process_pending_screen(void)
{
    lvgl_app_screen_req_t req;

    req = s_pending_screen_req;
    s_pending_screen_req = LVGL_APP_SCREEN_REQ_NONE;

    if (req == LVGL_APP_SCREEN_REQ_MAIN)
    {
        lvgl_app_show_main_menu();
    }
    else if (req == LVGL_APP_SCREEN_REQ_MOTOR_MENU)
    {
        lvgl_app_show_motor_control_menu();
    }
    else if (req == LVGL_APP_SCREEN_REQ_MOTOR_SPEED)
    {
        lvgl_app_show_motor_speed_control();
    }
    else if (req == LVGL_APP_SCREEN_REQ_SERVO_ANGLE)
    {
        lvgl_app_show_servo_angle_control();
    }
    else if (req == LVGL_APP_SCREEN_REQ_SD_BROWSER)
    {
        lvgl_app_show_sd_browser();
    }
    else if (req == LVGL_APP_SCREEN_REQ_COMMAND)
    {
        lvgl_app_show_command_control();
    }
    else if (req == LVGL_APP_SCREEN_REQ_MECANUM)
    {
        lvgl_app_show_mecanum_control();
    }
    else if (req == LVGL_APP_SCREEN_REQ_MPU6500)
    {
        lvgl_app_show_mpu6500_data();
    }
    else if (req == LVGL_APP_SCREEN_REQ_WS2812)
    {
        lvgl_app_show_ws2812_control();
    }
    else if (req == LVGL_APP_SCREEN_REQ_DIAGNOSTICS)
    {
        lvgl_app_show_diagnostics();
    }
}

static lv_obj_t *s_cmd_ctrl_label = NULL;

static int8_t s_joy_lx = 0;
static int8_t s_joy_ly = 0;
static int8_t s_joy_rx = 0;
static int8_t s_joy_ry = 0;

uint8_t LVGL_App_IsCommandControlActive(void)
{
    return (s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND) ? 1U : 0U;
}

uint8_t LVGL_App_CommandSetMotorSpeed(uint8_t motor_index, int16_t speed_percent)
{
    if ((LVGL_App_IsCommandControlActive() == 0U) ||
        (motor_index == 0U) || (motor_index > LVGL_APP_MOTOR_COUNT))
    {
        return 0U;
    }

    if (speed_percent < LVGL_APP_SPEED_MIN)
    {
        speed_percent = LVGL_APP_SPEED_MIN;
    }
    else if (speed_percent > LVGL_APP_SPEED_MAX)
    {
        speed_percent = LVGL_APP_SPEED_MAX;
    }

    s_command_motor_speed_setpoint[motor_index - 1U] = speed_percent;
    lvgl_app_motor_speed_send_cmd(motor_index, speed_percent);
    s_ctrl_last_actual_refresh_tick = 0U;
    return 1U;
}

void LVGL_App_CommandStopMotors(void)
{
    uint8_t i;

    if (LVGL_App_IsCommandControlActive() == 0U)
    {
        return;
    }

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        s_command_motor_speed_setpoint[i] = 0;
    }
    s_joy_lx = 0;
    s_joy_ly = 0;
    s_joy_rx = 0;
    s_joy_ry = 0;

    Mecanum_MixedControl(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DCMotor_OL_RequestStopAll();
    s_ctrl_last_actual_refresh_tick = 0U;
}

static void lvgl_app_control_refresh_rows(void)
{
    uint8_t i;
    char line[64];
    lv_obj_t *row_label;
    lv_obj_t *row_btn;
    uint8_t mecanum_active = 0U;

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND)
    {
        if (s_cmd_ctrl_label != NULL)
        {
            char big_buf[256];
            snprintf(big_buf, sizeof(big_buf),
                     "M1 Set: %+4d, Act: %+5ld\n"
                     "M2 Set: %+4d, Act: %+5ld\n"
                     "M3 Set: %+4d, Act: %+5ld\n"
                     "M4 Set: %+4d, Act: %+5ld\n"
                     "Sv Set: %3d, %3d\n"
                     "Joy: L(%+3d,%+3d) R(%+3d,%+3d)",
                     s_command_motor_speed_setpoint[0], s_motor_speed_display[0],
                     s_command_motor_speed_setpoint[1], s_motor_speed_display[1],
                     s_command_motor_speed_setpoint[2], s_motor_speed_display[2],
                     s_command_motor_speed_setpoint[3], s_motor_speed_display[3],
                     s_servo_angle_preset[0], s_servo_angle_preset[1],
                     s_joy_lx, s_joy_ly, s_joy_rx, s_joy_ry);
            (void)UI_LabelSetTextIfChanged(s_cmd_ctrl_label, big_buf);
        }
        return;
    }

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM)
    {
        mecanum_active = ((s_mecanum_executing != 0U) ||
                          (Mecanum_IsMotionActive() != 0U)) ? 1U : 0U;
    }

    uint8_t loop_count = LVGL_APP_MOTOR_COUNT;
    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM) {
        loop_count = 7U;
    } else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE) {
        loop_count = LVGL_APP_SERVO_COUNT;
    }
    
    for (i = 0U; i < loop_count; ++i)
    {
        ui_visual_state_t visual_state;
        int32_t signed_bar_value = 0;
        int32_t bar_max = 100;
        int32_t bar_percent;

        row_label = s_ctrl_row_labels[i];
        row_btn = s_ctrl_row_btns[i];

        if ((row_label == NULL) || (lv_obj_is_valid(row_label) == false))
        {
            s_ctrl_row_labels[i] = NULL;
            s_ctrl_row_btns[i] = NULL;
            continue;
        }

        if ((row_btn != NULL) && (lv_obj_is_valid(row_btn) == false))
        {
            row_btn = NULL;
            s_ctrl_row_btns[i] = NULL;
        }

        visual_state = UI_VISUAL_NORMAL;
        if (s_last_safety_faults != SAFETY_FAULT_NONE)
        {
            visual_state = UI_VISUAL_FAULT;
        }
        else if ((i == s_ctrl_selected_row) && (s_ctrl_editing != 0U))
        {
            visual_state = UI_VISUAL_EDITING;
        }
        else if ((s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM) &&
                 (i == 6U) && (mecanum_active != 0U))
        {
            visual_state = UI_VISUAL_RUNNING;
        }
        else if (i == s_ctrl_selected_row)
        {
            visual_state = UI_VISUAL_FOCUSED;
        }
        else if ((s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED) &&
                 ((s_motor_speed_preset[i] != 0) ||
                  (s_motor_speed_actual[i] > 20) || (s_motor_speed_actual[i] < -20)))
        {
            visual_state = UI_VISUAL_RUNNING;
        }

        if ((row_btn != NULL) &&
            (s_ctrl_row_visual_state[i] != (uint8_t)visual_state))
        {
            s_ctrl_row_visual_state[i] = (uint8_t)visual_state;
            UI_Theme_SetVisualState(row_btn, row_label, visual_state);
            UI_Anim_SetFocusColor(row_btn, UI_Theme_GetVisualColor(visual_state));
        }

        if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
        {
            signed_bar_value = s_motor_speed_preset[i];
        }
        else if ((s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM) && (i < 6U))
        {
            if (i == 0U) { signed_bar_value = s_mec_trans_x; bar_max = 200; }
            else if (i == 1U) { signed_bar_value = s_mec_trans_y; bar_max = 200; }
            else if (i == 2U) { signed_bar_value = s_mec_rot_z; bar_max = 360; }
            else if (i == 3U) { signed_bar_value = s_mec_speed_x; }
            else if (i == 4U) { signed_bar_value = s_mec_speed_y; }
            else { signed_bar_value = s_mec_speed_z; }
        }

        if ((s_ctrl_row_bars[i] != NULL) &&
            (lv_obj_is_valid(s_ctrl_row_bars[i]) != false))
        {
            int8_t bar_direction;
            int32_t magnitude = (signed_bar_value < 0) ? -signed_bar_value : signed_bar_value;

            if ((s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM) &&
                (visual_state == UI_VISUAL_FAULT))
            {
                magnitude = 0;
            }
            if (magnitude > bar_max) { magnitude = bar_max; }
            bar_percent = (magnitude * 100) / bar_max;

            bar_direction = (signed_bar_value > 0) ? 1 :
                            ((signed_bar_value < 0) ? -1 : 0);
            if (s_ctrl_row_bar_values[i] != (int16_t)bar_percent)
            {
                s_ctrl_row_bar_values[i] = (int16_t)bar_percent;
                UI_Anim_SetBarValue(s_ctrl_row_bars[i], bar_percent);
            }
            if (s_ctrl_row_bar_directions[i] != bar_direction)
            {
                s_ctrl_row_bar_directions[i] = bar_direction;
                if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM)
                {
                    UI_Theme_SetValueFillColor(s_ctrl_row_bars[i], signed_bar_value);
                }
                else
                {
                    UI_Theme_SetValueBarColor(s_ctrl_row_bars[i], signed_bar_value);
                }
            }
        }

        if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
        {
            (void)snprintf(
                line,
                sizeof(line),
                "M%u  Set:%+4d%%  Act:%+5ld",
                (unsigned int)(i + 1U),
                (int)s_motor_speed_preset[i],
                (long)s_motor_speed_display[i]
            );
        }
        else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
        {
            (void)snprintf(
                line,
                sizeof(line),
                "S%u  Angle:%3d deg",
                (unsigned int)(i + 1U),
                (int)s_servo_angle_preset[i]
            );
        }
        else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM)
        {
            if (i == 0)
                (void)snprintf(line, sizeof(line), "X Dist(cm)  %+5d", (int)s_mec_trans_x);
            else if (i == 1)
                (void)snprintf(line, sizeof(line), "Y Dist(cm)  %+5d", (int)s_mec_trans_y);
            else if (i == 2)
                (void)snprintf(line, sizeof(line), "Z Rot(deg)  %+5d", (int)s_mec_rot_z);
            else if (i == 3)
                (void)snprintf(line, sizeof(line), "X Speed     %5d", (int)s_mec_speed_x);
            else if (i == 4)
                (void)snprintf(line, sizeof(line), "Y Speed     %5d", (int)s_mec_speed_y);
            else if (i == 5)
                (void)snprintf(line, sizeof(line), "Z Speed     %5d", (int)s_mec_speed_z);
            else if (i == 6)
                (void)snprintf(line, sizeof(line), "[ %s ]", mecanum_active ? "STOP ACTIVE" : "EXECUTE");
        }
        else
        {
            line[0] = '\0';
        }

        (void)UI_LabelSetTextIfChanged(row_label, line);
    }
}

static void mecanum_start_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    s_mecanum_timer = NULL;
    if (s_mecanum_executing)
    {
        lvgl_app_set_status("Mecanum EXECUTING");
        lvgl_app_show_toast(UI_NOTICE_SUCCESS, "Mecanum executing");
        
        float vx = (float)s_mec_speed_x * 10.0f; // Input was cm/s, convert to mm/s
        float vy = (float)s_mec_speed_y * 10.0f; // Input was cm/s, convert to mm/s
        float wz = (float)s_mec_speed_z;         // Rotational speed was presumably deg/s

        if (s_mec_trans_x == 0 && s_mec_trans_y == 0 && s_mec_rot_z == 0)
        {
            Mecanum_MixedControl(vx, vy, wz, 0.0f, 0.0f, 0.0f);
        }
        else
        {
            float dx = (float)s_mec_trans_x * 10.0f; // Input was cm, convert to mm
            float dy = (float)s_mec_trans_y * 10.0f; // Input was cm, convert to mm
            float dw = (float)s_mec_rot_z;           // Input was degrees
            
            Mecanum_MixedControl(vx, vy, wz, dx, dy, dw);
            s_mecanum_executing = 0;
            lvgl_app_control_refresh_rows();
        }
    }
}

static void lvgl_app_control_confirm_selected(void)
{
    if (s_ctrl_editing == 0U)
    {
        if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM && s_ctrl_selected_row == 6)
        {
            if ((s_mecanum_executing != 0U) || (Mecanum_IsMotionActive() != 0U))
            {
                s_mecanum_executing = 0;
                Mecanum_MixedControl(0, 0, 0, 0, 0, 0);
                lvgl_app_set_status("Mecanum STOPPED");
                lvgl_app_show_toast(UI_NOTICE_WARNING, "Mecanum stopped");
                if (s_mecanum_timer)
                {
                    lv_timer_del(s_mecanum_timer);
                    s_mecanum_timer = NULL;
                }
            }
            else
            {
                s_mecanum_executing = 1;
                lvgl_app_set_status("Mecanum Wait 3s...");
                lvgl_app_show_toast(UI_NOTICE_INFO, "Starting in 3 seconds");
                if (s_mecanum_timer == NULL)
                {
                    extern void mecanum_start_timer_cb(lv_timer_t *timer); // will define below
                    s_mecanum_timer = lv_timer_create(mecanum_start_timer_cb, 3000, NULL);
                    lv_timer_set_repeat_count(s_mecanum_timer, 1);
                }
            }
            lvgl_app_control_refresh_rows();
            return;
        }

        s_ctrl_editing = 1U;
        if (s_group != NULL)
        {
            lv_group_set_editing(s_group, true);
            lv_group_focus_freeze(s_group, true);
        }

        if (s_ctrl_row_btns[s_ctrl_selected_row] != NULL)
        {
            if (lv_obj_is_valid(s_ctrl_row_btns[s_ctrl_selected_row]) != false)
            {
                lv_group_focus_obj(s_ctrl_row_btns[s_ctrl_selected_row]);
            }
        }

        if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
        {
            lvgl_app_set_status("M%u: Left/Right adjust speed, OK send, KEY2 exit edit", (unsigned int)(s_ctrl_selected_row + 1U));
        }
        else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
        {
            lvgl_app_set_status("S%u: Left/Right adjust angle, OK send, KEY2 exit edit", (unsigned int)(s_ctrl_selected_row + 1U));
        }
    }
    else
    {
        if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
        {
            lvgl_app_motor_speed_send_cmd((uint8_t)(s_ctrl_selected_row + 1U), s_motor_speed_preset[s_ctrl_selected_row]);
            lvgl_app_set_status(
                "M%u sent: %d (KEY2 exit edit)",
                (unsigned int)(s_ctrl_selected_row + 1U),
                (int)s_motor_speed_preset[s_ctrl_selected_row]
            );
            lvgl_app_show_toast(UI_NOTICE_SUCCESS, "M%u speed command sent",
                                (unsigned int)(s_ctrl_selected_row + 1U));
        }
        else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
        {
            lvgl_app_servo_angle_send_cmd((uint8_t)(s_ctrl_selected_row + 1U), s_servo_angle_preset[s_ctrl_selected_row]);
            lvgl_app_set_status(
                "S%u sent: %d (KEY2 exit edit)",
                (unsigned int)(s_ctrl_selected_row + 1U),
                (int)s_servo_angle_preset[s_ctrl_selected_row]
            );
            lvgl_app_show_toast(UI_NOTICE_SUCCESS, "S%u angle command sent",
                                (unsigned int)(s_ctrl_selected_row + 1U));
        }
    }

    lvgl_app_control_refresh_rows();
}

static void lvgl_app_control_exit_edit_mode(void)
{
    if (s_ctrl_editing == 0U)
    {
        return;
    }

    s_ctrl_editing = 0U;
    if (s_group != NULL)
    {
        lv_group_focus_freeze(s_group, false);
        lv_group_set_editing(s_group, false);
    }

    if (s_ctrl_row_btns[s_ctrl_selected_row] != NULL)
    {
        if (lv_obj_is_valid(s_ctrl_row_btns[s_ctrl_selected_row]) != false)
        {
            lv_group_focus_obj(s_ctrl_row_btns[s_ctrl_selected_row]);
        }
    }

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
    {
        lvgl_app_set_status("Exit speed edit, use Up/Down to change motor, OK to edit");
    }
    else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
    {
        lvgl_app_set_status("Exit servo edit, use Up/Down to change servo, OK to edit");
    }

    lvgl_app_control_refresh_rows();
}

static void lvgl_app_control_back_to_motor_menu(void)
{
    if (s_group != NULL)
    {
        lv_group_focus_freeze(s_group, false);
        lv_group_set_editing(s_group, false);
    }

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
    {
        lvgl_app_motor_speed_force_clear_all();
    }
    
    uint8_t previous_page = s_ctrl_page;

    lvgl_app_control_clear_row_refs();
    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;

    if (previous_page == LVGL_APP_CTRL_PAGE_MECANUM)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
    }
    else
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MOTOR_MENU);
    }
}

static void lvgl_app_control_adjust_selected(int8_t direction)
{
    int16_t value;

    if (s_ctrl_editing == 0U)
    {
        return;
    }

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
    {
        value = (int16_t)(s_motor_speed_preset[s_ctrl_selected_row] + (int16_t)(direction * LVGL_APP_SPEED_STEP));
        if (value < LVGL_APP_SPEED_MIN)
        {
            value = LVGL_APP_SPEED_MIN;
        }
        else if (value > LVGL_APP_SPEED_MAX)
        {
            value = LVGL_APP_SPEED_MAX;
        }

        s_motor_speed_preset[s_ctrl_selected_row] = value;
    }
    else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
    {
        value = (int16_t)(s_servo_angle_preset[s_ctrl_selected_row] + (int16_t)(direction * LVGL_APP_SERVO_STEP));
        if (value < LVGL_APP_SERVO_MIN)
        {
            value = LVGL_APP_SERVO_MIN;
        }
        else if (value > LVGL_APP_SERVO_MAX)
        {
            value = LVGL_APP_SERVO_MAX;
        }

        s_servo_angle_preset[s_ctrl_selected_row] = value;
    }
    else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM)
    {
        if (s_ctrl_selected_row == 0)
        {
            value = s_mec_trans_x + direction * 10;
            s_mec_trans_x = value;
        }
        else if (s_ctrl_selected_row == 1)
        {
            value = s_mec_trans_y + direction * 10;
            s_mec_trans_y = value;
        }
        else if (s_ctrl_selected_row == 2)
        {
            value = s_mec_rot_z + direction * 10;
            s_mec_rot_z = value;
        }
        else if (s_ctrl_selected_row == 3)
        {
            value = s_mec_speed_x + direction * 10;
            if (value < 0) value = 0;
            s_mec_speed_x = value;
        }
        else if (s_ctrl_selected_row == 4)
        {
            value = s_mec_speed_y + direction * 10;
            if (value < 0) value = 0;
            s_mec_speed_y = value;
        }
        else if (s_ctrl_selected_row == 5)
        {
            value = s_mec_speed_z + direction * 10;
            if (value < 0) value = 0;
            s_mec_speed_z = value;
        }
    }

    lvgl_app_control_refresh_rows();
    if ((s_ctrl_selected_row < 7U) &&
        (s_ctrl_row_labels[s_ctrl_selected_row] != NULL) &&
        (lv_obj_is_valid(s_ctrl_row_labels[s_ctrl_selected_row]) != false))
    {
        UI_Anim_PulseOpacity(s_ctrl_row_labels[s_ctrl_selected_row]);
    }
}

static void lvgl_app_motor_speed_reset_followers(void)
{
    uint8_t i;

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        UI_ValueFollower_Reset(&s_motor_speed_followers[i], 0);
        s_motor_speed_display[i] = 0;
    }
}

static void lvgl_app_motor_speed_update_followers(uint32_t now)
{
    uint8_t i;

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        UI_ValueFollower_SetTarget(&s_motor_speed_followers[i],
                                   s_motor_speed_actual[i]);
        s_motor_speed_display[i] = UI_ValueFollower_Update(
            &s_motor_speed_followers[i], now, 160U);
    }
}

static void lvgl_app_control_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uint32_t key;
    uint8_t row;
    uint32_t now;

    code = lv_event_get_code(e);
    row = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    uint8_t max_rows = (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM) ? 7U : (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE ? LVGL_APP_SERVO_COUNT : LVGL_APP_MOTOR_COUNT);
    if (row >= max_rows)
    {
        row = s_ctrl_selected_row;
    }

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_NONE)
    {
        return;
    }

    if (code == LV_EVENT_FOCUSED)
    {
        if (s_ctrl_editing == 0U)
        {
            s_ctrl_selected_row = row;
            lvgl_app_control_refresh_rows();

            if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
            {
                lvgl_app_set_status("Motor M%u selected, OK to edit", (unsigned int)(s_ctrl_selected_row + 1U));
            }
            else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
            {
                lvgl_app_set_status("Servo S%u selected, OK to edit", (unsigned int)(s_ctrl_selected_row + 1U));
            }
        }
        return;
    }

    if (code == LV_EVENT_CLICKED)
    {
        now = HAL_GetTick();
        if ((now - s_ctrl_last_confirm_tick) < 120U)
        {
            return;
        }

        s_ctrl_last_confirm_tick = now;
        s_ctrl_selected_row = row;
        lvgl_app_control_confirm_selected();
        return;
    }

    if (code != LV_EVENT_KEY)
    {
        return;
    }

    key = lvgl_app_event_get_key(e);
    if (s_ctrl_editing == 0U)
    {
        if (key == LV_KEY_LEFT)
        {
            lvgl_app_control_back_to_motor_menu();
            return;
        }

        if (key == LV_KEY_ESC)
        {
            lvgl_app_control_back_to_motor_menu();
            return;
        }

        /* Remainder handled by LV_EVENT_CLICKED */

        return;
    }

    if ((key == LV_KEY_UP) || (key == LV_KEY_PREV) || (key == LV_KEY_DOWN) || (key == LV_KEY_NEXT))
    {
        return;
    }

    if (key == LV_KEY_LEFT)
    {
        lvgl_app_control_adjust_selected(-1);
        return;
    }

    if (key == LV_KEY_RIGHT)
    {
        lvgl_app_control_adjust_selected(1);
        return;
    }

    if (key == LV_KEY_ENTER)
    {
        now = HAL_GetTick();
        if ((now - s_ctrl_last_confirm_tick) >= 120U)
        {
            s_ctrl_last_confirm_tick = now;
            lvgl_app_control_confirm_selected();
        }
        return;
    }

    if (key == LV_KEY_ESC)
    {
        lvgl_app_control_exit_edit_mode();
    }
}

static void lvgl_app_show_motor_speed_control(void)
{
    lv_obj_t *row_btn;
    uint8_t i;

    // Set WS2812 to 20% Red
    ws2812_set_all(rgb_to_color(51, 0, 0));
    ws2812_update();

    s_ctrl_page = LVGL_APP_CTRL_PAGE_MOTOR_SPEED;
    s_ctrl_selected_row = 0U;
    s_ctrl_editing = 0U;
    s_ctrl_last_confirm_tick = 0U;
    lvgl_app_control_clear_row_refs();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_MOTOR_SPEED, "Motor Speed");

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        row_btn = lv_btn_create(s_page_content);
        s_ctrl_row_btns[i] = row_btn;
        lv_obj_set_size(row_btn, 220, 38);
        lv_obj_align(row_btn, LV_ALIGN_TOP_MID, 0, 3 + (lv_coord_t)i * 43);
        UI_Theme_ApplyDataCard(row_btn);
        lv_obj_clear_flag(row_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_FOCUSED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)i);

        s_ctrl_row_labels[i] = lv_label_create(row_btn);
        lv_obj_set_style_text_font(s_ctrl_row_labels[i], &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align(s_ctrl_row_labels[i], LV_ALIGN_TOP_LEFT, 8, 3);

        s_ctrl_row_bars[i] = lv_bar_create(row_btn);
        lv_bar_set_range(s_ctrl_row_bars[i], 0, 100);
        lv_obj_set_size(s_ctrl_row_bars[i], 202, 5);
        lv_obj_align(s_ctrl_row_bars[i], LV_ALIGN_BOTTOM_MID, 0, -4);
        UI_Theme_ApplyValueBar(s_ctrl_row_bars[i]);

        lvgl_app_group_add_obj(row_btn);
        UI_Anim_StaggerIn(row_btn, i);
    }

    if (s_ctrl_row_btns[0] != NULL)
    {
        lv_group_focus_obj(s_ctrl_row_btns[0]);
    }
    lv_group_set_editing(s_group, false);

    lvgl_app_motor_speed_sync_actual();
    lvgl_app_motor_speed_reset_followers();
    s_ctrl_last_actual_refresh_tick = HAL_GetTick();
    lvgl_app_control_refresh_rows();
    lvgl_app_set_status("Up/Down select | OK edit/send | KEY2 cancel | Left back");
    lvgl_app_page_finish();
}

static void lvgl_app_show_servo_angle_control(void)
{
    lv_obj_t *header;
    lv_obj_t *row_btn;
    uint8_t i;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_SERVO_ANGLE;
    s_ctrl_selected_row = 0U;
    s_ctrl_editing = 0U;
    s_ctrl_last_confirm_tick = 0U;
    lvgl_app_control_clear_row_refs();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_SERVO_ANGLE, "Servo Angle");

    header = lv_label_create(s_page_content);
    lv_label_set_text(header, "ID   ANGLE");
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 8);

    for (i = 0U; i < LVGL_APP_SERVO_COUNT; ++i)
    {
        row_btn = lv_btn_create(s_page_content);
        s_ctrl_row_btns[i] = row_btn;
        lv_obj_set_size(row_btn, 220, 22);
        lv_obj_align(row_btn, LV_ALIGN_TOP_MID, 0, 38 + (lv_coord_t)i * 34);
        UI_Theme_ApplyControlRow(row_btn);
        lv_obj_clear_flag(row_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_FOCUSED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)i);

        s_ctrl_row_labels[i] = lv_label_create(row_btn);
        lv_obj_set_style_pad_left(s_ctrl_row_labels[i], 6, 0);
        lv_obj_set_style_pad_right(s_ctrl_row_labels[i], 6, 0);
        lv_obj_set_style_pad_top(s_ctrl_row_labels[i], 2, 0);
        lv_obj_set_style_pad_bottom(s_ctrl_row_labels[i], 2, 0);
        lv_obj_set_style_radius(s_ctrl_row_labels[i], 4, 0);
        lv_obj_align(s_ctrl_row_labels[i], LV_ALIGN_LEFT_MID, 0, 0);

        lvgl_app_group_add_obj(row_btn);
        UI_Anim_StaggerIn(row_btn, i);
    }

    if (s_ctrl_row_btns[0] != NULL)
    {
        lv_group_focus_obj(s_ctrl_row_btns[0]);
    }
    lv_group_set_editing(s_group, false);

    lvgl_app_control_refresh_rows();
    lvgl_app_set_status("Up/Down select | OK edit/send | KEY2 cancel | Left back");
    lvgl_app_page_finish();
}

static void lvgl_app_motor_menu_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uintptr_t id;
    uint32_t key;

    code = lv_event_get_code(e);
    id = (uintptr_t)lv_event_get_user_data(e);

    if (code == LV_EVENT_KEY)
    {
        key = lvgl_app_event_get_key(e);
        if ((key == LV_KEY_ESC) || (key == LV_KEY_LEFT))
        {
            lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
            return;
        }

        if (key != LV_KEY_RIGHT)
        {
            return;
        }
    }
    else if (code != LV_EVENT_CLICKED)
    {
        return;
    }

    if (id == LVGL_APP_MOTOR_SUB_ID_SPEED)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MOTOR_SPEED);
    }
    else if (id == LVGL_APP_MOTOR_SUB_ID_SERVO)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_SERVO_ANGLE);
    }
    else
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
    }
}

static void lvgl_app_show_motor_control_menu(void)
{
    lv_obj_t *list;
    lv_obj_t *btn;
    lv_obj_t *first_btn;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_control_clear_row_refs();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_MOTOR_MENU, "Motor Control");

    list = lv_list_create(s_page_content);
    lv_obj_set_size(list, 224, 160);
    lv_obj_center(list);
    UI_Theme_ApplyPanel(list);

    btn = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "1 Motor Speed");
    first_btn = btn;
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SPEED);
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SPEED);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 0U);

    btn = lv_list_add_btn(list, LV_SYMBOL_REFRESH, "2 Servo Angle");
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SERVO);
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SERVO);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 1U);

    btn = lv_list_add_btn(list, LV_SYMBOL_LEFT, "Back to Main Menu");
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_BACK);
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_BACK);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 2U);
    lv_group_focus_obj(first_btn);

    lvgl_app_set_status("Motor first, Left to go back");
    lvgl_app_page_finish();
}

static void lvgl_app_sd_enter_dir_by_index(uint16_t index)
{
    uint8_t enter_ok;
    char prev_path[LVGL_APP_BROWSER_PATH_LEN];

    (void)snprintf(prev_path, sizeof(prev_path), "%s", s_browser_path);
    enter_ok = lvgl_app_browser_enter_dir(s_browser_entries[index].name);
    if (enter_ok == 0U)
    {
        lvgl_app_set_status("Path too long");
        (void)snprintf(s_browser_path, sizeof(s_browser_path), "%s", prev_path);
    }
    else
    {
        lvgl_app_set_status("Path: %s", s_browser_path);
    }

    lvgl_app_show_sd_browser();
}

static void lvgl_app_gif_render_current(void)
{
    if ((s_gif == NULL) || (s_gif_obj == NULL))
    {
        return;
    }

    gd_render_frame(s_gif, (uint8_t *)s_gif_imgdsc.data);
    lv_img_cache_invalidate_src(&s_gif_imgdsc);
    lv_obj_invalidate(s_gif_obj);
}

static void lvgl_app_gif_advance_once(void)
{
    int has_next;

    if (s_gif == NULL)
    {
        return;
    }

    has_next = gd_get_frame(s_gif);
    if (has_next == 0)
    {
        if (s_gif->loop_count > 1U)
        {
            s_gif->loop_count--;
            gd_rewind(s_gif);
            has_next = gd_get_frame(s_gif);
        }
    }

    if (has_next >= 0)
    {
        s_gif_current_frame++;
        lvgl_app_gif_render_current();
    }
}

static void lvgl_app_gif_rewind_to(uint16_t target_frame)
{
    uint16_t i;

    if (s_gif == NULL)
    {
        return;
    }

    gd_rewind(s_gif);
    s_gif_current_frame = 0U;

    for (i = 0U; i < target_frame; ++i)
    {
        lvgl_app_gif_advance_once();
    }
}

static void lvgl_app_gif_seek_relative(int32_t step_frames)
{
    uint16_t target_frame;

    if ((s_gif == NULL) || (step_frames == 0))
    {
        return;
    }

    if (step_frames < 0)
    {
        if ((uint32_t)s_gif_current_frame <= (uint32_t)(-step_frames))
        {
            target_frame = 0U;
        }
        else
        {
            target_frame = (uint16_t)(s_gif_current_frame + step_frames);
        }

        lvgl_app_gif_rewind_to(target_frame);
    }
    else
    {
        uint32_t i;

        for (i = 0U; i < (uint32_t)step_frames; ++i)
        {
            lvgl_app_gif_advance_once();
        }
    }

    s_gif_last_call = lv_tick_get();
}

static void lvgl_app_gif_timer_cb(lv_timer_t *timer)
{
    uint32_t delay_ms;
    uint8_t left_pressed;
    uint8_t right_pressed;

    (void)timer;

    left_pressed = (HAL_GPIO_ReadPin(Key_Left_GPIO_Port, Key_Left_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    right_pressed = (HAL_GPIO_ReadPin(Key_Right_GPIO_Port, Key_Right_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if ((left_pressed == 0U) && (right_pressed == 0U))
    {
        if (s_gif_seek_key_latched != 0U)
        {
            s_gif_last_call = lv_tick_get();
        }
        s_gif_seek_key_latched = 0U;
    }
    else if (((left_pressed != 0U) && (right_pressed == 0U)) ||
             ((right_pressed != 0U) && (left_pressed == 0U)))
    {
        /* LVGL emits the initial/repeat seek events; freeze auto advance here. */
        s_gif_seek_key_latched = 1U;
        return;
    }

    if ((s_gif_playing == 0U) || (s_gif_paused != 0U) || (s_gif == NULL))
    {
        return;
    }

    delay_ms = (uint32_t)s_gif->gce.delay * 10U;
    if (delay_ms == 0U)
    {
        delay_ms = 10U;
    }

    if (lv_tick_elaps(s_gif_last_call) < delay_ms)
    {
        return;
    }

    s_gif_last_call = lv_tick_get();
    lvgl_app_gif_advance_once();
}

static uint8_t lvgl_app_probe_gif_open(const char *full_path, char *reason, size_t reason_size)
{
    FIL file;
    uint8_t header[13];
    UINT read_len;
    FRESULT fr;
    uint16_t width;
    uint16_t height;
    uint8_t packed;
    uint32_t estimated_alloc;

    if ((full_path == NULL) || (full_path[0] == '\0') || (reason == NULL) || (reason_size == 0U))
    {
        return 0U;
    }

    fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (fr != FR_OK)
    {
        (void)snprintf(reason, reason_size, "GIF mount failed (%d)", (int)fr);
        return 0U;
    }

    fr = f_open(&file, full_path, FA_READ);
    if (fr != FR_OK)
    {
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(reason, reason_size, "GIF open failed (%d)", (int)fr);
        return 0U;
    }

    fr = f_read(&file, header, sizeof(header), &read_len);
    if ((fr != FR_OK) || (read_len != sizeof(header)))
    {
        (void)f_close(&file);
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(reason, reason_size, "GIF header read failed");
        return 0U;
    }

    if (memcmp(header, "GIF", 3) != 0)
    {
        (void)f_close(&file);
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(reason, reason_size, "GIF signature invalid");
        return 0U;
    }

    if (memcmp(&header[3], "89a", 3) != 0)
    {
        (void)f_close(&file);
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(reason, reason_size, "GIF version must be 89a");
        return 0U;
    }

    width = (uint16_t)((uint16_t)header[6] | ((uint16_t)header[7] << 8));
    height = (uint16_t)((uint16_t)header[8] | ((uint16_t)header[9] << 8));
    packed = header[10];

    if ((width == 0U) || (height == 0U))
    {
        (void)f_close(&file);
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(reason, reason_size, "GIF size invalid");
        return 0U;
    }

    if ((packed & 0x80U) == 0U)
    {
        (void)f_close(&file);
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(reason, reason_size, "GIF no global color table");
        return 0U;
    }

#if LV_COLOR_DEPTH == 32
    estimated_alloc = (uint32_t)sizeof(gd_GIF) + (5U * (uint32_t)width * (uint32_t)height);
#elif LV_COLOR_DEPTH == 16
    estimated_alloc = (uint32_t)sizeof(gd_GIF) + (4U * (uint32_t)width * (uint32_t)height);
#elif LV_COLOR_DEPTH == 8 || LV_COLOR_DEPTH == 1
    estimated_alloc = (uint32_t)sizeof(gd_GIF) + (3U * (uint32_t)width * (uint32_t)height);
#else
    estimated_alloc = 0U;
#endif

#if LV_MEM_CUSTOM == 0
    if ((estimated_alloc != 0U) && (estimated_alloc > (uint32_t)LV_MEM_SIZE))
    {
        (void)f_close(&file);
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        (void)snprintf(
            reason,
            reason_size,
            "GIF frame buffer too large: %luB > heap %luB",
            (unsigned long)estimated_alloc,
            (unsigned long)LV_MEM_SIZE
        );
        return 0U;
    }
#endif

    (void)f_close(&file);
    (void)snprintf(reason, reason_size, "GIF OK: %ux%u", (unsigned int)width, (unsigned int)height);
    return 1U;
}

static void lvgl_app_sd_play_bin_by_index(uint16_t index)
{
    int8_t play_status;
    char play_path[LVGL_APP_BROWSER_PATH_LEN];
    char played_name[LVGL_APP_ENTRY_NAME_LEN];

    if (lvgl_app_browser_make_file_path(s_browser_entries[index].name, play_path, sizeof(play_path)) == 0U)
    {
          lvgl_app_set_status("Failed to build path");
        lvgl_app_show_sd_browser();
        return;
    }

    (void)snprintf(played_name, sizeof(played_name), "%s", s_browser_entries[index].name);
    lvgl_app_set_status("BIN: OK pause, Left/Right seek, KEY3 return");
    lv_refr_now(NULL);
    (void)lv_port_disp_wait_idle(LVGL_APP_MEDIA_FLUSH_WAIT_MS);

    play_status = SD_StartAnim_PlayFile(play_path);
    if ((play_status == SD_START_ANIM_ERR_BACK) ||
        (play_status == SD_START_ANIM_ERR_STOPPED))
    {
        lv_port_indev_suppress_exit_keys_until_release();
    }

    lvgl_app_show_sd_browser();
    if (play_status == SD_START_ANIM_OK)
    {
          lvgl_app_set_status("Done: %s", played_name);
          lvgl_app_show_toast(UI_NOTICE_SUCCESS, "Playback complete");
    }
    else if (play_status == SD_START_ANIM_ERR_BACK)
    {
          lvgl_app_set_status("Returned by KEY3");
          lvgl_app_show_toast(UI_NOTICE_INFO, "Returned by KEY3");
    }
    else if (play_status == SD_START_ANIM_ERR_STOPPED)
    {
          lvgl_app_set_status("Stopped by KEY2");
          lvgl_app_show_toast(UI_NOTICE_WARNING, "Stopped by KEY2");
    }
    else
    {
          lvgl_app_set_status("Failed (%d): %s", (int)play_status, played_name);
          lvgl_app_show_toast(UI_NOTICE_ERROR, "BIN playback failed (%d)", (int)play_status);
    }
}

static void lvgl_app_gif_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uint32_t key;

    code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        s_gif_paused = (s_gif_paused == 0U) ? 1U : 0U;
        s_gif_last_call = lv_tick_get();
        if ((s_gif_control_label != NULL) &&
            (lv_obj_is_valid(s_gif_control_label) != false))
        {
            lv_label_set_text(s_gif_control_label,
                              (s_gif_paused != 0U) ?
                              "|<<    OK Play    >>|" :
                              "|<<   OK Pause    >>|");
        }
        if ((s_gif_control_card != NULL) &&
            (lv_obj_is_valid(s_gif_control_card) != false))
        {
            lv_obj_set_style_bg_color(
                s_gif_control_card,
                lv_color_hex((s_gif_paused != 0U) ? 0xFFF2CC : 0xE8F1FF),
                LV_PART_MAIN);
            UI_Anim_StateBounce(s_gif_control_card);
        }
        lvgl_app_set_status((s_gif_paused != 0U) ?
                            "Paused - OK plays, KEY3 returns" :
                            "Playing - OK pauses, Left/Right seek");
        return;
    }

    if (code == LV_EVENT_KEY)
    {
        key = lvgl_app_event_get_key(e);
        if (key == LV_KEY_LEFT)
        {
            lvgl_app_gif_seek_relative(-10);
            return;
        }

        if (key == LV_KEY_RIGHT)
        {
            lvgl_app_gif_seek_relative(10);
            return;
        }

        if (key == LV_KEY_ESC)
        {
            lvgl_app_exit_gif_player("GIF stopped");
        }
    }
}

static void lvgl_app_show_gif_player(const char *full_path, const char *name)
{
    lv_obj_t *ctrl_btn;
    char gif_probe_reason[96];
    uint32_t zoom;
    uint32_t fit_zoom;
    int n;
    FRESULT fr;

    if ((full_path == NULL) || (full_path[0] == '\0'))
    {
        lvgl_app_set_status("GIF path invalid");
        lvgl_app_show_sd_browser();
        return;
    }

    lvgl_app_fs_init();

    fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (fr != FR_OK)
    {
        lvgl_app_set_status("SD mount failed (%d)", (int)fr);
        lvgl_app_show_sd_browser();
        return;
    }

    if (full_path[0] == '/')
    {
        n = snprintf(s_gif_lvfs_path, sizeof(s_gif_lvfs_path), "S:%s", full_path);
    }
    else
    {
        n = snprintf(s_gif_lvfs_path, sizeof(s_gif_lvfs_path), "S:/%s", full_path);
    }

    if ((n <= 0) || ((size_t)n >= sizeof(s_gif_lvfs_path)))
    {
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        lvgl_app_set_status("GIF path too long");
        lvgl_app_show_sd_browser();
        return;
    }

    if (lvgl_app_probe_gif_open(full_path, gif_probe_reason, sizeof(gif_probe_reason)) == 0U)
    {
        lvgl_app_set_status("%s", gif_probe_reason);
        lvgl_app_show_sd_browser();
        return;
    }

    s_gif = gd_open_gif_file(s_gif_lvfs_path);
    if (s_gif == NULL)
    {
        (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
        lvgl_app_set_status("GIF open failed (decode/memory)");
        lvgl_app_show_sd_browser();
        return;
    }

    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_control_clear_row_refs();
    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_GIF, "Media Player");

    s_gif_obj = lv_img_create(s_page_content);
    s_gif_imgdsc.header.always_zero = 0;
    s_gif_imgdsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    s_gif_imgdsc.header.w = s_gif->width;
    s_gif_imgdsc.header.h = s_gif->height;
    s_gif_imgdsc.data = s_gif->canvas;
    lv_img_set_src(s_gif_obj, &s_gif_imgdsc);
    zoom = 256U;
    if (s_gif->width > 236U)
    {
        zoom = (236U * 256U) / s_gif->width;
    }
    if (s_gif->height > 146U)
    {
        fit_zoom = (146U * 256U) / s_gif->height;
        if (fit_zoom < zoom)
        {
            zoom = fit_zoom;
        }
    }
    if (zoom == 0U)
    {
        zoom = 1U;
    }
    lv_img_set_zoom(s_gif_obj, (uint16_t)zoom);
    lv_obj_align(s_gif_obj, LV_ALIGN_TOP_MID, 0, 2);

    s_gif_control_card = lv_obj_create(s_page_content);
    lv_obj_set_size(s_gif_control_card, 224, 26);
    lv_obj_align(s_gif_control_card, LV_ALIGN_BOTTOM_MID, 0, -2);
    UI_Theme_ApplyDataCard(s_gif_control_card);
    lv_obj_set_style_bg_color(s_gif_control_card, lv_color_hex(0xE8F1FF), LV_PART_MAIN);
    lv_obj_clear_flag(s_gif_control_card, LV_OBJ_FLAG_SCROLLABLE);

    s_gif_control_label = lv_label_create(s_gif_control_card);
    lv_label_set_text(s_gif_control_label, "|<<   OK Pause    >>|");
    lv_obj_set_style_text_font(s_gif_control_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_gif_control_label, lv_color_hex(0x173B67), LV_PART_MAIN);
    lv_obj_center(s_gif_control_label);

    /* Invisible focus target keeps encoder/key events without covering media. */
    ctrl_btn = lv_btn_create(s_page_content);
    lv_obj_set_size(ctrl_btn, 1, 1);
    lv_obj_set_style_opa(ctrl_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(ctrl_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(ctrl_btn, lvgl_app_gif_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ctrl_btn, lvgl_app_gif_event_cb, LV_EVENT_KEY, NULL);
    lv_group_add_obj(s_group, ctrl_btn);
    lv_group_focus_obj(ctrl_btn);

    if (name != NULL)
    {
        lvgl_app_set_status("Playing %s - OK pauses", name);
    }
    else
    {
        lvgl_app_set_status("Playing GIF");
    }

    s_gif_playing = 1U;
    s_gif_paused = 0U;
    s_gif_seek_key_latched = 0U;
    s_key3_latched = (HAL_GPIO_ReadPin(Key3_GPIO_Port, Key3_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    s_key2_latched = (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    s_gif_current_frame = 0U;
    s_gif_last_call = lv_tick_get();
    s_gif_timer = lv_timer_create(lvgl_app_gif_timer_cb, 10U, NULL);
    lvgl_app_page_finish();
}

static void lvgl_app_exit_gif_player(const char *reason)
{
    if (s_gif_playing == 0U)
    {
        return;
    }

    s_gif_playing = 0U;
    s_gif_paused = 0U;
    s_gif_seek_key_latched = 0U;
    s_gif_control_card = NULL;
    s_gif_control_label = NULL;

    if (s_gif_timer != NULL)
    {
        lv_timer_del(s_gif_timer);
        s_gif_timer = NULL;
    }

    if (s_gif_obj != NULL)
    {
        lv_obj_del(s_gif_obj);
        s_gif_obj = NULL;
    }

    if (s_gif != NULL)
    {
        gd_close_gif(s_gif);
        s_gif = NULL;
    }

    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);

    if (reason != NULL)
    {
        lvgl_app_set_status("%s", reason);
    }

    lvgl_app_show_sd_browser();
    if (reason != NULL)
    {
        lvgl_app_show_toast(UI_NOTICE_INFO, "%s", reason);
    }
}

static void lvgl_app_sd_play_gif_by_index(uint16_t index)
{
    char play_path[LVGL_APP_BROWSER_PATH_LEN];

    if (lvgl_app_browser_make_file_path(s_browser_entries[index].name, play_path, sizeof(play_path)) == 0U)
    {
            lvgl_app_set_status("Failed to build path");
        lvgl_app_show_sd_browser();
        return;
    }

    lvgl_app_show_gif_player(play_path, s_browser_entries[index].name);
}

static void lvgl_app_sd_play_mjpeg_by_index(uint16_t index)
{
    int8_t play_status;
    char play_path[LVGL_APP_BROWSER_PATH_LEN];
    char played_name[LVGL_APP_ENTRY_NAME_LEN];

    if (lvgl_app_browser_make_file_path(s_browser_entries[index].name, play_path, sizeof(play_path)) == 0U)
    {
          lvgl_app_set_status("Failed to build path");
        lvgl_app_show_sd_browser();
        return;
    }

    (void)snprintf(played_name, sizeof(played_name), "%s", s_browser_entries[index].name);
    lvgl_app_set_status("Video: OK pause, Left/Right seek, KEY3 return");
    lv_refr_now(NULL);
    (void)lv_port_disp_wait_idle(LVGL_APP_MEDIA_FLUSH_WAIT_MS);

    play_status = MJPEG_Player_PlayFile(play_path);
    if ((play_status == MJPEG_PLAYER_ERR_BACK) ||
        (play_status == MJPEG_PLAYER_ERR_STOPPED))
    {
        lv_port_indev_suppress_exit_keys_until_release();
    }
    lvgl_app_show_sd_browser();

    if (s_browser_scan_result != FR_OK)
    {
        lvgl_app_set_status(
            "MJPEG %d, SD scan failed (%d)",
            (int)play_status,
            (int)s_browser_scan_result
        );
        lvgl_app_show_toast(UI_NOTICE_ERROR, "SD scan failed (%d)",
                            (int)s_browser_scan_result);
        return;
    }

    if (play_status == MJPEG_PLAYER_OK)
    {
          lvgl_app_set_status("Done: %s", played_name);
          lvgl_app_show_toast(UI_NOTICE_SUCCESS, "Playback complete");
    }
    else if (play_status == MJPEG_PLAYER_ERR_BACK)
    {
          lvgl_app_set_status("Returned by KEY3");
          lvgl_app_show_toast(UI_NOTICE_INFO, "Returned by KEY3");
    }
    else if (play_status == MJPEG_PLAYER_ERR_STOPPED)
    {
          lvgl_app_set_status("Stopped by KEY2");
          lvgl_app_show_toast(UI_NOTICE_WARNING, "Stopped by KEY2");
    }
    else
    {
          lvgl_app_set_status(
              "MJPEG failed (%d, fs=%u): %s",
              (int)play_status,
              (unsigned int)MJPEG_Player_GetLastFsError(),
              played_name
          );
          lvgl_app_show_toast(UI_NOTICE_ERROR, "MJPEG playback failed (%d)",
                              (int)play_status);
    }
}

static void lvgl_app_sd_select_id(uintptr_t id)
{
    uint16_t index;

    if (id == LVGL_APP_SD_ID_BACK)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
        return;
    }

    if (id == LVGL_APP_SD_ID_UP)
    {
        if (lvgl_app_browser_go_parent() == 0U)
        {
            lvgl_app_set_status("Already at root directory");
        }
        lvgl_app_show_sd_browser();
        return;
    }

    index = (uint16_t)(id - LVGL_APP_SD_ID_BASE);
    if (index >= s_browser_entry_count)
    {
        return;
    }

    if (s_browser_entries[index].type == LVGL_APP_ENTRY_DIR)
    {
        lvgl_app_sd_enter_dir_by_index(index);
    }
    else if (s_browser_entries[index].type == LVGL_APP_ENTRY_BIN)
    {
        lvgl_app_sd_play_bin_by_index(index);
    }
    else if (s_browser_entries[index].type == LVGL_APP_ENTRY_GIF)
    {
        lvgl_app_sd_play_gif_by_index(index);
    }
    else if (s_browser_entries[index].type == LVGL_APP_ENTRY_MJPEG)
    {
        lvgl_app_sd_play_mjpeg_by_index(index);
    }
    else if (s_browser_entries[index].type == LVGL_APP_ENTRY_FILE)
    {
        lvgl_app_set_status("File: %s", s_browser_entries[index].name);
    }
}

static void lvgl_app_sd_left_action(void)
{
    if (lvgl_app_browser_go_parent() != 0U)
    {
        lvgl_app_show_sd_browser();
        return;
    }

    lvgl_app_set_status("Back to main menu");
    lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
}

static void lvgl_app_sd_right_action(uintptr_t id)
{
    uint16_t index;

    if (id < LVGL_APP_SD_ID_BASE)
    {
        return;
    }

    index = (uint16_t)(id - LVGL_APP_SD_ID_BASE);
    if (index >= s_browser_entry_count)
    {
        return;
    }

    if (s_browser_entries[index].type == LVGL_APP_ENTRY_DIR)
    {
        lvgl_app_sd_enter_dir_by_index(index);
    }
    else
    {
        lvgl_app_set_status("Right key enters folders only");
    }
}

static void lvgl_app_menu_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uintptr_t id;
    uint32_t key;

    code = lv_event_get_code(e);
    if (code == LV_EVENT_KEY)
    {
        key = lvgl_app_event_get_key(e);
        if (key == LV_KEY_ESC)
        {
            lvgl_app_set_status("Main menu");
            lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
            return;
        }

        if (key != LV_KEY_RIGHT)
        {
            return;
        }
    }
    else if (code != LV_EVENT_CLICKED)
    {
        return;
    }

    id = (uintptr_t)lv_event_get_user_data(e);
    if (id == LVGL_APP_MENU_ID_MANUAL)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MOTOR_MENU);
    }
    else if (id == LVGL_APP_MENU_ID_COMMAND)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_COMMAND);
    }
    else if (id == LVGL_APP_MENU_ID_SD_BROWSER)
    {
        lvgl_app_browser_reset_path();
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_SD_BROWSER);
    }
    else if (id == LVGL_APP_MENU_ID_MECANUM)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MECANUM);
    }
    else if (id == LVGL_APP_MENU_ID_MPU6500)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MPU6500);
    }
    else if (id == LVGL_APP_MENU_ID_WS2812)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_WS2812);
    }
    else if (id == LVGL_APP_MENU_ID_DIAGNOSTICS)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_DIAGNOSTICS);
    }
}

static void lvgl_app_sd_file_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uintptr_t id;
    uint32_t key;

    code = lv_event_get_code(e);
    id = (uintptr_t)lv_event_get_user_data(e);

    if (code == LV_EVENT_KEY)
    {
        key = lvgl_app_event_get_key(e);

        if (key == LV_KEY_ESC)
        {
            lvgl_app_set_status("Global exit");
            lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
            return;
        }

        if (key == LV_KEY_LEFT)
        {
            lvgl_app_sd_left_action();
            return;
        }

        if (key == LV_KEY_RIGHT)
        {
            lvgl_app_sd_right_action(id);
            return;
        }

        return;
    }
    else if (code != LV_EVENT_CLICKED)
    {
        return;
    }

    lvgl_app_sd_select_id(id);
}

static void lvgl_app_diagnostics_refresh(void)
{
    ui_perf_snapshot_t snapshot;
    const char *page_name;
    const char *status_text;
    lv_color_t status_bg;
    char text_buf[320];

    if ((s_diag_page_label == NULL) || (s_diag_data_label == NULL) ||
        (lv_obj_is_valid(s_diag_page_label) == false) ||
        (lv_obj_is_valid(s_diag_data_label) == false))
    {
        return;
    }

    UI_PerfDiag_GetSnapshot(&snapshot);
    if ((s_diag_chart != NULL) &&
        (lv_obj_is_valid(s_diag_chart) != false) &&
        (s_diag_fps_series != NULL) && (s_diag_heap_series != NULL))
    {
        lv_chart_set_next_value(s_diag_chart, s_diag_fps_series,
                                (snapshot.refresh_fps > 100U) ? 100 :
                                (lv_coord_t)snapshot.refresh_fps);
        lv_chart_set_next_value(s_diag_chart, s_diag_heap_series,
                                (lv_coord_t)snapshot.lv_mem_used_pct);
    }
    if (snapshot.status == UI_PERF_STATUS_OVERLOAD)
    {
        status_text = "OVERLOAD";
        status_bg = lv_color_hex(0xFDE8E8);
    }
    else if (snapshot.status == UI_PERF_STATUS_BUSY)
    {
        status_text = "BUSY";
        status_bg = lv_color_hex(0xFFF2CC);
    }
    else
    {
        status_text = "GOOD";
        status_bg = lv_color_hex(0xE4F7EA);
    }

    if (s_diag_page_index == 0U)
    {
        page_name = "1/3  OVERVIEW";
        (void)snprintf(
            text_buf,
            sizeof(text_buf),
            "FPS                 %5u\n"
            "Refresh avg/max %3u.%1u/%3u ms\n"
            "UI loop avg/max %5u/%5u us\n"
            "Pixels avg/max %6lu/%6lu\n"
            "Control max       %6lu us\n"
            "State               %s",
            (unsigned int)snapshot.refresh_fps,
            (unsigned int)(snapshot.refresh_avg_ms_x10 / 10U),
            (unsigned int)(snapshot.refresh_avg_ms_x10 % 10U),
            (unsigned int)snapshot.refresh_max_ms,
            (unsigned int)snapshot.ui_handler_avg_us,
            (unsigned int)snapshot.ui_handler_max_us,
            (unsigned long)snapshot.pixels_avg,
            (unsigned long)snapshot.pixels_max,
            (unsigned long)snapshot.control_max_us,
            status_text);
    }
    else if (s_diag_page_index == 1U)
    {
        page_name = "2/3  DISPLAY";
        (void)snprintf(
            text_buf,
            sizeof(text_buf),
            "Refresh rate        %5u fps\n"
            "Refresh max         %5u ms\n"
            "Flush avg/max   %5u/%5u us\n"
            "Flush count       %7lu\n"
            "Wait / timeout %6lu/%4lu\n"
            "Transfer errors   %7lu",
            (unsigned int)snapshot.refresh_fps,
            (unsigned int)snapshot.refresh_max_ms,
            (unsigned int)snapshot.flush_avg_us,
            (unsigned int)snapshot.flush_max_us,
            (unsigned long)snapshot.flush_count,
            (unsigned long)snapshot.flush_wait_count,
            (unsigned long)snapshot.flush_timeout_count,
            (unsigned long)snapshot.flush_error_count);
    }
    else
    {
        page_name = "3/3  MEMORY";
        (void)snprintf(
            text_buf,
            sizeof(text_buf),
            "LV heap used          %3u %%\n"
            "Free bytes          %7lu\n"
            "Largest block       %7lu\n"
            "Fragmentation         %3u %%\n"
            "Peak used           %7lu\n"
            "Control overruns    %7lu",
            (unsigned int)snapshot.lv_mem_used_pct,
            (unsigned long)snapshot.lv_mem_free,
            (unsigned long)snapshot.lv_mem_biggest_free,
            (unsigned int)snapshot.lv_mem_frag_pct,
            (unsigned long)snapshot.lv_mem_max_used,
            (unsigned long)snapshot.control_overrun_count);
    }

    (void)UI_LabelSetTextIfChanged(s_diag_page_label, page_name);
    (void)UI_LabelSetTextIfChanged(s_diag_data_label, text_buf);
    if ((s_diag_status_card != NULL) &&
        (lv_obj_is_valid(s_diag_status_card) != false))
    {
        lv_obj_set_style_bg_color(s_diag_status_card, status_bg, LV_PART_MAIN);
        if (s_diag_last_status != snapshot.status)
        {
            s_diag_last_status = snapshot.status;
            UI_Anim_StateBounce(s_diag_status_card);
        }
    }
}

static void lvgl_app_diagnostics_event_cb(lv_event_t *e)
{
    uint32_t key;

    if (lv_event_get_code(e) != LV_EVENT_KEY)
    {
        return;
    }

    key = lvgl_app_event_get_key(e);
    if (key == LV_KEY_RIGHT)
    {
        s_diag_page_index = (uint8_t)((s_diag_page_index + 1U) % 3U);
        lvgl_app_diagnostics_refresh();
        UI_Anim_CarouselIn(s_diag_data_panel, 1);
    }
    else if (key == LV_KEY_LEFT)
    {
        s_diag_page_index = (s_diag_page_index == 0U) ? 2U : (uint8_t)(s_diag_page_index - 1U);
        lvgl_app_diagnostics_refresh();
        UI_Anim_CarouselIn(s_diag_data_panel, -1);
    }
    else if (key == LV_KEY_ESC)
    {
        lv_port_indev_suppress_all_keys_until_release();
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
    }
}

static void lvgl_app_show_diagnostics(void)
{
    lv_obj_t *key_receiver;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_control_clear_row_refs();
    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_DIAGNOSTICS, "UI Diagnostics");

    s_diag_status_card = lv_obj_create(s_page_content);
    lv_obj_set_size(s_diag_status_card, 220, 28);
    lv_obj_align(s_diag_status_card, LV_ALIGN_TOP_MID, 0, 3);
    UI_Theme_ApplyDataCard(s_diag_status_card);
    lv_obj_clear_flag(s_diag_status_card, LV_OBJ_FLAG_SCROLLABLE);

    s_diag_page_label = lv_label_create(s_diag_status_card);
    lv_obj_set_style_text_font(s_diag_page_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_diag_page_label, lv_color_hex(0x173B67), LV_PART_MAIN);
    lv_obj_center(s_diag_page_label);
    UI_Anim_StaggerIn(s_diag_status_card, 0U);

    s_diag_data_panel = lv_obj_create(s_page_content);
    lv_obj_set_size(s_diag_data_panel, 220, 142);
    lv_obj_align(s_diag_data_panel, LV_ALIGN_BOTTOM_MID, 0, -3);
    UI_Theme_ApplyPanel(s_diag_data_panel);
    lv_obj_clear_flag(s_diag_data_panel, LV_OBJ_FLAG_SCROLLABLE);
    UI_Anim_StaggerIn(s_diag_data_panel, 1U);

    s_diag_data_label = lv_label_create(s_diag_data_panel);
    lv_obj_set_style_text_font(s_diag_data_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_diag_data_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_align(s_diag_data_label, LV_ALIGN_TOP_LEFT, 7, 8);

    s_diag_chart = lv_chart_create(s_diag_data_panel);
    lv_obj_set_size(s_diag_chart, 204, 40);
    lv_obj_align(s_diag_chart, LV_ALIGN_BOTTOM_MID, 0, -3);
    lv_obj_set_style_bg_opa(s_diag_chart, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_diag_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_line_width(s_diag_chart, 1, LV_PART_ITEMS);
    lv_chart_set_type(s_diag_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_diag_chart, 24U);
    lv_chart_set_range(s_diag_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_div_line_count(s_diag_chart, 2U, 4U);
    s_diag_fps_series = lv_chart_add_series(s_diag_chart,
                                            lv_color_hex(0x2563EB),
                                            LV_CHART_AXIS_PRIMARY_Y);
    s_diag_heap_series = lv_chart_add_series(s_diag_chart,
                                             lv_color_hex(0xF59E0B),
                                             LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_clear_flag(s_diag_chart, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    key_receiver = lv_obj_create(s_page_content);
    lv_obj_set_size(key_receiver, 1, 1);
    lv_obj_set_style_opa(key_receiver, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(key_receiver, 0, LV_PART_MAIN);
    lv_obj_clear_flag(key_receiver, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(key_receiver, lvgl_app_diagnostics_event_cb, LV_EVENT_KEY, NULL);
    lv_group_add_obj(s_group, key_receiver);
    lv_group_focus_obj(key_receiver);

    s_diag_page_index = 0U;
    s_diag_last_status = (ui_perf_status_t)0xFFU;
    s_diag_last_refresh_tick = HAL_GetTick();
    lvgl_app_diagnostics_refresh();
    lvgl_app_set_status("Left/Right pages - KEY2/KEY3 exits");
    lvgl_app_page_finish();
}

static void lvgl_app_show_main_menu(void)
{
    lv_obj_t *list;
    lv_obj_t *btn;
    lv_obj_t *first_btn;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_control_clear_row_refs();

    /* 恢复自动彩虹模式，并清空 WS2812 颜色 */
    g_ws2812_manual_mode = 0U;
    ws2812_set_all(0);
    ws2812_update();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_MAIN, "Main Menu");

    list = lv_list_create(s_page_content);
    lv_obj_set_size(list, 224, 172);
    lv_obj_center(list);
    UI_Theme_ApplyPanel(list);

    btn = lv_list_add_btn(list, LV_SYMBOL_PLAY, "1 Motor Control");
    first_btn = btn;
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_MANUAL);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_MANUAL);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 0U);

    btn = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "2 Command Control");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_COMMAND);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_COMMAND);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 1U);

    btn = lv_list_add_btn(list, LV_SYMBOL_VIDEO, "3 SD Card Files");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_SD_BROWSER);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_SD_BROWSER);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 2U);

    btn = lv_list_add_btn(list, LV_SYMBOL_SHUFFLE, "4 Mecanum Control");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_MECANUM);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_MECANUM);
    lvgl_app_group_add_obj(btn);
    UI_Anim_StaggerIn(btn, 3U);

    btn = lv_list_add_btn(list, LV_SYMBOL_LOOP, "5 MPU6500 Data");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_MPU6500);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_MPU6500);
    lvgl_app_group_add_obj(btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_EDIT, "6 WS2812 Control");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_WS2812);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_WS2812);
    lvgl_app_group_add_obj(btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_EYE_OPEN, "7 UI Diagnostics");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_DIAGNOSTICS);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_DIAGNOSTICS);
    lvgl_app_group_add_obj(btn);

    lv_group_focus_obj(first_btn);

    lvgl_app_page_finish();
}

#include "usbd_cdc_if.h"

static uint8_t s_cmd_rx_buf[2][64];
static uint16_t s_cmd_rx_idx[2] = {0U, 0U};

static void lvgl_app_cmd_parse(uint8_t channel, uint8_t *frame, uint8_t len)
{
    uint8_t dev_id = frame[3];
    uint8_t cmd    = frame[4];

    if (LVGL_App_IsCommandControlActive() == 0U)
    {
        return;
    }
    
    if (dev_id == 0x0C && len == 0x0A) // Virtual Joystick Mecanum Control
    {
        s_joy_lx = (int8_t)frame[4];
        s_joy_ly = (int8_t)frame[5];
        s_joy_rx = (int8_t)frame[6];
        s_joy_ry = (int8_t)frame[7];
        
        float wz = (float)s_joy_lx * 2.0f;
        float vy = (float)s_joy_rx * 10.0f;
        float vx = (float)s_joy_ry * 10.0f;
        
        Mecanum_MixedControl(vx, vy, wz, 0.0f, 0.0f, 0.0f);
        
        s_ctrl_last_actual_refresh_tick = 0; // Force UI refresh
        return;
    }
    
    if (cmd == 0x02) // Write
    {
        if (dev_id == 0x01 && len == 0x0A) // Multi-motor 
        {
            uint8_t i;
            for (i = 0; i < 4; i++) {
                (void)LVGL_App_CommandSetMotorSpeed((uint8_t)(i + 1U),
                                                    (int8_t)frame[5 + i]);
            }
        }
        else if (dev_id == 0x02 && len >= 0x08) // Single-motor
        {
            uint8_t port = frame[5];
            int8_t speed = (int8_t)frame[6];
            if (port >= 1 && port <= 4) {
                (void)LVGL_App_CommandSetMotorSpeed(port, speed);
            }
        }
        else if (dev_id == 0x03 && len >= 0x0D) // Mecanum Mixed Control
        {
            // format: 0x77 0x68 [len=13] 0x03 0x02 [mode] [VxL] [VxH] [VyL] [VyH] [WzL] [WzH] [ck/tail]
            uint8_t m_mode = frame[5];
            int16_t vx_in  = (int16_t)((frame[7] << 8)  | frame[6]);
            int16_t vy_in  = (int16_t)((frame[9] << 8)  | frame[8]);
            int16_t wz_in  = (int16_t)((frame[11] << 8) | frame[10]);
            
            if (m_mode == 1) {
                // DISTANCE mode: use fixed default speed (e.g. 100.0)
                Mecanum_MixedControl(100.0f, 100.0f, 100.0f, (float)vx_in, (float)vy_in, (float)wz_in);
            } else {
                // SPEED mode
                Mecanum_MixedControl((float)vx_in, (float)vy_in, (float)wz_in, 0.0f, 0.0f, 0.0f);
            }
        }
        else if (dev_id == 0x05 && len >= 0x09) // PWM Servo
        {
            uint8_t port  = frame[5]; // 1~7
            uint8_t angle = frame[7]; // 0~180
            
            if (port >= 1 && port <= 2) {
                int16_t target_angle = (int16_t)angle * 3 / 2; // 0~180 to 0~270
                s_servo_angle_preset[port - 1] = target_angle;
                lvgl_app_servo_angle_send_cmd(port, target_angle);
            }
        }
    }
    else if (cmd == 0x01) // Read Encoder
    {
        uint8_t tx_buf[16];
        uint8_t tx_len = 0;
        
        lvgl_app_motor_speed_sync_actual();
        
        if (dev_id == 0x03) // Multi-encoder
        {
            tx_buf[0] = 0x77;
            tx_buf[1] = 0x68;
            tx_buf[2] = 0x0A;
            tx_buf[3] = 0x03;
            tx_buf[4] = 0x01; 
            tx_buf[5] = (uint8_t)(s_motor_speed_actual[0]);
            tx_buf[6] = (uint8_t)(s_motor_speed_actual[1]);
            tx_buf[7] = (uint8_t)(s_motor_speed_actual[2]);
            tx_buf[8] = (uint8_t)(s_motor_speed_actual[3]);
            tx_buf[9] = 0x0A;
            tx_len = 0x0A;
        }
        else if (dev_id == 0x04) // Single-encoder 
        {
            uint8_t port = frame[5];
            tx_buf[0] = 0x77;
            tx_buf[1] = 0x68;
            tx_buf[2] = 0x08;
            tx_buf[3] = 0x04;
            tx_buf[4] = 0x01;
            tx_buf[5] = port;
            if (port >= 1 && port <= 4) {
                tx_buf[6] = (uint8_t)(s_motor_speed_actual[port - 1]);
            } else {
                tx_buf[6] = 0;
            }
            tx_buf[7] = 0x0A;
            tx_len = 0x08;
        }
        
        if (tx_len > 0) {
            if (channel == 0U)
            {
                (void)CommService_UartSend(tx_buf, tx_len);
            }
            else
            {
                (void)CDC_Transmit_FS(tx_buf, tx_len);
            }
        }
    }
    
    // Force UI refresh on next main loop
    s_ctrl_last_actual_refresh_tick = 0;
}

void lvgl_app_com_rx_cb(uint8_t *buf, uint32_t len)
{
    lvgl_app_com_rx_channel_cb(0U, buf, len);
}

void lvgl_app_com_rx_channel_cb(uint8_t channel, uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint8_t *rx_buf;
    uint16_t *rx_idx;

    if ((buf == NULL) || (channel >= 2U)) return;
    if (LVGL_App_IsCommandControlActive() == 0U)
    {
        s_cmd_rx_idx[channel] = 0U;
        return;
    }
    rx_buf = s_cmd_rx_buf[channel];
    rx_idx = &s_cmd_rx_idx[channel];

    for (i = 0; i < len; i++) {
        if (*rx_idx < sizeof(s_cmd_rx_buf[channel])) {
            rx_buf[(*rx_idx)++] = buf[i];
        }
        
        while (*rx_idx >= 3U) { 
            if (rx_buf[0] != 0x77 || rx_buf[1] != 0x68) {
                uint16_t j;
                for (j = 0; j < *rx_idx - 1U; j++) rx_buf[j] = rx_buf[j+1U];
                (*rx_idx)--;
                continue;
            }
            
            uint8_t frame_len = rx_buf[2];
            if (frame_len < 0x04 || frame_len > 0x10) { 
                uint16_t j;
                for (j = 0; j < *rx_idx - 2U; j++) rx_buf[j] = rx_buf[j+2U];
                *rx_idx -= 2U;
                continue;
            }
            
            if (*rx_idx >= frame_len) {
                if (rx_buf[frame_len - 1U] == 0x0A) {
                    lvgl_app_cmd_parse(channel, rx_buf, frame_len);
                }
                uint16_t j;
                for (j = 0; j < *rx_idx - frame_len; j++) {
                    rx_buf[j] = rx_buf[j + frame_len];
                }
                *rx_idx -= frame_len;
            } else {
                break;
            }
        }
    }
}

static void lvgl_app_command_exit_event_cb(lv_event_t *e)
{
    uint32_t key;
    lv_event_code_t code = lv_event_get_code(e);

    if (code != LV_EVENT_KEY)
    {
        return;
    }

    key = lvgl_app_event_get_key(e);
    if ((key != LV_KEY_ESC) && (key != LV_KEY_LEFT))
    {
        return;
    }

    LVGL_App_CommandStopMotors();
    s_cmd_rx_idx[0] = 0U;
    s_cmd_rx_idx[1] = 0U;
    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    lv_port_indev_suppress_all_keys_until_release();
    lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
}

static void lvgl_app_show_command_control(void)
{
    lv_obj_t *status_card;
    lv_obj_t *data_panel;
    lv_obj_t *key_receiver;
    lv_obj_t *lbl;

    // Set WS2812 to 20% Red
    ws2812_set_all(rgb_to_color(51, 0, 0));
    ws2812_update();

    s_ctrl_page = LVGL_APP_CTRL_PAGE_COMMAND;
    LVGL_App_CommandStopMotors();
    s_cmd_rx_idx[0] = 0U;
    s_cmd_rx_idx[1] = 0U;
    lvgl_app_control_clear_row_refs();
    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_COMMAND, "Command Control");

    status_card = lv_obj_create(s_page_content);
    lv_obj_set_size(status_card, 220, 28);
    lv_obj_align(status_card, LV_ALIGN_TOP_MID, 0, 3);
    UI_Theme_ApplyDataCard(status_card);
    lv_obj_set_style_bg_color(status_card, lv_color_hex(0xE8F1FF), LV_PART_MAIN);
    lv_obj_clear_flag(status_card, LV_OBJ_FLAG_SCROLLABLE);

    lbl = lv_label_create(status_card);
    lv_label_set_text(lbl, "USB / USART COMMAND MODE");
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x173B67), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_center(lbl);
    UI_Anim_StaggerIn(status_card, 0U);

    data_panel = lv_obj_create(s_page_content);
    lv_obj_set_size(data_panel, 220, 142);
    lv_obj_align(data_panel, LV_ALIGN_BOTTOM_MID, 0, -3);
    UI_Theme_ApplyPanel(data_panel);
    lv_obj_clear_flag(data_panel, LV_OBJ_FLAG_SCROLLABLE);
    UI_Anim_StaggerIn(data_panel, 1U);

    lbl = lv_label_create(data_panel);
    lv_label_set_text(lbl, "LIVE CONTROL DATA");
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x475467), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 7, 4);

    s_cmd_ctrl_label = lv_label_create(data_panel);
    lv_label_set_text(s_cmd_ctrl_label, "Listening...");
    lv_obj_set_style_text_color(s_cmd_ctrl_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_cmd_ctrl_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(s_cmd_ctrl_label, LV_ALIGN_TOP_LEFT, 7, 25);

    key_receiver = lv_obj_create(s_page_content);
    lv_obj_set_size(key_receiver, 1, 1);
    lv_obj_set_style_opa(key_receiver, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(key_receiver, 0, LV_PART_MAIN);
    lv_obj_clear_flag(key_receiver, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(key_receiver, lvgl_app_command_exit_event_cb, LV_EVENT_KEY, NULL);
    lv_group_add_obj(s_group, key_receiver);
    lv_group_focus_obj(key_receiver);

    lvgl_app_set_status("USB/USART active - Left/KEY2 returns");

    lvgl_app_motor_speed_sync_actual();
    lvgl_app_motor_speed_reset_followers();
    s_ctrl_last_actual_refresh_tick = 0;
    lvgl_app_control_refresh_rows();
    lvgl_app_page_finish();
}

static void lvgl_app_show_sd_browser(void)
{
    lv_obj_t *list;
    lv_obj_t *btn;
    lv_obj_t *back_btn;
    lv_obj_t *up_btn;
    lv_obj_t *focus_obj;
    char path_line[LVGL_APP_BROWSER_PATH_LEN + 8U];
    uint16_t i;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_control_clear_row_refs();
    (void)lvgl_app_scan_browser_entries();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_SD_BROWSER, "SD Card Files");

    (void)snprintf(path_line, sizeof(path_line), "Path: %s", s_browser_path);
    btn = lv_label_create(s_page_content);
    lv_label_set_long_mode(btn, LV_LABEL_LONG_DOT);
    lv_obj_set_width(btn, 224);
    lv_label_set_text(btn, path_line);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 2);

    list = lv_list_create(s_page_content);
    lv_obj_set_size(list, 224, 154);
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -2);
    UI_Theme_ApplyPanel(list);

    back_btn = lv_list_add_btn(list, LV_SYMBOL_LEFT, "Back");
    lv_obj_add_event_cb(back_btn, lvgl_app_sd_file_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_SD_ID_BACK);
    lv_obj_add_event_cb(back_btn, lvgl_app_sd_file_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_SD_ID_BACK);
    lvgl_app_group_add_obj(back_btn);
    UI_Anim_StaggerIn(back_btn, 0U);

    up_btn = lv_list_add_btn(list, LV_SYMBOL_UP, "Parent");
    lv_obj_add_event_cb(up_btn, lvgl_app_sd_file_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_SD_ID_UP);
    lv_obj_add_event_cb(up_btn, lvgl_app_sd_file_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_SD_ID_UP);
    lvgl_app_group_add_obj(up_btn);
    UI_Anim_StaggerIn(up_btn, 1U);

    focus_obj = up_btn;

    if (s_browser_entry_count == 0U)
    {
        if (s_browser_scan_result == FR_OK)
        {
            btn = lv_list_add_btn(list, LV_SYMBOL_CLOSE, "No folders or media");
            lvgl_app_set_status("Empty: %s", s_browser_path);
        }
        else
        {
            btn = lv_list_add_btn(list, LV_SYMBOL_WARNING, "SD read failed");
            lvgl_app_set_status("SD scan failed (%d): %s", (int)s_browser_scan_result, s_browser_path);
        }
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_group_focus_obj(up_btn);
    }
    else
    {
        for (i = 0U; i < s_browser_entry_count; ++i)
        {
            if (s_browser_entries[i].type == LVGL_APP_ENTRY_DIR)
            {
                char line[LVGL_APP_ENTRY_NAME_LEN + 4U];
                (void)snprintf(line, sizeof(line), "[%s]", s_browser_entries[i].name);
                btn = lv_list_add_btn(list, LV_SYMBOL_RIGHT, line);
            }
            else if (s_browser_entries[i].type == LVGL_APP_ENTRY_GIF)
            {
                btn = lv_list_add_btn(list, LV_SYMBOL_IMAGE, s_browser_entries[i].name);
            }
            else if (s_browser_entries[i].type == LVGL_APP_ENTRY_MJPEG)
            {
                btn = lv_list_add_btn(list, LV_SYMBOL_VIDEO, s_browser_entries[i].name);
            }
            else
            {
                btn = lv_list_add_btn(list, LV_SYMBOL_FILE, s_browser_entries[i].name);
            }

            lv_obj_add_event_cb(btn, lvgl_app_sd_file_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)(i + LVGL_APP_SD_ID_BASE));
            lv_obj_add_event_cb(btn, lvgl_app_sd_file_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)(i + LVGL_APP_SD_ID_BASE));
            lvgl_app_group_add_obj(btn);
            UI_Anim_StaggerIn(btn, (uint8_t)(i + 2U));
            if (i == 0U)
            {
                focus_obj = btn;
            }
        }

        lv_group_focus_obj(focus_obj);
    }

    lvgl_app_page_finish();
}

static void lvgl_app_process_global_stop_key(void)
{
    uint8_t key2_pressed = (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    if (key2_pressed == 0U)
    {
        s_key2_latched = 0U;
        return;
    }

    if (s_key2_latched != 0U)
    {
        return;
    }

    s_key2_latched = 1U;

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM)
    {
        // Emergency stop logic for Mecanum specifically
        lvgl_app_motor_speed_force_clear_all();
        Mecanum_MixedControl(0, 0, 0, 0, 0, 0);
        s_mecanum_executing = 0U;
        if (s_mecanum_timer)
        {
            lv_timer_del(s_mecanum_timer);
            s_mecanum_timer = NULL;
        }
        lvgl_app_set_status("EMERGENCY STOP!");
        lvgl_app_control_refresh_rows();
    }
    else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND)
    {
        LVGL_App_CommandStopMotors();
        s_cmd_rx_idx[0] = 0U;
        s_cmd_rx_idx[1] = 0U;
        lvgl_app_set_status("EMERGENCY STOP!");
        lvgl_app_control_refresh_rows();
    }

    if (s_gif_playing != 0U)
    {
        lv_port_indev_suppress_exit_keys_until_release();
        lvgl_app_exit_gif_player("Stopped by KEY2");
    }
}

static void lvgl_app_process_media_return_key(void)
{
    uint8_t key3_pressed =
        (HAL_GPIO_ReadPin(Key3_GPIO_Port, Key3_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (key3_pressed == 0U)
    {
        s_key3_latched = 0U;
        return;
    }

    if (s_key3_latched != 0U)
    {
        return;
    }

    s_key3_latched = 1U;
    if (s_gif_playing != 0U)
    {
        lv_port_indev_suppress_exit_keys_until_release();
        lvgl_app_exit_gif_player("Returned by KEY3");
    }
}

void LVGL_App_Init(void)
{
    UI_Theme_Init(lv_disp_get_default());
    UI_TransitionManager_Init(&s_transition_manager);
    UI_Feedback_Init(&s_feedback);
    UI_PerfDiag_Init();

    /* Start TIM8 PWM for servos (1 and 2 only, 3 and 4 used by SDMMC1) */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    
    MPU6500_Init();
    imu_init();

    lvgl_app_set_status("Up/Down move, Right enters, Left goes back, KEY2 exits");
    lvgl_app_show_main_menu();
}

void LVGL_App_Process(void)
{
    uint32_t safety_faults = Safety_GetFaults();
    uint32_t ui_handler_start;

    UI_PerfDiag_Process();
    if ((s_current_screen == LVGL_APP_SCREEN_REQ_DIAGNOSTICS) &&
        ((HAL_GetTick() - s_diag_last_refresh_tick) >= 500U))
    {
        s_diag_last_refresh_tick = HAL_GetTick();
        lvgl_app_diagnostics_refresh();
    }

    if (safety_faults != s_last_safety_faults)
    {
        s_last_safety_faults = safety_faults;
        UI_Feedback_SetFault(&s_feedback, safety_faults, s_status_text);
        if (safety_faults != SAFETY_FAULT_NONE)
        {
            lvgl_app_show_toast(UI_NOTICE_ERROR, "Safety stop active");
        }
        else
        {
            lvgl_app_show_toast(UI_NOTICE_SUCCESS, "Safety stop released");
        }
        lvgl_app_control_refresh_rows();
    }

    if ((s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED) ||
        (s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND) ||
        (s_ctrl_page == LVGL_APP_CTRL_PAGE_MECANUM))
    {
        uint32_t now = HAL_GetTick();
        if ((now - s_ctrl_last_actual_refresh_tick) >= LVGL_APP_CTRL_REFRESH_MS)
        {
            s_ctrl_last_actual_refresh_tick = now;
            if ((s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED) ||
                (s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND))
            {
                lvgl_app_motor_speed_sync_actual();
                lvgl_app_motor_speed_update_followers(now);
            }
            lvgl_app_control_refresh_rows();
        }
    }

    if (g_adc_update_flag)
    {
        g_adc_update_flag = 0;
        if (s_adc_label == NULL)
        {
            s_adc_label = lv_label_create(lv_layer_sys());
            lv_obj_align(s_adc_label, LV_ALIGN_TOP_RIGHT, -10, 10);
            lv_obj_set_style_text_color(s_adc_label, lv_color_make(255, 0, 0), 0);
        }
        char buf[16];
        int v_int = (int)g_adc_voltage;
        // Calculate with 2 decimal precision and rounding to avoid truncation errors
        int v_frac = (int)((g_adc_voltage - v_int) * 100.0f + 0.5f);
        if (v_frac < 0) v_frac = -v_frac;
        snprintf(buf, sizeof(buf), "%d.%02dV", v_int, v_frac);
        (void)UI_LabelSetTextIfChanged(s_adc_label, buf);
    }

    lvgl_app_process_media_return_key();
    lvgl_app_process_global_stop_key();
    lvgl_app_update_header_activity();
    UI_Feedback_Process(&s_feedback);
    lv_port_disp_process();
    ui_handler_start = UI_PerfDiag_BeginMeasure();
    lv_timer_handler();
    UI_PerfDiag_EndUiHandler(ui_handler_start);
    lv_port_disp_process();
    lvgl_app_process_pending_screen();
}

#include "mpu6500.h"
#include "mpu6500_reg.h"
#include "imu.h"

static lv_obj_t *s_mpu_label = NULL;
static lv_timer_t *s_mpu_timer = NULL;
static lv_obj_t *s_mpu_chart = NULL;
static lv_chart_series_t *s_mpu_pitch_series = NULL;
static lv_chart_series_t *s_mpu_roll_series = NULL;
static ui_value_follower_t s_mpu_angle_followers[3];
static uint32_t s_mpu_last_chart_tick = 0U;

static void lvgl_app_format_centi(char *buffer, size_t size, int32_t value)
{
    int32_t whole;
    int32_t fraction;

    if ((buffer == NULL) || (size == 0U))
    {
        return;
    }

    whole = value / 100;
    fraction = value % 100;
    if (fraction < 0)
    {
        fraction = -fraction;
    }

    if ((value < 0) && (whole == 0))
    {
        (void)snprintf(buffer, size, "-0.%02ld", (long)fraction);
    }
    else
    {
        (void)snprintf(buffer, size, "%ld.%02ld", (long)whole, (long)fraction);
    }
}

static void mpu6500_timer_cb(lv_timer_t *timer)
{
    ImuServiceSnapshot_t snapshot;
    uint32_t now;
    int32_t angles[3];
    char pitch_text[20];
    char roll_text[20];
    char yaw_text[20];

    (void)timer;
    if (s_ctrl_page != LVGL_APP_CTRL_PAGE_MPU6500) {
        return;
    }

    if (IMU_Service_GetSnapshot(&snapshot) == 0U) {
        if (s_mpu_label) {
            char errmsg[64];
            snprintf(errmsg, sizeof(errmsg), "IIC Error! No valid IMU sample");
            (void)UI_LabelSetTextIfChanged(s_mpu_label, errmsg);
        }
        return;
    }
    
    now = HAL_GetTick();
    UI_ValueFollower_SetTarget(&s_mpu_angle_followers[0],
                               (int32_t)(snapshot.angles.pitch * 100.0f));
    UI_ValueFollower_SetTarget(&s_mpu_angle_followers[1],
                               (int32_t)(snapshot.angles.roll * 100.0f));
    UI_ValueFollower_SetTarget(&s_mpu_angle_followers[2],
                               (int32_t)(snapshot.angles.yaw * 100.0f));
    angles[0] = UI_ValueFollower_Update(&s_mpu_angle_followers[0], now, 160U);
    angles[1] = UI_ValueFollower_Update(&s_mpu_angle_followers[1], now, 160U);
    angles[2] = UI_ValueFollower_Update(&s_mpu_angle_followers[2], now, 160U);

    lvgl_app_format_centi(pitch_text, sizeof(pitch_text), angles[0]);
    lvgl_app_format_centi(roll_text, sizeof(roll_text), angles[1]);
    lvgl_app_format_centi(yaw_text, sizeof(yaw_text), angles[2]);

    if ((s_mpu_chart != NULL) && (lv_obj_is_valid(s_mpu_chart) != false) &&
        (s_mpu_pitch_series != NULL) && (s_mpu_roll_series != NULL) &&
        ((now - s_mpu_last_chart_tick) >= 100U))
    {
        s_mpu_last_chart_tick = now;
        lv_chart_set_next_value(s_mpu_chart, s_mpu_pitch_series,
                                (lv_coord_t)(angles[0] / 100));
        lv_chart_set_next_value(s_mpu_chart, s_mpu_roll_series,
                                (lv_coord_t)(angles[1] / 100));
    }

    char buf[160];
    snprintf(buf, sizeof(buf),
             "Pitch: %s   Roll: %s\n"
             "Yaw: %s\n"
             "Acc: %d, %d, %d\n"
             "Gyro: %d, %d, %d",
             pitch_text, roll_text, yaw_text,
             snapshot.ax, snapshot.ay, snapshot.az,
             snapshot.gx, snapshot.gy, snapshot.gz);
    if (s_mpu_label) {
        (void)UI_LabelSetTextIfChanged(s_mpu_label, buf);
    }
}

static void lvgl_app_mpu6500_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_KEY)
    {
        uint32_t key = lv_event_get_key(e);
        if (key == LV_KEY_ESC || key == LV_KEY_LEFT)
        {
            if (s_mpu_timer) {
                lv_timer_del(s_mpu_timer);
                s_mpu_timer = NULL;
            }
            s_mpu_chart = NULL;
            s_mpu_pitch_series = NULL;
            s_mpu_roll_series = NULL;
            lvgl_app_set_status("Global exit");
            lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
        }
    }
}

static void lvgl_app_show_mecanum_control(void)
{
    lv_obj_t *row_btn;
    uint8_t i;

    // Set WS2812 to 20% Red
    ws2812_set_all(rgb_to_color(51, 0, 0));
    ws2812_update();

    s_ctrl_page = LVGL_APP_CTRL_PAGE_MECANUM;
    s_ctrl_selected_row = 0U;
    s_ctrl_editing = 0U;
    s_mecanum_executing = 0U;
    if (s_mecanum_timer)
    {
        lv_timer_del(s_mecanum_timer);
        s_mecanum_timer = NULL;
    }
    Mecanum_MixedControl(0, 0, 0, 0, 0, 0);

    lvgl_app_control_clear_row_refs();
    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_MECANUM, "Mecanum Control");

    for (i = 0U; i < 7U; ++i)
    {
        row_btn = lv_btn_create(s_page_content);
        s_ctrl_row_btns[i] = row_btn;
        lv_obj_set_size(row_btn, 220, 22);
        lv_obj_align(row_btn, LV_ALIGN_TOP_MID, 0, 2 + (lv_coord_t)i * 25);
        UI_Theme_ApplyDataCard(row_btn);
        lv_obj_clear_flag(row_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_FOCUSED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)i);

        s_ctrl_row_labels[i] = lv_label_create(row_btn);
        lv_obj_set_style_text_font(s_ctrl_row_labels[i], &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align(s_ctrl_row_labels[i], LV_ALIGN_TOP_LEFT, 8, 1);

        if (i < 6U)
        {
            s_ctrl_row_bars[i] = lv_bar_create(row_btn);
            lv_bar_set_range(s_ctrl_row_bars[i], 0, 100);
            lv_obj_set_size(s_ctrl_row_bars[i], 212, 18);
            lv_obj_align(s_ctrl_row_bars[i], LV_ALIGN_RIGHT_MID, -3, 0);
            UI_Theme_ApplyValueFill(s_ctrl_row_bars[i]);
            lv_obj_clear_flag(s_ctrl_row_bars[i], LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_move_background(s_ctrl_row_bars[i]);
            lv_obj_move_foreground(s_ctrl_row_labels[i]);
        }

        lvgl_app_group_add_obj(row_btn);
        UI_Anim_StaggerIn(row_btn, i);
    }

    if (s_ctrl_row_btns[0] != NULL)
    {
        lv_group_focus_obj(s_ctrl_row_btns[0]);
    }
    lv_group_set_editing(s_group, false);

    lvgl_app_control_refresh_rows();
    s_ctrl_last_actual_refresh_tick = HAL_GetTick();
    lvgl_app_set_status("OK to edit/exec, Left returns");
    lvgl_app_page_finish();
}

static void lvgl_app_show_mpu6500_data(void)
{
    uint8_t i;

    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_MPU6500, "MPU6500 Data");

    s_ctrl_page = LVGL_APP_CTRL_PAGE_MPU6500;

    s_mpu_label = lv_label_create(s_page_content);
    lv_label_set_long_mode(s_mpu_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_mpu_label, 228);
    lv_obj_set_style_text_align(s_mpu_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(s_mpu_label, "Loading...");
    lv_obj_align(s_mpu_label, LV_ALIGN_TOP_MID, 0, 5);
    UI_Anim_StaggerIn(s_mpu_label, 0U);

    s_mpu_chart = lv_chart_create(s_page_content);
    lv_obj_set_size(s_mpu_chart, 220, 70);
    lv_obj_align(s_mpu_chart, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_bg_opa(s_mpu_chart, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_mpu_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_line_width(s_mpu_chart, 2, LV_PART_ITEMS);
    lv_chart_set_type(s_mpu_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_mpu_chart, 32U);
    lv_chart_set_range(s_mpu_chart, LV_CHART_AXIS_PRIMARY_Y, -180, 180);
    lv_chart_set_div_line_count(s_mpu_chart, 3U, 4U);
    s_mpu_pitch_series = lv_chart_add_series(s_mpu_chart,
                                             lv_color_hex(0x2563EB),
                                             LV_CHART_AXIS_PRIMARY_Y);
    s_mpu_roll_series = lv_chart_add_series(s_mpu_chart,
                                            lv_color_hex(0xF59E0B),
                                            LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_clear_flag(s_mpu_chart, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    UI_Anim_StaggerIn(s_mpu_chart, 1U);

    for (i = 0U; i < 3U; ++i)
    {
        UI_ValueFollower_Reset(&s_mpu_angle_followers[i], 0);
    }
    s_mpu_last_chart_tick = HAL_GetTick();

    lv_obj_t *btn = lv_btn_create(s_page_content);
    lv_obj_set_size(btn, 0, 0); 
    lv_obj_add_event_cb(btn, lvgl_app_mpu6500_event_cb, LV_EVENT_KEY, NULL);
    lvgl_app_group_add_obj(btn);
    lv_group_focus_obj(btn);

    if (s_mpu_timer == NULL) {
        s_mpu_timer = lv_timer_create(mpu6500_timer_cb, 50, NULL);
    }

    lvgl_app_set_status("KEY2 or Left to go back");
    lvgl_app_page_finish();
}

static lv_obj_t *s_ws2812_slider_r = NULL;
static lv_obj_t *s_ws2812_slider_g = NULL;
static lv_obj_t *s_ws2812_slider_b = NULL;

#define WS2812_RGB_STEP 10U

static void lvgl_app_ws2812_apply(void)
{
    int32_t r;
    int32_t g;
    int32_t b;

    if (s_ctrl_page != LVGL_APP_CTRL_PAGE_WS2812)
    {
        return;
    }

    r = lv_slider_get_value(s_ws2812_slider_r);
    g = lv_slider_get_value(s_ws2812_slider_g);
    b = lv_slider_get_value(s_ws2812_slider_b);

    ws2812_set_all(rgb_to_color(r, g, b));
    ws2812_update();
}

/* VALUE_CHANGED: 仅负责把当前滑块值应用到 WS2812 */
static void lvgl_app_ws2812_slider_cb(lv_event_t *e)
{
    (void)e;
    lvgl_app_ws2812_apply();
}

/*
 * KEY PREPROCESS 回调 — 在 LVGL 类回调(默认±1)之前运行。
 * 直接 ±10，手动发送 VALUE_CHANGED（因为 stop_processing 会阻止类回调发送），
 * 然后阻止后续回调，彻底避免与类回调的竞态。
 */
static void lvgl_app_ws2812_key_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code != LV_EVENT_KEY) return;

    lv_obj_t *slider = lv_event_get_target(e);
    uint32_t key = lv_event_get_key(e);

    if (key == LV_KEY_ESC)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
        return;
    }

    if (key == LV_KEY_RIGHT || key == LV_KEY_LEFT)
    {
        int32_t val = lv_slider_get_value(slider);
        int32_t step = (key == LV_KEY_RIGHT) ? (int32_t)WS2812_RGB_STEP : -(int32_t)WS2812_RGB_STEP;

        val += step;
        if (val < 0)   val = 0;
        if (val > 255) val = 255;

        lv_slider_set_value(slider, val, LV_ANIM_OFF);

        /*
         * lv_bar_set_value 不发送 VALUE_CHANGED，类回调才发送。
         * stop_processing 会阻止类回调，所以需要手动发送 VALUE_CHANGED。
         */
        lv_event_send(slider, LV_EVENT_VALUE_CHANGED, NULL);

        /* 阻止类回调(lv_slider_event)再做 ±1 */
        lv_event_stop_processing(e);
    }
}

static void lvgl_app_show_ws2812_control(void)
{
    lvgl_app_group_reset();
    s_status_label = NULL;
    lvgl_app_page_begin(LVGL_APP_SCREEN_REQ_WS2812, "WS2812 RGB Control");

    s_ctrl_page = LVGL_APP_CTRL_PAGE_WS2812;

    /* 进入手动模式，暂停 main 循环自动彩虹 */
    g_ws2812_manual_mode = 1U;

    // R Slider (value 0..255, one-to-one with RGB channel)
    s_ws2812_slider_r = lv_slider_create(s_page_content);
    lv_slider_set_range(s_ws2812_slider_r, 0, 255);
    lv_obj_set_size(s_ws2812_slider_r, 190, 15);
    lv_obj_align(s_ws2812_slider_r, LV_ALIGN_CENTER, -4, -50);
    UI_Theme_ApplySlider(s_ws2812_slider_r);
    lv_obj_t *lr = lv_label_create(s_page_content);
    lv_label_set_text(lr, "R");
    lv_obj_align_to(lr, s_ws2812_slider_r, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // G Slider
    s_ws2812_slider_g = lv_slider_create(s_page_content);
    lv_slider_set_range(s_ws2812_slider_g, 0, 255);
    lv_obj_set_size(s_ws2812_slider_g, 190, 15);
    lv_obj_align(s_ws2812_slider_g, LV_ALIGN_CENTER, -4, 0);
    UI_Theme_ApplySlider(s_ws2812_slider_g);
    lv_obj_t *lg = lv_label_create(s_page_content);
    lv_label_set_text(lg, "G");
    lv_obj_align_to(lg, s_ws2812_slider_g, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // B Slider
    s_ws2812_slider_b = lv_slider_create(s_page_content);
    lv_slider_set_range(s_ws2812_slider_b, 0, 255);
    lv_obj_set_size(s_ws2812_slider_b, 190, 15);
    lv_obj_align(s_ws2812_slider_b, LV_ALIGN_CENTER, -4, 50);
    UI_Theme_ApplySlider(s_ws2812_slider_b);
    lv_obj_t *lb = lv_label_create(s_page_content);
    lv_label_set_text(lb, "B");
    lv_obj_align_to(lb, s_ws2812_slider_b, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // Bind slider VALUE_CHANGED events (必须在 set_value 之前注册)
    lv_obj_add_event_cb(s_ws2812_slider_r, lvgl_app_ws2812_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_ws2812_slider_g, lvgl_app_ws2812_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_ws2812_slider_b, lvgl_app_ws2812_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // KEY PREPROCESS: 在类回调(±1)之前执行，±10 并 stop_processing
    lv_obj_add_event_cb(s_ws2812_slider_r, lvgl_app_ws2812_key_cb, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(s_ws2812_slider_g, lvgl_app_ws2812_key_cb, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(s_ws2812_slider_b, lvgl_app_ws2812_key_cb, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);

    // Initial state: 10% green (G=20), R/B=0
    lv_slider_set_value(s_ws2812_slider_r, 0,  LV_ANIM_OFF);
    lv_slider_set_value(s_ws2812_slider_g, 20, LV_ANIM_OFF);
    lv_slider_set_value(s_ws2812_slider_b, 0,  LV_ANIM_OFF);

    // 显式触发一次 apply，确保 WS2812 显示初始绿色
    lvgl_app_ws2812_apply();

    lvgl_app_group_add_obj(s_ws2812_slider_r);
    lvgl_app_group_add_obj(s_ws2812_slider_g);
    lvgl_app_group_add_obj(s_ws2812_slider_b);

    lv_group_set_editing(s_group, true);

    lv_group_focus_obj(s_ws2812_slider_r);

    lvgl_app_set_status("Select to edit, KEY2 to return");
    lvgl_app_page_finish();
}


