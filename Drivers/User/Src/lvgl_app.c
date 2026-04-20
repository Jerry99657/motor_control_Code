#include "lvgl_app.h"

#include "lv_port_indev.h"
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

#define LVGL_APP_MAX_BROWSER_ENTRIES 48U
#define LVGL_APP_ENTRY_NAME_LEN      256U
#define LVGL_APP_BROWSER_PATH_LEN    512U
#define LVGL_APP_LVFS_PATH_LEN       (LVGL_APP_BROWSER_PATH_LEN + 3U)

#define LVGL_APP_MENU_ID_MANUAL      1U
#define LVGL_APP_MENU_ID_COMMAND     2U
#define LVGL_APP_MENU_ID_SD_BROWSER  3U
#define LVGL_APP_MENU_ID_MPU6500     4U
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
    LVGL_APP_CTRL_PAGE_MPU6500
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
    LVGL_APP_SCREEN_REQ_MPU6500
} lvgl_app_screen_req_t;

static uint16_t s_browser_entry_count = 0U;
static lvgl_app_browser_entry_t s_browser_entries[LVGL_APP_MAX_BROWSER_ENTRIES];
static char s_browser_path[LVGL_APP_BROWSER_PATH_LEN] = "/";

static gd_GIF *s_gif = NULL;
static lv_obj_t *s_gif_obj = NULL;
static lv_timer_t *s_gif_timer = NULL;
static lv_img_dsc_t s_gif_imgdsc;
static uint32_t s_gif_last_call = 0U;
static uint16_t s_gif_current_frame = 0U;
static uint8_t s_gif_playing = 0U;
static uint8_t s_key2_latched = 0U;
static char s_gif_lvfs_path[LVGL_APP_LVFS_PATH_LEN] = "S:/";

static lv_fs_drv_t s_lvfs_drv;
static uint8_t s_lvfs_registered = 0U;

static lvgl_app_ctrl_page_t s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
static lvgl_app_screen_req_t s_pending_screen_req = LVGL_APP_SCREEN_REQ_NONE;
static uint8_t s_ctrl_selected_row = 0U;
static uint8_t s_ctrl_editing = 0U;
static int16_t s_motor_speed_preset[LVGL_APP_MOTOR_COUNT] = {0, 0, 0, 0};
static int32_t s_motor_speed_actual[LVGL_APP_MOTOR_COUNT] = {0, 0, 0, 0};
static int16_t s_servo_angle_preset[LVGL_APP_SERVO_COUNT] = {0, 0};
static lv_obj_t *s_ctrl_row_btns[LVGL_APP_MOTOR_COUNT] = {NULL, NULL, NULL, NULL};
static lv_obj_t *s_ctrl_row_labels[LVGL_APP_MOTOR_COUNT] = {NULL, NULL, NULL, NULL};
static uint32_t s_ctrl_last_confirm_tick = 0U;
static uint32_t s_ctrl_last_actual_refresh_tick = 0U;

static void lvgl_app_control_clear_row_refs(void)
{
    uint8_t i;

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        s_ctrl_row_btns[i] = NULL;
        s_ctrl_row_labels[i] = NULL;
    }
}

static void lvgl_app_show_main_menu(void);
static void lvgl_app_show_motor_control_menu(void);
static void lvgl_app_show_motor_speed_control(void);
static void lvgl_app_show_servo_angle_control(void);
static void lvgl_app_show_command_control(void);
static void lvgl_app_show_sd_browser(void);
static void lvgl_app_show_mpu6500_data(void);
static void lvgl_app_show_gif_player(const char *full_path, const char *name);
static void lvgl_app_exit_gif_player(const char *reason);
static void lvgl_app_motor_menu_event_cb(lv_event_t *e);
static void lvgl_app_control_event_cb(lv_event_t *e);
static void lvgl_app_request_screen(lvgl_app_screen_req_t req);
static void lvgl_app_process_pending_screen(void);

static void lvgl_app_set_status(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
           (void)vsnprintf(s_status_text, sizeof(s_status_text), fmt, args);
    va_end(args);

    if (s_status_label != NULL)
    {
        lv_label_set_text(s_status_label, s_status_text);
    }
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

    fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (fr != FR_OK)
    {
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
            if ((fr != FR_OK) || (fno.fname[0] == '\0'))
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
    DCMotor_OL_SetSpeed(motor_index, speed);
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
    else if (req == LVGL_APP_SCREEN_REQ_MPU6500)
    {
        lvgl_app_show_mpu6500_data();
    }
}

static lv_obj_t *s_cmd_ctrl_label = NULL;

static void lvgl_app_control_refresh_rows(void)
{
    uint8_t i;
    char line[64];
    char mark;
    lv_obj_t *row_label;
    lv_obj_t *row_btn;

    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND)
    {
        if (s_cmd_ctrl_label != NULL)
        {
            char big_buf[256];
            snprintf(big_buf, sizeof(big_buf),
                     "M1 Set: %+4d,  Act: %+5ld\n"
                     "M2 Set: %+4d,  Act: %+5ld\n"
                     "M3 Set: %+4d,  Act: %+5ld\n"
                     "M4 Set: %+4d,  Act: %+5ld\n"
                     "S1 Angle Set: %3d\n"
                     "S2 Angle Set: %3d",
                     s_motor_speed_preset[0], s_motor_speed_actual[0],
                     s_motor_speed_preset[1], s_motor_speed_actual[1],
                     s_motor_speed_preset[2], s_motor_speed_actual[2],
                     s_motor_speed_preset[3], s_motor_speed_actual[3],
                     s_servo_angle_preset[0], s_servo_angle_preset[1]);
            lv_label_set_text(s_cmd_ctrl_label, big_buf);
        }
        return;
    }

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
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

        if ((row_btn != NULL) && (i == s_ctrl_selected_row))
        {
            lv_obj_set_style_bg_opa(row_btn, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_opa(row_btn, LV_OPA_COVER, LV_STATE_FOCUSED);
            if (s_ctrl_editing != 0U)
            {
                lv_obj_set_style_bg_color(row_btn, lv_palette_main(LV_PALETTE_AMBER), 0);
                lv_obj_set_style_bg_color(row_btn, lv_palette_main(LV_PALETTE_AMBER), LV_STATE_FOCUSED);
            }
            else
            {
                lv_obj_set_style_bg_color(row_btn, lv_palette_main(LV_PALETTE_BLUE), 0);
                lv_obj_set_style_bg_color(row_btn, lv_palette_main(LV_PALETTE_BLUE), LV_STATE_FOCUSED);
            }
            lv_obj_set_style_text_color(row_label, lv_color_white(), 0);
        }
        else
        {
            if (row_btn != NULL)
            {
                lv_obj_set_style_bg_opa(row_btn, LV_OPA_TRANSP, 0);
                lv_obj_set_style_bg_opa(row_btn, LV_OPA_TRANSP, LV_STATE_FOCUSED);
            }
            lv_obj_set_style_text_color(row_label, lv_color_black(), 0);
        }

        mark = ' ';
        if ((s_ctrl_editing == 0U) && (i == s_ctrl_selected_row))
        {
            mark = '>';
        }

        if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED)
        {
            (void)snprintf(
                line,
                sizeof(line),
                "%c M%u   %+4d   %+8ld",
                mark,
                (unsigned int)(i + 1U),
                (int)s_motor_speed_preset[i],
                (long)s_motor_speed_actual[i]
            );
        }
        else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
        {
            (void)snprintf(
                line,
                sizeof(line),
                "%c S%u   %3d",
                mark,
                (unsigned int)(i + 1U),
                (int)s_servo_angle_preset[i]
            );
        }
        else
        {
            line[0] = '\0';
        }

        lv_label_set_text(row_label, line);
    }
}

static void lvgl_app_control_confirm_selected(void)
{
    if (s_ctrl_editing == 0U)
    {
        s_ctrl_editing = 1U;
        if (s_group != NULL)
        {
            lv_group_set_editing(s_group, true);
            lv_group_focus_freeze(s_group, true);
        }

        if ((s_ctrl_selected_row < LVGL_APP_MOTOR_COUNT) && (s_ctrl_row_btns[s_ctrl_selected_row] != NULL))
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
        }
        else if (s_ctrl_page == LVGL_APP_CTRL_PAGE_SERVO_ANGLE)
        {
            lvgl_app_servo_angle_send_cmd((uint8_t)(s_ctrl_selected_row + 1U), s_servo_angle_preset[s_ctrl_selected_row]);
            lvgl_app_set_status(
                "S%u sent: %d (KEY2 exit edit)",
                (unsigned int)(s_ctrl_selected_row + 1U),
                (int)s_servo_angle_preset[s_ctrl_selected_row]
            );
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

    if (s_ctrl_selected_row < LVGL_APP_MOTOR_COUNT)
    {
        if (s_ctrl_row_btns[s_ctrl_selected_row] != NULL)
        {
            if (lv_obj_is_valid(s_ctrl_row_btns[s_ctrl_selected_row]) != false)
            {
                lv_group_focus_obj(s_ctrl_row_btns[s_ctrl_selected_row]);
            }
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

    lvgl_app_control_clear_row_refs();
    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MOTOR_MENU);
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

    lvgl_app_control_refresh_rows();
}

static void lvgl_app_control_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uint32_t key;
    uint8_t row;
    uint32_t now;

    code = lv_event_get_code(e);
    row = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (row >= LVGL_APP_MOTOR_COUNT)
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

    if (code == LV_EVENT_SHORT_CLICKED)
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

        if (key == LV_KEY_ENTER)
        {
            now = HAL_GetTick();
            if ((now - s_ctrl_last_confirm_tick) >= 120U)
            {
                s_ctrl_last_confirm_tick = now;
                s_ctrl_selected_row = row;
                lvgl_app_control_confirm_selected();
            }

            return;
        }

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
    lv_obj_t *title;
    lv_obj_t *header;
    lv_obj_t *row_btn;
    uint8_t i;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_MOTOR_SPEED;
    s_ctrl_selected_row = 0U;
    s_ctrl_editing = 0U;
    s_ctrl_last_confirm_tick = 0U;
    lvgl_app_control_clear_row_refs();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
            lv_label_set_text(title, "Motor Speed");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    header = lv_label_create(lv_scr_act());
            lv_label_set_text(header, "ID   PRESET(%)   ACTUAL(r/min)");
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 32);

    for (i = 0U; i < LVGL_APP_MOTOR_COUNT; ++i)
    {
        row_btn = lv_btn_create(lv_scr_act());
        s_ctrl_row_btns[i] = row_btn;
        lv_obj_set_size(row_btn, 220, 22);
        lv_obj_align(row_btn, LV_ALIGN_TOP_MID, 0, 54 + (lv_coord_t)i * 24);
        lv_obj_set_style_bg_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_border_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_shadow_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_outline_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_outline_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_radius(row_btn, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_flag(row_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_FOCUSED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_SHORT_CLICKED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)i);

        s_ctrl_row_labels[i] = lv_label_create(row_btn);
        lv_obj_set_style_pad_left(s_ctrl_row_labels[i], 6, 0);
        lv_obj_set_style_pad_right(s_ctrl_row_labels[i], 6, 0);
        lv_obj_set_style_pad_top(s_ctrl_row_labels[i], 2, 0);
        lv_obj_set_style_pad_bottom(s_ctrl_row_labels[i], 2, 0);
        lv_obj_set_style_radius(s_ctrl_row_labels[i], 4, 0);
        lv_obj_align(s_ctrl_row_labels[i], LV_ALIGN_LEFT_MID, 0, 0);

        lv_group_add_obj(s_group, row_btn);
    }

    if (s_ctrl_row_btns[0] != NULL)
    {
        lv_group_focus_obj(s_ctrl_row_btns[0]);
    }
    lv_group_set_editing(s_group, false);

    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, s_status_text);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);

    lvgl_app_motor_speed_sync_actual();
    s_ctrl_last_actual_refresh_tick = HAL_GetTick();
    lvgl_app_control_refresh_rows();
        lvgl_app_set_status("Up/Down select motor, OK to edit, OK sends while editing, KEY2 exits edit, Left returns and clears speed cmd");
}

static void lvgl_app_show_servo_angle_control(void)
{
    lv_obj_t *title;
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
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Servo Angle");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    header = lv_label_create(lv_scr_act());
    lv_label_set_text(header, "ID   ANGLE");
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 32);

    for (i = 0U; i < LVGL_APP_SERVO_COUNT; ++i)
    {
        row_btn = lv_btn_create(lv_scr_act());
        s_ctrl_row_btns[i] = row_btn;
        lv_obj_set_size(row_btn, 220, 22);
        lv_obj_align(row_btn, LV_ALIGN_TOP_MID, 0, 54 + (lv_coord_t)i * 24);
        lv_obj_set_style_bg_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_border_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_shadow_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_outline_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_outline_opa(row_btn, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_radius(row_btn, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_flag(row_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_FOCUSED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_SHORT_CLICKED, (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row_btn, lvgl_app_control_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)i);

        s_ctrl_row_labels[i] = lv_label_create(row_btn);
        lv_obj_set_style_pad_left(s_ctrl_row_labels[i], 6, 0);
        lv_obj_set_style_pad_right(s_ctrl_row_labels[i], 6, 0);
        lv_obj_set_style_pad_top(s_ctrl_row_labels[i], 2, 0);
        lv_obj_set_style_pad_bottom(s_ctrl_row_labels[i], 2, 0);
        lv_obj_set_style_radius(s_ctrl_row_labels[i], 4, 0);
        lv_obj_align(s_ctrl_row_labels[i], LV_ALIGN_LEFT_MID, 0, 0);

        lv_group_add_obj(s_group, row_btn);
    }

    if (s_ctrl_row_btns[0] != NULL)
    {
        lv_group_focus_obj(s_ctrl_row_btns[0]);
    }
    lv_group_set_editing(s_group, false);

    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, s_status_text);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);

    lvgl_app_control_refresh_rows();
    lvgl_app_set_status("Up/Down select servo, OK to edit, OK sends while editing, KEY2 exits edit, Left returns");
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
    lv_obj_t *title;
    lv_obj_t *list;
    lv_obj_t *btn;
    lv_obj_t *first_btn;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
    s_ctrl_editing = 0U;
    lvgl_app_control_clear_row_refs();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Motor Control");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    list = lv_list_create(lv_scr_act());
    lv_obj_set_size(list, 220, 160);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 34);

    btn = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "1 Motor Speed");
    first_btn = btn;
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SPEED);
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SPEED);
    lv_group_add_obj(s_group, btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_REFRESH, "2 Servo Angle");
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SERVO);
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_SERVO);
    lv_group_add_obj(s_group, btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_LEFT, "Back to Main Menu");
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_BACK);
    lv_obj_add_event_cb(btn, lvgl_app_motor_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MOTOR_SUB_ID_BACK);
    lv_group_add_obj(s_group, btn);
    lv_group_focus_obj(first_btn);

    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, s_status_text);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);

    lvgl_app_set_status("Motor first, Left to go back");
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

    (void)timer;

    if ((s_gif_playing == 0U) || (s_gif == NULL))
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

    if (lvgl_app_browser_make_file_path(s_browser_entries[index].name, play_path, sizeof(play_path)) == 0U)
    {
          lvgl_app_set_status("Failed to build path");
        lvgl_app_show_sd_browser();
        return;
    }

    lvgl_app_set_status("Playing BIN %s...", s_browser_entries[index].name);
    lv_refr_now(NULL);

    play_status = SD_StartAnim_PlayFile(play_path);
    if (play_status == SD_START_ANIM_OK)
    {
          lvgl_app_set_status("Done: %s", s_browser_entries[index].name);
    }
    else if (play_status == SD_START_ANIM_ERR_STOPPED)
    {
          lvgl_app_set_status("Stopped by KEY2");
    }
    else
    {
          lvgl_app_set_status("Failed (%d): %s", (int)play_status, s_browser_entries[index].name);
    }

    lvgl_app_show_sd_browser();
}

static void lvgl_app_gif_event_cb(lv_event_t *e)
{
    lv_event_code_t code;
    uint32_t key;

    code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        lvgl_app_exit_gif_player("GIF stopped");
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

        if ((key == LV_KEY_ESC) || (key == LV_KEY_ENTER))
        {
            lvgl_app_exit_gif_player("GIF stopped");
        }
    }
}

static void lvgl_app_show_gif_player(const char *full_path, const char *name)
{
    lv_obj_t *title;
    lv_obj_t *ctrl_btn;
    lv_obj_t *ctrl_label;
    char gif_probe_reason[96];
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

    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "GIF Player");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    s_gif_obj = lv_img_create(lv_scr_act());
    s_gif_imgdsc.header.always_zero = 0;
    s_gif_imgdsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    s_gif_imgdsc.header.w = s_gif->width;
    s_gif_imgdsc.header.h = s_gif->height;
    s_gif_imgdsc.data = s_gif->canvas;
    lv_img_set_src(s_gif_obj, &s_gif_imgdsc);
    lv_obj_align(s_gif_obj, LV_ALIGN_CENTER, 0, 0);

    ctrl_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(ctrl_btn, 220, 34);
    lv_obj_align(ctrl_btn, LV_ALIGN_BOTTOM_MID, 0, -8);

    ctrl_label = lv_label_create(ctrl_btn);
    lv_label_set_text(ctrl_label, "Left/Right seek 10s, KEY2 stop");
    lv_obj_center(ctrl_label);

    lv_obj_add_event_cb(ctrl_btn, lvgl_app_gif_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ctrl_btn, lvgl_app_gif_event_cb, LV_EVENT_KEY, NULL);
    lv_group_add_obj(s_group, ctrl_btn);
    lv_group_focus_obj(ctrl_btn);

    s_status_label = lv_label_create(lv_scr_act());
    if (name != NULL)
    {
        lvgl_app_set_status("GIF: %s", name);
    }
    else
    {
        lvgl_app_set_status("Playing GIF");
    }
    lv_label_set_text(s_status_label, s_status_text);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 30);

    s_gif_playing = 1U;
    s_key2_latched = (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    s_gif_current_frame = 0U;
    s_gif_last_call = lv_tick_get();
    s_gif_timer = lv_timer_create(lvgl_app_gif_timer_cb, 10U, NULL);
}

static void lvgl_app_exit_gif_player(const char *reason)
{
    if (s_gif_playing == 0U)
    {
        return;
    }

    s_gif_playing = 0U;

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

    if (lvgl_app_browser_make_file_path(s_browser_entries[index].name, play_path, sizeof(play_path)) == 0U)
    {
          lvgl_app_set_status("Failed to build path");
        lvgl_app_show_sd_browser();
        return;
    }

    lvgl_app_set_status("Playing %s...", s_browser_entries[index].name);
    lv_refr_now(NULL);

    play_status = MJPEG_Player_PlayFile(play_path);
    if (play_status == MJPEG_PLAYER_OK)
    {
          lvgl_app_set_status("Done: %s", s_browser_entries[index].name);
    }
    else if (play_status == MJPEG_PLAYER_ERR_STOPPED)
    {
          lvgl_app_set_status("Stopped by KEY2");
    }
    else
    {
          lvgl_app_set_status("MJPEG failed (%d): %s", (int)play_status, s_browser_entries[index].name);
    }

    lvgl_app_show_sd_browser();
}

static void lvgl_app_sd_select_id(uintptr_t id)
{
    uint16_t index;

    if (id == LVGL_APP_SD_ID_BACK)
    {
        lvgl_app_show_main_menu();
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
    lvgl_app_show_main_menu();
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
            lvgl_app_show_main_menu();
            return;
        }

        return;
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
    else if (id == LVGL_APP_MENU_ID_MPU6500)
    {
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MPU6500);
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
            lvgl_app_show_main_menu();
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

static void lvgl_app_show_main_menu(void)
{
    lv_obj_t *title;
    lv_obj_t *list;
    lv_obj_t *btn;
    lv_obj_t *first_btn;

    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Main Menu");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    list = lv_list_create(lv_scr_act());
    lv_obj_set_size(list, 220, 160);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 34);

    btn = lv_list_add_btn(list, LV_SYMBOL_PLAY, "1 Motor Control");
    first_btn = btn;
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_MANUAL);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_MANUAL);
    lv_group_add_obj(s_group, btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "2 Command Control");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_COMMAND);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_COMMAND);
    lv_group_add_obj(s_group, btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_VIDEO, "3 SD Card Files");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_SD_BROWSER);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_SD_BROWSER);
    lv_group_add_obj(s_group, btn);

    btn = lv_list_add_btn(list, LV_SYMBOL_DUMMY, "4 MPU6500 Data");
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_MENU_ID_MPU6500);
    lv_obj_add_event_cb(btn, lvgl_app_menu_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_MENU_ID_MPU6500);
    lv_group_add_obj(s_group, btn);

    lv_group_focus_obj(first_btn);

    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, s_status_text);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);
}

#include "usbd_cdc_if.h"

static uint8_t s_cmd_rx_buf[64];
static uint16_t s_cmd_rx_idx = 0;

static void lvgl_app_cmd_parse(uint8_t *frame, uint8_t len)
{
    uint8_t dev_id = frame[3];
    uint8_t cmd    = frame[4];
    
    if (cmd == 0x02) // Write
    {
        if (dev_id == 0x01 && len == 0x0A) // Multi-motor 
        {
            uint8_t i;
            for (i = 0; i < 4; i++) {
                s_motor_speed_preset[i] = (int8_t)frame[5 + i];
                lvgl_app_motor_speed_send_cmd((uint8_t)(i + 1), s_motor_speed_preset[i]);
            }
        }
        else if (dev_id == 0x02 && len >= 0x08) // Single-motor
        {
            uint8_t port = frame[5];
            int8_t speed = (int8_t)frame[6];
            if (port >= 1 && port <= 4) {
                s_motor_speed_preset[port - 1] = speed;
                lvgl_app_motor_speed_send_cmd(port, speed);
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
            CDC_Transmit_FS(tx_buf, tx_len);
        }
    }
    
    // Force UI refresh on next main loop
    s_ctrl_last_actual_refresh_tick = 0;
}

void lvgl_app_usb_rx_cb(uint8_t *buf, uint32_t len)
{
    uint32_t i;
    if (s_ctrl_page != LVGL_APP_CTRL_PAGE_COMMAND) return;

    for (i = 0; i < len; i++) {
        if (s_cmd_rx_idx < sizeof(s_cmd_rx_buf)) {
            s_cmd_rx_buf[s_cmd_rx_idx++] = buf[i];
        }
        
        while (s_cmd_rx_idx >= 3) { 
            if (s_cmd_rx_buf[0] != 0x77 || s_cmd_rx_buf[1] != 0x68) {
                uint16_t j;
                for (j = 0; j < s_cmd_rx_idx - 1; j++) s_cmd_rx_buf[j] = s_cmd_rx_buf[j+1];
                s_cmd_rx_idx--;
                continue;
            }
            
            uint8_t frame_len = s_cmd_rx_buf[2];
            if (frame_len < 0x04 || frame_len > 0x10) { 
                uint16_t j;
                for (j = 0; j < s_cmd_rx_idx - 2; j++) s_cmd_rx_buf[j] = s_cmd_rx_buf[j+2];
                s_cmd_rx_idx -= 2;
                continue;
            }
            
            if (s_cmd_rx_idx >= frame_len) {
                if (s_cmd_rx_buf[frame_len - 1] == 0x0A) {
                    lvgl_app_cmd_parse(s_cmd_rx_buf, frame_len);
                }
                uint16_t j;
                for (j = 0; j < s_cmd_rx_idx - frame_len; j++) {
                    s_cmd_rx_buf[j] = s_cmd_rx_buf[j + frame_len];
                }
                s_cmd_rx_idx -= frame_len;
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
    if (code == LV_EVENT_CLICKED || code == LV_EVENT_KEY) {
        if (code == LV_EVENT_KEY) {
            key = lvgl_app_event_get_key(e);
            if (key != LV_KEY_ESC && key != LV_KEY_LEFT && key != LV_KEY_ENTER) {
                return;
            }
        }
        s_ctrl_page = LVGL_APP_CTRL_PAGE_NONE;
        s_cmd_rx_idx = 0;
        lvgl_app_motor_speed_force_clear_all();
        lvgl_app_request_screen(LVGL_APP_SCREEN_REQ_MAIN);
    }
}

static void lvgl_app_show_command_control(void)
{
    lv_obj_t *title;
    lv_obj_t *btn;
    lv_obj_t *lbl;

    s_ctrl_page = LVGL_APP_CTRL_PAGE_COMMAND;
    lvgl_app_control_clear_row_refs();
    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Command Control RX");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    s_cmd_ctrl_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_cmd_ctrl_label, "Listening...");
    lv_obj_align(s_cmd_ctrl_label, LV_ALIGN_CENTER, 0, -20);
    
    btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 120, 30);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -40);
    lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Exit Mode");
    lv_obj_center(lbl);
    
    lv_group_add_obj(s_group, btn);
    lv_obj_add_event_cb(btn, lvgl_app_command_exit_event_cb, LV_EVENT_ALL, NULL);
    
    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, "Wait CDC... Press ENTER/LEFT to exit");
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);

    lvgl_app_motor_speed_sync_actual();
    s_ctrl_last_actual_refresh_tick = 0;
    lvgl_app_control_refresh_rows();
}

static void lvgl_app_show_sd_browser(void)
{
    lv_obj_t *title;
    lv_obj_t *list;
    lv_obj_t *btn;
    lv_obj_t *back_btn;
    lv_obj_t *up_btn;
    lv_obj_t *focus_obj;
    char path_line[LVGL_APP_BROWSER_PATH_LEN + 8U];
    uint16_t i;

    (void)lvgl_app_scan_browser_entries();

    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "SD Card Files");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    (void)snprintf(path_line, sizeof(path_line), "Path: %s", s_browser_path);
    btn = lv_label_create(lv_scr_act());
    lv_label_set_text(btn, path_line);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 24);

    list = lv_list_create(lv_scr_act());
    lv_obj_set_size(list, 220, 146);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 44);

    back_btn = lv_list_add_btn(list, LV_SYMBOL_LEFT, "Back");
    lv_obj_add_event_cb(back_btn, lvgl_app_sd_file_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_SD_ID_BACK);
    lv_obj_add_event_cb(back_btn, lvgl_app_sd_file_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_SD_ID_BACK);
    lv_group_add_obj(s_group, back_btn);

    up_btn = lv_list_add_btn(list, LV_SYMBOL_UP, "Parent");
    lv_obj_add_event_cb(up_btn, lvgl_app_sd_file_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)LVGL_APP_SD_ID_UP);
    lv_obj_add_event_cb(up_btn, lvgl_app_sd_file_event_cb, LV_EVENT_KEY, (void *)(uintptr_t)LVGL_APP_SD_ID_UP);
    lv_group_add_obj(s_group, up_btn);

    focus_obj = up_btn;

    if (s_browser_entry_count == 0U)
    {
        btn = lv_list_add_btn(list, LV_SYMBOL_CLOSE, "No folders or media");
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lvgl_app_set_status("Empty: %s", s_browser_path);
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
            lv_group_add_obj(s_group, btn);
            if (i == 0U)
            {
                focus_obj = btn;
            }
        }

        lv_group_focus_obj(focus_obj);
    }

    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, s_status_text);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);
}

static void lvgl_app_process_gif_stop_key(void)
{
    uint8_t key2_pressed;

    if (s_gif_playing == 0U)
    {
        return;
    }

    key2_pressed = (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
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
    lvgl_app_exit_gif_player("Stopped by KEY2");
}

void LVGL_App_Init(void)
{
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
    if (s_ctrl_page == LVGL_APP_CTRL_PAGE_MOTOR_SPEED || s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND)
    {
        uint32_t now = HAL_GetTick();
        if ((now - s_ctrl_last_actual_refresh_tick) >= 10U)
        {
            s_ctrl_last_actual_refresh_tick = now;
            lvgl_app_motor_speed_sync_actual();
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
        lv_label_set_text(s_adc_label, buf);
    }

    lvgl_app_process_gif_stop_key();
    lv_timer_handler();
    lvgl_app_process_pending_screen();
}

#include "mpu6500.h"
#include "mpu6500_reg.h"
#include "imu.h"

static uint8_t s_mpu_id = 0;
static lv_obj_t *s_mpu_label = NULL;
static lv_timer_t *s_mpu_timer = NULL;

static void mpu6500_timer_cb(lv_timer_t *timer)
{
    if (s_ctrl_page != LVGL_APP_CTRL_PAGE_MPU6500) {
        return;
    }
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    MPU6500_GetData(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
        uint8_t id = MPU6500_ReadReg(MPU6500_WHO_AM_I);
        if (s_mpu_label) {
            char errmsg[64];
            snprintf(errmsg, sizeof(errmsg), "IIC Error! ID: 0x%02X Check wiring!", id);
            lv_label_set_text(s_mpu_label, errmsg);
        }
        return;
    }
    
    imu_data_calibration(&gx, &gy, &gz, &ax, &ay, &az);
    eulerian_angles_t angles = imu_get_eulerian_angles((float)gx, (float)gy, (float)gz, (float)ax, (float)ay, (float)az);
    
    char buf[128];
    snprintf(buf, sizeof(buf), "Pitch: %d.%02d\nRoll: %d.%02d\nYaw: %d.%02d\nAcc: %d, %d, %d\nGyro: %d, %d, %d", 
             (int)angles.pitch, abs((int)(angles.pitch*100)%100), 
             (int)angles.roll, abs((int)(angles.roll*100)%100), 
             (int)angles.yaw, abs((int)(angles.yaw*100)%100), 
             ax, ay, az, gx, gy, gz);
    if (s_mpu_label) {
        lv_label_set_text(s_mpu_label, buf);
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
            lvgl_app_set_status("Global exit");
            lvgl_app_show_main_menu();
        }
    }
}

static void lvgl_app_show_mpu6500_data(void)
{
    lvgl_app_group_reset();
    s_status_label = NULL;
    lv_obj_clean(lv_scr_act());

    s_ctrl_page = LVGL_APP_CTRL_PAGE_MPU6500;

    lv_obj_t *title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "MPU6500 Data");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    s_mpu_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_mpu_label, "Loading...");
    lv_obj_align(s_mpu_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 0, 0); 
    lv_obj_add_event_cb(btn, lvgl_app_mpu6500_event_cb, LV_EVENT_KEY, NULL);
    lv_group_add_obj(s_group, btn);
    lv_group_focus_obj(btn);

    if (s_mpu_timer == NULL) {
        s_mpu_timer = lv_timer_create(mpu6500_timer_cb, 50, NULL);
    }

    s_status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_status_label, "KEY2 or Left to go back");
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);
}


