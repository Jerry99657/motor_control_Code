#include "lv_port_disp.h"

#include "lcd_spi_154.h"
#include "lvgl.h"
#include "ui_perf_diag.h"

#define LV_PORT_HOR_RES 240
#define LV_PORT_VER_RES 240
#define LV_PORT_BUF_LINES 20
#define LV_PORT_FLUSH_TIMEOUT_MS 250U

typedef enum
{
    LV_PORT_FLUSH_IDLE = 0,
    LV_PORT_FLUSH_WAIT_START,
    LV_PORT_FLUSH_ACTIVE
} lv_port_flush_state_t;

static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t s_buf_1[LV_PORT_HOR_RES * LV_PORT_BUF_LINES] __attribute__((aligned(32)));
static lv_color_t s_buf_2[LV_PORT_HOR_RES * LV_PORT_BUF_LINES] __attribute__((aligned(32)));
static volatile lv_port_flush_state_t s_flush_state = LV_PORT_FLUSH_IDLE;
static lv_disp_drv_t *s_flush_drv = NULL;
static const lv_color_t *s_flush_color = NULL;
static uint16_t s_flush_x = 0U;
static uint16_t s_flush_y = 0U;
static uint16_t s_flush_width = 0U;
static uint16_t s_flush_height = 0U;
static uint32_t s_flush_start_tick = 0U;
static uint8_t s_flush_wait_reported = 0U;

static void lv_port_disp_complete_flush(HAL_StatusTypeDef status)
{
    lv_disp_drv_t *disp_drv = s_flush_drv;

    s_flush_state = LV_PORT_FLUSH_IDLE;
    s_flush_drv = NULL;
    s_flush_color = NULL;
    UI_PerfDiag_OnFlushComplete(status);

    if (disp_drv != NULL)
    {
        lv_disp_flush_ready(disp_drv);
    }
}

static void lv_port_disp_transfer_complete(HAL_StatusTypeDef status, void *context)
{
    (void)status;
    (void)context;

    if (s_flush_state == LV_PORT_FLUSH_ACTIVE)
    {
        lv_port_disp_complete_flush(status);
    }
}

static void lv_port_disp_try_start_flush(void)
{
    HAL_StatusTypeDef status;

    if (s_flush_state != LV_PORT_FLUSH_WAIT_START)
    {
        return;
    }

    if (LCD_IsTransmitBusy() != 0U)
    {
        if (s_flush_wait_reported == 0U)
        {
            s_flush_wait_reported = 1U;
            UI_PerfDiag_OnFlushWait();
        }
        return;
    }

    s_flush_state = LV_PORT_FLUSH_ACTIVE;
    status = LCD_CopyBufferAsyncCallback(s_flush_x, s_flush_y,
                                         s_flush_width, s_flush_height,
                                         (const uint16_t *)s_flush_color,
                                         lv_port_disp_transfer_complete,
                                         s_flush_drv);
    if (status == HAL_BUSY)
    {
        s_flush_state = LV_PORT_FLUSH_WAIT_START;
        if (s_flush_wait_reported == 0U)
        {
            s_flush_wait_reported = 1U;
            UI_PerfDiag_OnFlushWait();
        }
    }
    else if (status != HAL_OK)
    {
        lv_port_disp_complete_flush(status);
    }
}

void lv_port_disp_process(void)
{
    LCD_TransferService();
    lv_port_disp_try_start_flush();

    if ((s_flush_state == LV_PORT_FLUSH_ACTIVE) &&
        ((HAL_GetTick() - s_flush_start_tick) >= LV_PORT_FLUSH_TIMEOUT_MS))
    {
        UI_PerfDiag_OnFlushTimeout();
        LCD_ResetTransferState();
    }
    else if ((s_flush_state == LV_PORT_FLUSH_WAIT_START) &&
             ((HAL_GetTick() - s_flush_start_tick) >= LV_PORT_FLUSH_TIMEOUT_MS))
    {
        UI_PerfDiag_OnFlushTimeout();
        lv_port_disp_complete_flush(HAL_TIMEOUT);
    }

    if (LCD_IsTransmitBusy() == 0U)
    {
        (void)LCD_WaitTransmitDone(0U);
    }
}

static void lv_port_disp_wait_cb(lv_disp_drv_t *disp_drv)
{
    (void)disp_drv;
    lv_port_disp_process();
}

static void lv_port_disp_monitor_cb(lv_disp_drv_t *disp_drv, uint32_t time_ms, uint32_t pixels)
{
    (void)disp_drv;
    UI_PerfDiag_OnRefresh(time_ms, pixels);
}

static void lv_port_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint16_t width;
    uint16_t height;

    if ((area->x1 < 0) || (area->y1 < 0) ||
        (area->x2 >= LV_PORT_HOR_RES) || (area->y2 >= LV_PORT_VER_RES))
    {
        lv_disp_flush_ready(disp_drv);
        return;
    }

    width = (uint16_t)(area->x2 - area->x1 + 1);
    height = (uint16_t)(area->y2 - area->y1 + 1);

    s_flush_drv = disp_drv;
    s_flush_color = color_p;
    s_flush_x = (uint16_t)area->x1;
    s_flush_y = (uint16_t)area->y1;
    s_flush_width = width;
    s_flush_height = height;
    s_flush_start_tick = HAL_GetTick();
    s_flush_wait_reported = 0U;
    s_flush_state = LV_PORT_FLUSH_WAIT_START;
    UI_PerfDiag_OnFlushStart();
    lv_port_disp_try_start_flush();
}

uint8_t lv_port_disp_wait_idle(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((s_flush_state != LV_PORT_FLUSH_IDLE) || (LCD_IsTransmitBusy() != 0U))
    {
        lv_port_disp_process();
        if ((timeout_ms != HAL_MAX_DELAY) && ((HAL_GetTick() - start_tick) >= timeout_ms))
        {
            if (s_flush_state == LV_PORT_FLUSH_ACTIVE)
            {
                UI_PerfDiag_OnFlushTimeout();
                LCD_ResetTransferState();
            }
            else if (s_flush_state == LV_PORT_FLUSH_WAIT_START)
            {
                UI_PerfDiag_OnFlushTimeout();
                lv_port_disp_complete_flush(HAL_TIMEOUT);
            }
            return 0U;
        }
    }

    lv_port_disp_process();
    return 1U;
}

void lv_port_disp_init(void)
{
    static lv_disp_drv_t disp_drv;

    lv_disp_draw_buf_init(&s_draw_buf, s_buf_1, s_buf_2, LV_PORT_HOR_RES * LV_PORT_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LV_PORT_HOR_RES;
    disp_drv.ver_res = LV_PORT_VER_RES;
    disp_drv.flush_cb = lv_port_disp_flush;
    disp_drv.wait_cb = lv_port_disp_wait_cb;
    disp_drv.monitor_cb = lv_port_disp_monitor_cb;
    disp_drv.draw_buf = &s_draw_buf;

    lv_disp_drv_register(&disp_drv);
}
