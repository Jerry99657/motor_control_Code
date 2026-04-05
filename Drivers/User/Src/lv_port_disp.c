#include "lv_port_disp.h"

#include "lcd_spi_154.h"
#include "lvgl.h"

#define LV_PORT_HOR_RES 240
#define LV_PORT_VER_RES 240
#define LV_PORT_BUF_LINES 20

static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t s_buf_1[LV_PORT_HOR_RES * LV_PORT_BUF_LINES];

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

    LCD_CopyBuffer((uint16_t)area->x1, (uint16_t)area->y1, width, height, (const uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    static lv_disp_drv_t disp_drv;

    lv_disp_draw_buf_init(&s_draw_buf, s_buf_1, NULL, LV_PORT_HOR_RES * LV_PORT_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LV_PORT_HOR_RES;
    disp_drv.ver_res = LV_PORT_VER_RES;
    disp_drv.flush_cb = lv_port_disp_flush;
    disp_drv.draw_buf = &s_draw_buf;

    lv_disp_drv_register(&disp_drv);
}
