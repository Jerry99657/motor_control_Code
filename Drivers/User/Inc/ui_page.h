#ifndef UI_PAGE_H
#define UI_PAGE_H

#include "lvgl.h"

#define UI_PAGE_HEADER_HEIGHT  30
#define UI_PAGE_FOOTER_HEIGHT  30
#define UI_PAGE_CONTENT_HEIGHT 180

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *header;
    lv_obj_t *content;
    lv_obj_t *footer;
    lv_obj_t *title_label;
    lv_obj_t *status_icon;
    lv_obj_t *status_label;
} ui_page_t;

uint8_t UI_Page_Create(ui_page_t *page, lv_obj_t *parent, const char *title);
lv_obj_t *UI_Page_CreateContentLayer(ui_page_t *page);
void UI_Page_Clear(ui_page_t *page);

#endif /* UI_PAGE_H */
