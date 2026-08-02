#include "ui_page.h"

#include "ui_theme.h"

#include <string.h>

void UI_Page_Clear(ui_page_t *page)
{
    if (page != NULL)
    {
        (void)memset(page, 0, sizeof(*page));
    }
}

uint8_t UI_Page_Create(ui_page_t *page, lv_obj_t *parent, const char *title)
{
    if ((page == NULL) || (parent == NULL) || (title == NULL))
    {
        return 0U;
    }

    UI_Page_Clear(page);

    page->root = lv_obj_create(parent);
    lv_obj_remove_style_all(page->root);
    UI_Theme_ApplyPageRoot(page->root);
    lv_obj_set_size(page->root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(page->root, 0, 0);
    lv_obj_clear_flag(page->root, LV_OBJ_FLAG_SCROLLABLE);

    page->header = lv_obj_create(page->root);
    lv_obj_remove_style_all(page->header);
    UI_Theme_ApplyHeader(page->header);
    lv_obj_set_size(page->header, LV_PCT(100), UI_PAGE_HEADER_HEIGHT);
    lv_obj_set_pos(page->header, 0, 0);
    lv_obj_clear_flag(page->header, LV_OBJ_FLAG_SCROLLABLE);

    page->title_label = lv_label_create(page->header);
    UI_Theme_ApplyTitle(page->title_label);
    lv_label_set_long_mode(page->title_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(page->title_label, 172);
    lv_label_set_text(page->title_label, title);
    lv_obj_align(page->title_label, LV_ALIGN_LEFT_MID, 8, 0);

    page->status_icon = lv_label_create(page->header);
    lv_obj_set_size(page->status_icon, 28, 24);
    lv_obj_set_style_text_align(page->status_icon, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(page->status_icon, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(page->status_icon, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_add_flag(page->status_icon, LV_OBJ_FLAG_HIDDEN);

    page->content = lv_obj_create(page->root);
    lv_obj_remove_style_all(page->content);
    UI_Theme_ApplyContent(page->content);
    lv_obj_set_size(page->content, LV_PCT(100), UI_PAGE_CONTENT_HEIGHT);
    lv_obj_set_pos(page->content, 0, UI_PAGE_HEADER_HEIGHT);
    lv_obj_clear_flag(page->content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(page->content, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    page->footer = lv_obj_create(page->root);
    lv_obj_remove_style_all(page->footer);
    UI_Theme_ApplyFooter(page->footer);
    lv_obj_set_size(page->footer, LV_PCT(100), UI_PAGE_FOOTER_HEIGHT);
    lv_obj_align(page->footer, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_clear_flag(page->footer, LV_OBJ_FLAG_SCROLLABLE);

    page->status_label = lv_label_create(page->footer);
    UI_Theme_ApplyStatus(page->status_label);
    lv_label_set_long_mode(page->status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_size(page->status_label, 228, 28);
    lv_obj_center(page->status_label);

    return 1U;
}

lv_obj_t *UI_Page_CreateContentLayer(ui_page_t *page)
{
    lv_obj_t *layer;

    if ((page == NULL) || (page->content == NULL) ||
        (lv_obj_is_valid(page->content) == false))
    {
        return NULL;
    }

    layer = lv_obj_create(page->content);
    if (layer == NULL)
    {
        return NULL;
    }

    lv_obj_remove_style_all(layer);
    UI_Theme_ApplyContent(layer);
    lv_obj_set_size(layer, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(layer, 0, 0);
    lv_obj_clear_flag(layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(layer, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    return layer;
}
