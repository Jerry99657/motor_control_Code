#include "ui_theme.h"

static lv_style_t s_page_style;
static lv_style_t s_header_style;
static lv_style_t s_content_style;
static lv_style_t s_footer_style;
static lv_style_t s_title_style;
static lv_style_t s_status_style;
static lv_style_t s_panel_style;
static lv_style_t s_list_style;
static lv_style_t s_list_item_style;
static lv_style_t s_list_item_focused_style;
static lv_style_t s_list_item_pressed_style;
static lv_style_t s_list_item_disabled_style;
static lv_style_t s_control_row_style;
static lv_style_t s_data_card_style;
static lv_style_t s_value_bar_style;
static lv_style_t s_value_fill_style;
static uint8_t s_theme_initialized = 0U;

void UI_Theme_Init(lv_disp_t *disp)
{
    lv_theme_t *theme;

    if (s_theme_initialized != 0U)
    {
        return;
    }

    if (disp != NULL)
    {
        theme = lv_theme_default_init(disp,
                                      lv_color_hex(0x2563EB),
                                      lv_color_hex(0xF59E0B),
                                      false,
                                      LV_FONT_DEFAULT);
        lv_disp_set_theme(disp, theme);
    }

    lv_style_init(&s_page_style);
    lv_style_set_bg_color(&s_page_style, lv_color_hex(0xF4F7FB));
    lv_style_set_bg_opa(&s_page_style, LV_OPA_COVER);
    lv_style_set_border_width(&s_page_style, 0);
    lv_style_set_radius(&s_page_style, 0);
    lv_style_set_pad_all(&s_page_style, 0);

    lv_style_init(&s_header_style);
    lv_style_set_bg_color(&s_header_style, lv_color_hex(0x477A9C));
    lv_style_set_bg_opa(&s_header_style, LV_OPA_COVER);
    lv_style_set_border_color(&s_header_style, lv_color_hex(0x8FB7D1));
    lv_style_set_border_width(&s_header_style, 1);
    lv_style_set_border_side(&s_header_style, LV_BORDER_SIDE_BOTTOM);
    lv_style_set_radius(&s_header_style, 0);
    lv_style_set_pad_all(&s_header_style, 0);

    lv_style_init(&s_content_style);
    lv_style_set_bg_color(&s_content_style, lv_color_hex(0xF4F7FB));
    lv_style_set_bg_opa(&s_content_style, LV_OPA_COVER);
    lv_style_set_border_width(&s_content_style, 0);
    lv_style_set_radius(&s_content_style, 0);
    lv_style_set_pad_all(&s_content_style, 0);

    lv_style_init(&s_footer_style);
    lv_style_set_bg_color(&s_footer_style, lv_color_hex(0xE7EEF7));
    lv_style_set_bg_opa(&s_footer_style, LV_OPA_COVER);
    lv_style_set_border_color(&s_footer_style, lv_color_hex(0xC8D5E5));
    lv_style_set_border_width(&s_footer_style, 1);
    lv_style_set_border_side(&s_footer_style, LV_BORDER_SIDE_TOP);
    lv_style_set_radius(&s_footer_style, 0);
    lv_style_set_pad_all(&s_footer_style, 0);

    lv_style_init(&s_title_style);
    lv_style_set_text_color(&s_title_style, lv_color_hex(0xF7FBFF));
    lv_style_set_text_font(&s_title_style, &lv_font_montserrat_16);
    lv_style_set_text_align(&s_title_style, LV_TEXT_ALIGN_LEFT);

    lv_style_init(&s_status_style);
    lv_style_set_text_color(&s_status_style, lv_color_hex(0x344054));
    lv_style_set_text_font(&s_status_style, &lv_font_montserrat_12);
    lv_style_set_text_align(&s_status_style, LV_TEXT_ALIGN_CENTER);

    lv_style_init(&s_panel_style);
    lv_style_set_bg_color(&s_panel_style, lv_color_white());
    lv_style_set_bg_opa(&s_panel_style, LV_OPA_COVER);
    lv_style_set_border_color(&s_panel_style, lv_color_hex(0xCBD5E1));
    lv_style_set_border_width(&s_panel_style, 1);
    lv_style_set_radius(&s_panel_style, 10);
    lv_style_set_pad_all(&s_panel_style, 2);

    lv_style_init(&s_list_style);
    lv_style_set_bg_opa(&s_list_style, LV_OPA_TRANSP);
    lv_style_set_border_width(&s_list_style, 0);
    lv_style_set_outline_width(&s_list_style, 0);
    lv_style_set_shadow_width(&s_list_style, 0);
    lv_style_set_radius(&s_list_style, 0);
    lv_style_set_pad_all(&s_list_style, 4);
    lv_style_set_pad_row(&s_list_style, 4);

    lv_style_init(&s_list_item_style);
    lv_style_set_bg_color(&s_list_item_style, lv_color_white());
    lv_style_set_bg_opa(&s_list_item_style, LV_OPA_COVER);
    lv_style_set_text_color(&s_list_item_style, lv_color_hex(0x24364B));
    lv_style_set_text_font(&s_list_item_style, &lv_font_montserrat_16);
    lv_style_set_border_width(&s_list_item_style, 0);
    lv_style_set_radius(&s_list_item_style, 8);
    lv_style_set_outline_color(&s_list_item_style, lv_color_hex(0xD8E3EE));
    lv_style_set_outline_width(&s_list_item_style, 1);
    lv_style_set_outline_pad(&s_list_item_style, 0);
    lv_style_set_shadow_width(&s_list_item_style, 0);
    lv_style_set_pad_left(&s_list_item_style, 8);
    lv_style_set_pad_right(&s_list_item_style, 8);
    lv_style_set_pad_top(&s_list_item_style, 7);
    lv_style_set_pad_bottom(&s_list_item_style, 7);
    lv_style_set_transform_width(&s_list_item_style, 0);
    lv_style_set_transform_height(&s_list_item_style, 0);

    lv_style_init(&s_list_item_focused_style);
    lv_style_set_bg_color(&s_list_item_focused_style, lv_color_hex(0xC9E5FA));
    lv_style_set_bg_opa(&s_list_item_focused_style, LV_OPA_COVER);
    lv_style_set_text_color(&s_list_item_focused_style, lv_color_hex(0x123A5A));
    lv_style_set_outline_color(&s_list_item_focused_style, lv_color_hex(0x70B7E8));
    lv_style_set_outline_width(&s_list_item_focused_style, 1);
    lv_style_set_outline_pad(&s_list_item_focused_style, 0);
    lv_style_set_shadow_width(&s_list_item_focused_style, 0);
    lv_style_set_transform_width(&s_list_item_focused_style, 0);
    lv_style_set_transform_height(&s_list_item_focused_style, 0);

    lv_style_init(&s_list_item_pressed_style);
    lv_style_set_bg_color(&s_list_item_pressed_style, lv_color_hex(0xB8DDF8));
    lv_style_set_bg_opa(&s_list_item_pressed_style, LV_OPA_COVER);
    lv_style_set_text_color(&s_list_item_pressed_style, lv_color_hex(0x102A43));
    lv_style_set_outline_color(&s_list_item_pressed_style, lv_color_hex(0x4E9FD8));
    lv_style_set_outline_width(&s_list_item_pressed_style, 1);
    lv_style_set_shadow_width(&s_list_item_pressed_style, 0);
    lv_style_set_transform_width(&s_list_item_pressed_style, 0);
    lv_style_set_transform_height(&s_list_item_pressed_style, 0);

    lv_style_init(&s_list_item_disabled_style);
    lv_style_set_bg_color(&s_list_item_disabled_style, lv_color_hex(0xEEF2F6));
    lv_style_set_bg_opa(&s_list_item_disabled_style, LV_OPA_COVER);
    lv_style_set_text_color(&s_list_item_disabled_style, lv_color_hex(0x98A2B3));
    lv_style_set_outline_color(&s_list_item_disabled_style, lv_color_hex(0xD8E0E8));
    lv_style_set_outline_width(&s_list_item_disabled_style, 1);
    lv_style_set_shadow_width(&s_list_item_disabled_style, 0);

    lv_style_init(&s_control_row_style);
    lv_style_set_bg_color(&s_control_row_style, lv_color_white());
    lv_style_set_bg_opa(&s_control_row_style, LV_OPA_COVER);
    lv_style_set_border_width(&s_control_row_style, 0);
    lv_style_set_shadow_width(&s_control_row_style, 0);
    lv_style_set_outline_width(&s_control_row_style, 0);
    lv_style_set_radius(&s_control_row_style, 8);
    lv_style_set_pad_all(&s_control_row_style, 0);

    lv_style_init(&s_data_card_style);
    lv_style_set_bg_color(&s_data_card_style, lv_color_white());
    lv_style_set_bg_opa(&s_data_card_style, LV_OPA_COVER);
    lv_style_set_border_width(&s_data_card_style, 0);
    lv_style_set_radius(&s_data_card_style, 8);
    lv_style_set_shadow_width(&s_data_card_style, 0);
    lv_style_set_outline_color(&s_data_card_style, lv_color_hex(0xD6DFEA));
    lv_style_set_outline_width(&s_data_card_style, 1);
    lv_style_set_outline_pad(&s_data_card_style, 0);
    lv_style_set_pad_all(&s_data_card_style, 0);

    lv_style_init(&s_value_bar_style);
    lv_style_set_bg_color(&s_value_bar_style, lv_color_hex(0xDCE4EE));
    lv_style_set_bg_opa(&s_value_bar_style, LV_OPA_COVER);
    lv_style_set_radius(&s_value_bar_style, LV_RADIUS_CIRCLE);
    lv_style_set_anim_time(&s_value_bar_style, 160U);

    lv_style_init(&s_value_fill_style);
    lv_style_set_bg_opa(&s_value_fill_style, LV_OPA_TRANSP);
    lv_style_set_border_width(&s_value_fill_style, 0);
    lv_style_set_radius(&s_value_fill_style, 5);
    lv_style_set_anim_time(&s_value_fill_style, 160U);

    s_theme_initialized = 1U;
}

void UI_Theme_ApplyPageRoot(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_page_style, LV_PART_MAIN);
}

void UI_Theme_ApplyHeader(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_header_style, LV_PART_MAIN);
}

void UI_Theme_ApplyContent(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_content_style, LV_PART_MAIN);
}

void UI_Theme_ApplyFooter(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_footer_style, LV_PART_MAIN);
}

void UI_Theme_ApplyTitle(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_title_style, LV_PART_MAIN);
}

void UI_Theme_ApplyStatus(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_status_style, LV_PART_MAIN);
}

void UI_Theme_ApplyPanel(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_panel_style, LV_PART_MAIN);
}

void UI_Theme_ApplyList(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_list_style, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xAFC4D8), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(obj, LV_OPA_50, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(obj, 3, LV_PART_SCROLLBAR);
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_AUTO);
}

void UI_Theme_ApplyListItem(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_list_item_style, LV_PART_MAIN);
    lv_obj_add_style(obj, &s_list_item_focused_style,
                     LV_PART_MAIN | LV_STATE_FOCUSED);
    /* Keypad/encoder focus adds FOCUS_KEY in addition to FOCUSED.  The
     * default theme has a saturated-blue list style on that state, so cover
     * both the standalone and combined selectors explicitly. */
    lv_obj_add_style(obj, &s_list_item_focused_style,
                     LV_PART_MAIN | LV_STATE_FOCUS_KEY);
    lv_obj_add_style(obj, &s_list_item_focused_style,
                     LV_PART_MAIN | LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
    lv_obj_add_style(obj, &s_list_item_pressed_style,
                     LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_add_style(obj, &s_list_item_pressed_style,
                     LV_PART_MAIN | LV_STATE_FOCUSED | LV_STATE_PRESSED);
    lv_obj_add_style(obj, &s_list_item_pressed_style,
                     LV_PART_MAIN | LV_STATE_FOCUSED |
                         LV_STATE_FOCUS_KEY | LV_STATE_PRESSED);
    lv_obj_add_style(obj, &s_list_item_disabled_style,
                     LV_PART_MAIN | LV_STATE_DISABLED);
}

void UI_Theme_ApplyControlRow(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_control_row_style, LV_PART_MAIN);
}

void UI_Theme_ApplyDataCard(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_data_card_style, LV_PART_MAIN);
}

void UI_Theme_ApplyValueBar(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_value_bar_style, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x2563EB), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
}

void UI_Theme_ApplyValueFill(lv_obj_t *obj)
{
    lv_obj_add_style(obj, &s_value_fill_style, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xDCEBFF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(obj, 5, LV_PART_INDICATOR);
}

void UI_Theme_ApplySlider(lv_obj_t *obj)
{
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xD7E0EC), LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x2563EB), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(obj, lv_color_white(), LV_PART_KNOB);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x2563EB), LV_PART_KNOB);
    lv_obj_set_style_border_width(obj, 2, LV_PART_KNOB);
}

void UI_Theme_SetVisualState(lv_obj_t *obj, lv_obj_t *label, ui_visual_state_t state)
{
    lv_color_t bg_color = lv_color_white();
    lv_color_t text_color = lv_color_hex(0x1F2937);

    if (state == UI_VISUAL_FOCUSED)
    {
        bg_color = lv_color_hex(0xE8F1FF);
    }
    else if (state == UI_VISUAL_EDITING)
    {
        bg_color = lv_color_hex(0xFFF2CC);
    }
    else if (state == UI_VISUAL_RUNNING)
    {
        bg_color = lv_color_hex(0xE4F7EA);
    }
    else if (state == UI_VISUAL_FAULT)
    {
        bg_color = lv_color_hex(0xFDE8E8);
        text_color = lv_color_hex(0x9B1C1C);
    }
    else if (state == UI_VISUAL_DISABLED)
    {
        bg_color = lv_color_hex(0xE5E7EB);
        text_color = lv_color_hex(0x6B7280);
    }

    lv_obj_set_style_bg_color(obj, bg_color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, bg_color, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_FOCUSED);
    if (label != NULL)
    {
        lv_obj_set_style_text_color(label, text_color, LV_PART_MAIN);
    }
}

void UI_Theme_SetValueBarColor(lv_obj_t *bar, int32_t signed_value)
{
    lv_color_t color;

    if (signed_value > 0)
    {
        color = lv_color_hex(0x2563EB);
    }
    else if (signed_value < 0)
    {
        color = lv_color_hex(0xF59E0B);
    }
    else
    {
        color = lv_color_hex(0x94A3B8);
    }

    lv_obj_set_style_bg_color(bar, color, LV_PART_INDICATOR);
}

void UI_Theme_SetValueFillColor(lv_obj_t *bar, int32_t signed_value)
{
    lv_color_t color;

    if (signed_value > 0)
    {
        color = lv_color_hex(0xDCEBFF);
    }
    else if (signed_value < 0)
    {
        color = lv_color_hex(0xFFE4BA);
    }
    else
    {
        color = lv_color_hex(0xE8EEF5);
    }

    lv_obj_set_style_bg_color(bar, color, LV_PART_INDICATOR);
}

void UI_Theme_ApplyToast(lv_obj_t *toast, lv_obj_t *label, ui_notice_level_t level)
{
    lv_color_t color = lv_color_hex(0x2563EB);

    if (level == UI_NOTICE_SUCCESS)
    {
        color = lv_color_hex(0x15803D);
    }
    else if (level == UI_NOTICE_WARNING)
    {
        color = lv_color_hex(0xB45309);
    }
    else if (level == UI_NOTICE_ERROR)
    {
        color = lv_color_hex(0xB91C1C);
    }

    lv_obj_set_style_bg_color(toast, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(toast, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(toast, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(toast, 6, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
}

void UI_Theme_SetFooterFault(lv_obj_t *footer, lv_obj_t *label, uint8_t active)
{
    lv_obj_set_style_bg_color(footer,
                              (active != 0U) ? lv_color_hex(0xB91C1C) : lv_color_hex(0xE7EEF7),
                              LV_PART_MAIN);
    lv_obj_set_style_border_color(footer,
                                  (active != 0U) ? lv_color_hex(0x991B1B) : lv_color_hex(0xC8D5E5),
                                  LV_PART_MAIN);
    lv_obj_set_style_text_color(label,
                                (active != 0U) ? lv_color_white() : lv_color_hex(0x344054),
                                LV_PART_MAIN);
}

lv_color_t UI_Theme_GetVisualColor(ui_visual_state_t state)
{
    if (state == UI_VISUAL_EDITING)
    {
        return lv_color_hex(0xF59E0B);
    }
    if (state == UI_VISUAL_RUNNING)
    {
        return lv_color_hex(0x16A34A);
    }
    if (state == UI_VISUAL_FAULT)
    {
        return lv_color_hex(0xDC2626);
    }
    if (state == UI_VISUAL_DISABLED)
    {
        return lv_color_hex(0x94A3B8);
    }

    return lv_color_hex(0x2563EB);
}
