// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

lv_obj_t * uic_secondArc;
lv_obj_t * ui_Screen2;
lv_obj_t * ui_secondArc;
lv_obj_t * ui_minuteArc;
lv_obj_t * ui_hourArc;

// event funtions
void ui_event_secondArc(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_SCREEN_LOADED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 2500, &ui_Screen1_screen_init);
    }
}

void ui_event_minuteArc(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_SCREEN_LOADED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 2500, &ui_Screen1_screen_init);
    }
}

void ui_event_hourArc(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_SCREEN_LOADED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 2500, &ui_Screen1_screen_init);
    }
}

// build funtions

void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_secondArc = lv_arc_create(ui_Screen2);
    lv_obj_set_width(ui_secondArc, 80);
    lv_obj_set_height(ui_secondArc, 80);
    lv_obj_set_align(ui_secondArc, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_secondArc, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_arc_set_range(ui_secondArc, 0, 60);
    lv_arc_set_value(ui_secondArc, 0);
    lv_arc_set_bg_angles(ui_secondArc, 0, 360);
    lv_arc_set_rotation(ui_secondArc, 270);
    lv_obj_set_style_arc_width(ui_secondArc, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(ui_secondArc, false, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_secondArc, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_secondArc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_grad_dir(ui_secondArc, LV_GRAD_DIR_VER, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_minuteArc = lv_arc_create(ui_Screen2);
    lv_obj_set_width(ui_minuteArc, 220);
    lv_obj_set_height(ui_minuteArc, 220);
    lv_obj_set_align(ui_minuteArc, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_minuteArc, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_arc_set_range(ui_minuteArc, 0, 3600);
    lv_arc_set_value(ui_minuteArc, 0);
    lv_arc_set_bg_angles(ui_minuteArc, 0, 360);
    lv_arc_set_rotation(ui_minuteArc, 270);
    lv_obj_set_style_bg_grad_color(ui_minuteArc, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_minuteArc, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_minuteArc, lv_color_hex(0x10B453), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_minuteArc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_minuteArc, lv_color_hex(0x10B453), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_minuteArc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_minuteArc, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_minuteArc, LV_GRAD_DIR_VER, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_hourArc = lv_arc_create(ui_Screen2);
    lv_obj_set_width(ui_hourArc, 150);
    lv_obj_set_height(ui_hourArc, 150);
    lv_obj_set_align(ui_hourArc, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_hourArc, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_arc_set_range(ui_hourArc, 0, 3600);
    lv_arc_set_value(ui_hourArc, 0);
    lv_arc_set_bg_angles(ui_hourArc, 0, 360);
    lv_arc_set_rotation(ui_hourArc, 270);
    lv_obj_set_style_arc_color(ui_hourArc, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_hourArc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_hourArc, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(ui_hourArc, false, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_hourArc, lv_color_hex(0xB5320B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_hourArc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_hourArc, 10, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_hourArc, lv_color_hex(0xB5320B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_hourArc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_hourArc, 55, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_hourArc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_hourArc, LV_GRAD_DIR_VER, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_secondArc, ui_event_secondArc, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_minuteArc, ui_event_minuteArc, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_hourArc, ui_event_hourArc, LV_EVENT_ALL, NULL);
    uic_secondArc = ui_secondArc;

}

void ui_Screen2_screen_destroy(void)
{
    if(ui_Screen2) lv_obj_del(ui_Screen2);

    // NULL screen variables
    ui_Screen2 = NULL;
    uic_secondArc = NULL;
    ui_secondArc = NULL;
    ui_minuteArc = NULL;
    ui_hourArc = NULL;

}
