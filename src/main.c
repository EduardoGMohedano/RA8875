#include "freeRTOS/FreeRTOS.h"
#include <lvgl.h>
#include "lv_ra8875.h"

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   800
#define TFT_VER_RES   480

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 6];

uint32_t my_tick() {
  return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void set_angle(void * obj, int32_t v){
    lv_arc_set_value((lv_obj_t *)obj, v);
}

/**
 * Create an array of arcs which acts as a loader.
 */
void lv_example_arcs(void){
    /*Create an Arc*/
    lv_obj_t* arc[9];
    lv_anim_t a[9];

    for(int i = 0; i < 9; i++){
      arc[i] = lv_arc_create(lv_screen_active());
      lv_arc_set_rotation(arc[i], 270);
      lv_arc_set_bg_angles(arc[i], 0, 360);
      lv_obj_remove_style(arc[i], NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
      lv_obj_remove_flag(arc[i], LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
      
      lv_obj_align(arc[i], LV_ALIGN_TOP_LEFT+i, 0, 0);
      
      lv_anim_init(&a[i]);
      lv_anim_set_var(&a[i], arc[i]);
      lv_anim_set_exec_cb(&a[i], set_angle);
      lv_anim_set_duration(&a[i], 1000);
      lv_anim_set_repeat_count(&a[i], LV_ANIM_REPEAT_INFINITE);    /*Just for the demo*/
      lv_anim_set_repeat_delay(&a[i], 500);
      lv_anim_set_values(&a[i], 0, 100);
      lv_anim_start(&a[i]);
    }
}

void app_main() {

    //Init LVGL library
    lv_init();

    //Set a callback to help LVGL library determine how much time has passed
    lv_tick_set_cb(my_tick);

    //Create display and set proper callbacks
    lv_display_t* disp = NULL;
    disp = lv_ra8875_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));

    lv_example_arcs();

    while( 1 ){
        lv_timer_handler(); /* let the GUI do its work */
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
}