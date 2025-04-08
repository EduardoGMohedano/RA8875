#include "freeRTOS/FreeRTOS.h"
#include <lvgl.h>
#include "lv_ra8875.h"
#include "FT5316.h"

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   800
#define TFT_VER_RES   480

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 6];

uint32_t my_tick() {
  return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}

/**
 * Create a button with a label and react on click event.
 */
void lv_example_get_started(void){
    lv_obj_t * btn = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    uint16_t btn_width = 240;
    uint16_t btn_height = 120;
    uint16_t x_pos = ( TFT_HOR_RES - btn_width)/2;
    uint16_t y_pos = ( TFT_VER_RES - btn_height)/2;
    lv_obj_set_pos(btn, x_pos, y_pos);                            /*Set its position*/
    lv_obj_set_size(btn, btn_width, btn_height);                  /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);   /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_label_set_text(label, "Button");               /*Set the labels text*/
    lv_obj_center(label);
}


/*Read the touchpad*/
void my_touchpad_read(lv_indev_t* indev, lv_indev_data_t* data ){ 
    uint16_t x, y;
    bool touched = ft5316_getTouch(&x, &y);

    if(!touched) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    }
}

void app_main() {

    //Init LVGL library
    lv_init();

    //Set a callback to help LVGL library determine how much time has passed
    lv_tick_set_cb(my_tick);

    //Start creating a group that will help to associate screen and touch
    lv_group_set_default(lv_group_create());

    //Create display and set proper callbacks
    lv_display_t* disp = NULL;
    disp = lv_ra8875_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    
    /*Initialize the input device driver LVGL and also the Hardware driver*/
    ft5316_init();

    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);
    lv_indev_set_group(indev, lv_group_get_default());

    lv_example_get_started();

    while( 1 ){
        lv_timer_handler(); /* let the GUI do its work */
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
}