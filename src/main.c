#include "freeRTOS/FreeRTOS.h"
#include <lvgl.h>
// #include "lv_ra8875.h"
#include "ra8875.h"
// LETS ADD HERE THE TOUCHSCREEN DRIVER SUPPORT

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   480
#define TFT_VER_RES   800
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

lv_obj_t* screen1; //home

void createScreen1(void){
  screen1 = lv_obj_create(NULL);
  lv_obj_t* home_text = lv_label_create(screen1);
  lv_label_set_text(home_text, "GRBL CONTROLLER");
  lv_obj_align(home_text,LV_ALIGN_TOP_MID,0,0);
  lv_obj_set_style_text_font(home_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t* btn = lv_button_create(screen1);
  lv_obj_set_size(btn, 120,50);
  lv_obj_set_x(btn, 170);
  lv_obj_set_y(btn, 100);
  lv_obj_t* label = lv_label_create(btn);
  lv_label_set_text(label, "AUTO MODE");
  lv_obj_center(label);
//   lv_obj_add_event_cb(btn, change_screen_event_cb, LV_EVENT_CLICKED, NULL);
}



void app_main() {

    // lv_display_t* disp = NULL;
    // disp = lv_ra8875_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    //should i set rotation here?
    
    // createScreen1();
    // lv_scr_load(screen1);

    //Code only for SCREEN DEBUGGING
    ra8875_init();
    // ra8875_set_rotation(0);

    
    
    char text[] = "HELLO WORLD!";
    textMode();
    vTaskDelay(10 / portTICK_PERIOD_MS); /* let this time pass */
    
    int pos = 0;
    while(1){
      // lv_timer_handler(); /* let the GUI do its work */
        fillScreen(0x0000);
        setCursor(pos*15,pos*15);
        textEnlarge(1);
        textTransparent(0xFFFF - pos*1000);
        textWrite(text, sizeof(text));
        vTaskDelay(100 / portTICK_PERIOD_MS); /* let this time pass */
        pos++;
    }
}