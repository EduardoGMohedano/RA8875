#include "freeRTOS/FreeRTOS.h"
#include <lvgl.h>
#include "../demos/lv_demos.h"
#include "lv_ra8875.h"
// #include "ra8875.h"
// #include "string.h"
// LETS ADD HERE THE TOUCHSCREEN DRIVER SUPPORT

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   800
#define TFT_VER_RES   480
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 6];

// #define SQUARE_SIZE (800)
// #define LINES   (48*2)
// #define DRAW_BUF (SQUARE_SIZE*LINES)
// static uint8_t square_buf[DRAW_BUF];

uint32_t my_tick() {
  return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value((lv_obj_t *)obj, v);
}

/**
 * Create an arc which acts as a loader.
 */
void lv_example_arc_2(void)
{
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

static void set_temp(void * bar, int32_t temp)
{
    lv_bar_set_value((lv_obj_t *)bar, temp, LV_ANIM_ON);
}

/**
 * A temperature meter example
 */
void lv_example_bar_3(void)
{
    static lv_style_t style_indic;

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);

    lv_obj_t * bar[9];
    lv_anim_t a[9];
    for(int i = 0; i < 9; i ++){
        bar[i] = lv_bar_create(lv_screen_active());
        lv_obj_add_style(bar[i], &style_indic, LV_PART_INDICATOR);
        lv_obj_set_size(bar[i], 20, 200);
        lv_obj_align(bar[i], LV_ALIGN_TOP_LEFT+i, 0, 0);
        lv_bar_set_range(bar[i], -20, 40);

        lv_anim_init(&a[i]);
        lv_anim_set_exec_cb(&a[i], set_temp);
        lv_anim_set_duration(&a[i], 3000);
        // lv_anim_set_reverse_duration(&a[i], 3000);
        lv_anim_set_var(&a[i], bar);
        lv_anim_set_values(&a[i], -20, 40);
        lv_anim_set_repeat_count(&a[i], LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&a[i]);
    }
}

static void anim_x_cb(void * var, int32_t v)
{
    lv_obj_set_x(var, v);
}

static void anim_size_cb(void * var, int32_t v)
{
    lv_obj_set_size(var, 2*v, 2*v);
}

/**
 * Create a playback animation
 */
void lv_example_anim_2(void)
{

    lv_obj_t * obj = lv_obj_create(lv_screen_active());
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);

    lv_obj_align(obj, LV_ALIGN_LEFT_MID, 10, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 10, 50);
    lv_anim_set_duration(&a, 1500);
    lv_anim_set_playback_delay(&a, 100);
    lv_anim_set_playback_duration(&a, 300);
    lv_anim_set_repeat_delay(&a, 500);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);

    lv_anim_set_exec_cb(&a, anim_size_cb);
    lv_anim_start(&a);
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_set_values(&a, 10, 480);
    lv_anim_start(&a);
}

void app_main() {
    lv_init();
    lv_tick_set_cb(my_tick);

    lv_display_t* disp = NULL;
    disp = lv_ra8875_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    // fillScreen(0xff00);
    // vTaskDelay(200 / portTICK_PERIOD_MS); /* let this time pass */
    //should i set rotation here?
    
    // createScreen1();
    // lv_scr_load(screen1);
    
    // lv_obj_t *label = lv_label_create( lv_screen_active() );
    // lv_label_set_text( label, "Hello there, I'm LVGL!" );
    // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
    // lv_obj_set_style_text_font(label, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

  // lv_example_anim_2();
  lv_example_arc_2();
  // lv_example_bar_3();

  //  lv_demo_widgets();
  //  lv_demo_widgets_start_slideshow();

    // lv_example_get_started_1();
    // vTaskDelay(350 / portTICK_PERIOD_MS); /* let this time pass */
    // lv_example_anim_1();
    // vTaskDelay(350 / portTICK_PERIOD_MS); /* let this time pass */
    // lv_example_anim_2();
    // vTaskDelay(350 / portTICK_PERIOD_MS); /* let this time pass */
    // lv_example_anim_3();
    // vTaskDelay(350 / portTICK_PERIOD_MS); /* let this time pass */
    // lv_example_button_1();
    // vTaskDelay(350 / portTICK_PERIOD_MS); /* let this time pass */
    // lv_example_button_2();
    // vTaskDelay(350 / portTICK_PERIOD_MS); /* let this time pass */
    // lv_example_button_3();
    
    //Code only for SCREEN DEBUGGING
    // ra8875_init();
    // fillScreen(0x0000);
    // graphicsMode();
    // char text[] = "hello again to all my friends!";
    // textMode();
    
    // vTaskDelay(10 / portTICK_PERIOD_MS); /* let this time pass */
    // memset(square_buf, 224, DRAW_BUF);
    
    // int pos = 0;
    // uint16_t x1, x2, y1 ,y2;

    // while( pos < (480 / LINES) ){
    while( 1 ){
        lv_timer_handler(); /* let the GUI do its work */
        vTaskDelay(10 / portTICK_PERIOD_MS); /* let this time pass */

        // setCursor(pos*25,pos*25);
        // textEnlarge(1);
        // textTransparent(0xFFFF - pos*1000);
        // textWrite(text, sizeof(text));
        // x1 = 0;
        // y1 = pos* LINES;
        // x2 = 800;
        // y2 = y1 + LINES;
        
        // ra8875_set_window(x1, x2, y1, y2);
        // ra8875_set_memory_write_cursor(x1,y1);
        // // Write data
        // ra8875_send_buffer(square_buf, DRAW_BUF);

        // pos++;
      }
      // vTaskDelay(10 / portTICK_PERIOD_MS); /* let this time pass */
}