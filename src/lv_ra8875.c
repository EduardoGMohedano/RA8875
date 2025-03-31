#include "lv_ra8875.h"

#ifdef LV_USE_RA8875

#include "ra8875.h"
#include <stdint.h>

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void resolution_changed_event_cb(lv_event_t* e);

lv_display_t * lv_ra8875_create(uint32_t hor_res, uint32_t ver_res, void* buf, uint32_t buf_size_bytes){
    lv_display_t * disp = lv_display_create(hor_res, ver_res);
    if(disp == NULL) {
        return NULL;
    }

    ra8875_init();
    ra8875_set_rotation(0);
    // lv_display_set_driver_data(disp, (void *)dsc);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_add_event_cb(disp, resolution_changed_event_cb, LV_EVENT_RESOLUTION_CHANGED, NULL);
    lv_display_set_buffers(disp, (void *)buf, NULL, buf_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    return disp;
}

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map){
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    #if DEBUG
        ESP_LOGI(TAG, "flush: %d,%d at %d,%d", area->x1, area->x2, area->y1, area->y2 );
    #endif

    // Get lock
    // disp_spi_acquire(); TO-DO CHECK IF REALLY NECESSARY TO ACQUIRE THE BUS

    // Set window
    #if DEBUG
        ESP_LOGI(TAG, "flush: set window (x1,y1): %d,%d -> to point(x2,y2): %d,%d", area->x1, area->y1, area->x2, area->y2);
    #endif
        ra8875_set_window(area->x1, area->y1, area->x2 , area->y2);
    

    // Set cursor  TODO - IS THIS NEEDED?
    #if DEBUG
            ESP_LOGI(TAG, "flush: set cursor (x,y): %d,%d" area->x1, area->y1);
    #endif
            ra8875_set_memory_write_cursor(area->x1, area->y1);
    

    // // Update to future cursor location
    // y = area->y2 + 1;
    // if (y >= LV_VER_RES_MAX) {
    //     y = 0;
    // }

    // Write data
    ra8875_send_buffer(px_map, w*h, false);

    // Release lock
    // disp_spi_release();

    lv_display_flush_ready(disp);
}

static void resolution_changed_event_cb(lv_event_t* e){

    lv_display_t * disp = (lv_display_t *)lv_event_get_target(e);
    lv_display_rotation_t rot = lv_display_get_rotation(disp);

    /* handle rotation */
    switch(rot) {
        case LV_DISPLAY_ROTATION_0:
            ra8875_set_rotation(0);   /* Portrait orientation */
            break;
        case LV_DISPLAY_ROTATION_90:
            ra8875_set_rotation(1);   
            break;
        case LV_DISPLAY_ROTATION_180:
            ra8875_set_rotation(2);   
            break;
        case LV_DISPLAY_ROTATION_270:
            ra8875_set_rotation(3);   
            break;
    }
}

#endif