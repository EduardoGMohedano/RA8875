#include "lv_ra8875.h"
#include "ra8875.h"
#include "esp_log.h"

static const char* TAG = "lv_ra8875";
static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void resolution_changed_event_cb(lv_event_t* e);

lv_display_t * lv_ra8875_create(uint32_t hor_res, uint32_t ver_res, void* buf, uint32_t buf_size_bytes){
    lv_display_t * disp = lv_display_create(hor_res, ver_res);
    if(disp == NULL) {
        ESP_LOGE(TAG, "lv_display_create failed when being created...");
        return NULL;
    }

    if ( !ra8875_init()){
        ESP_LOGE(TAG, "RA8875 display init failed...");
        return NULL;
    } 
    ra8875_set_rotation(0);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_set_flush_wait_cb(disp, ra8875_wait_flush_cb);
    lv_display_add_event_cb(disp, resolution_changed_event_cb, LV_EVENT_RESOLUTION_CHANGED, NULL);
    lv_display_set_buffers(disp, (void *)buf, NULL, buf_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    return disp;
}

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map){

    // Write data
    ra8875_send_buffer((uint16_t*)px_map, area->x1, area->x2, area->y1, area->y2);

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
