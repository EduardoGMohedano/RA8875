#include "ra8875.h"

void app_main() {
    ra8875_init();
    // dsc->tft->setRotation(0);// to be implemented have a look at Configure the Scan Direction or Layers (DPCR / LTPR0 / LTPR1)
    // lv_display_set_driver_data(disp, (void *)dsc);
    // lv_display_set_flush_cb(disp, flush_cb);
    // lv_display_add_event_cb(disp, resolution_changed_event_cb, LV_EVENT_RESOLUTION_CHANGED, NULL);
    // lv_display_set_buffers(disp, (void *)buf, NULL, buf_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

}