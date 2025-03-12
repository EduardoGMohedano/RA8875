#ifndef LV_RA8875_H
#define LV_RA8875_H

#ifdef LV_USE_RA8875

#include "../../../display/lv_display.h"

lv_display_t * lv_ra8875_create(uint32_t hor_res, uint32_t ver_res, void* buf, uint32_t buf_size_bytes);

#endif

#endif