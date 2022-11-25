#ifndef _LVGL_SAMPLE_H_
#define _LVGL_SAMPLE_H_

void gui_init(uint32_t disp_width, uint32_t disp_height);
void gui_exit(void);
int _gui_loop_set_timer(int tv_usec);
void _gui_loop_clr_timer();
void lvgl_start();
void lvgl_stop();

#endif