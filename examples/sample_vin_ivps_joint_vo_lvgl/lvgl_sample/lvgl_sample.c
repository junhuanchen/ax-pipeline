
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include "../lvgl/lvgl/lvgl.h"
#include "../lvgl/lv_examples/lv_examples.h"
#include "../lvgl/lv_lib_png/lv_png.h"
#include "../lvgl/lv_drivers/display/fbdev.h"
#include "evdev_mouse.h"
#include "lvgl_sample.h"
#include "../sample_vin_ivps_joint_vo_lvgl.h"


#define EVDEV_MOUSE_NAME    "/dev/input/event0"
#define EVIOCGNAME(len) _IOC(_IOC_READ, 'E', 0x06, len)

static int getTouchEventNum()
{
    char          name[64];           /* RATS: Use ok, but could be better */  
    char          buf[256] = { 0, };  /* RATS: Use ok */  
    int           fd = 0;   
    int           i;    
    for (i = 0; i < 6; i++) 
    {  
        sprintf(name, "/dev/input/event%d", i);  
        if ((fd = open(name, O_RDONLY, 0)) >= 0) 
        {  
            ioctl(fd, EVIOCGNAME(sizeof(buf)), buf);   
            if(strstr(buf, "MTOUC Touch"))
            {
                close(fd);
                return i;
            }
            printf("%s\n", name);  
            printf("name: %s\n", buf);           
            close(fd);  
        }
    }
    return -1;
}

static lv_color_t *s_buf1 = NULL;
static lv_color_t *s_buf2 = NULL;

void gui_init(uint32_t disp_width, uint32_t disp_height)
{
    
    lv_init();
    fbdev_init();
    uint32_t size_in_px_cnt = disp_width * disp_height;
    s_buf1 = (lv_color_t *)malloc(size_in_px_cnt * sizeof(lv_color_t));
    s_buf2 = (lv_color_t *)malloc(size_in_px_cnt * sizeof(lv_color_t));

    /*Create a display buffer*/
    static lv_disp_buf_t disp_buf;
    lv_disp_buf_init(&disp_buf, s_buf1, s_buf2, size_in_px_cnt);

    /*Create a display*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.buffer = &disp_buf;
    disp_drv.hor_res = disp_width;
    disp_drv.ver_res = disp_height;
    disp_drv.flush_cb = fbdev_flush;
    // disp_drv.sw_rotate = 1; 
    // disp_drv.rotated = LV_DISP_ROT_90;
    lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

    // lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    // int num = getTouchEventNum();
    evdev_mouse_set_file(EVDEV_MOUSE_NAME);
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = evdev_mouse_read;
    lv_indev_t *mouse_indev = lv_indev_drv_register(&indev_drv);


  /*Set a cursor for the mouse*/
#if 1
    LV_IMG_DECLARE(mouse_cursor_icon);                                    /*Declare the image file.*/
    lv_obj_t *cursor_obj = lv_img_create(lv_scr_act(), NULL); /*Create an image object for the cursor */
    lv_img_set_src(cursor_obj, &mouse_cursor_icon);                       /*Set the image source*/
#else
    lv_obj_t *cursor_obj = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_recolor(cursor_obj, true);
    lv_label_set_text(cursor_obj, "#ffff00 .cursor");
#endif

    lv_indev_set_cursor(mouse_indev, cursor_obj);

    /* Optional:
        * Create a memory monitor task which prints the memory usage in
        * periodically.*/
    // lv_task_create(memory_monitor, 5000, LV_TASK_PRIO_MID, NULL);

    lv_port_fs_init();

    lv_png_init();
    
    static lv_style_t screen_style;
    lv_style_init(&screen_style);
    // lv_style_set_bg_opa(&screen_style, LV_STATE_DEFAULT, LV_OPA_0);
    lv_style_set_bg_color(&screen_style, LV_STATE_DEFAULT, (lv_color_t){0x00, 0x00, 0x00, 0x00});
    lv_obj_add_style(lv_scr_act(), LV_OBJ_PART_MAIN, &screen_style); /*Default button style*/
    lv_obj_add_style(lv_scr_act(), LV_BTN_PART_MAIN, &screen_style); /*Default button style*/
    lv_obj_add_style(lv_scr_act(), LV_IMG_PART_MAIN, &screen_style); /*Default button style*/

    lv_obj_t *screen = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(screen, disp_width, disp_height);
    lv_obj_add_style(screen, LV_OBJ_PART_MAIN, &screen_style);
    lv_obj_align(screen, NULL, LV_ALIGN_CENTER, 0, 0);

    // lv_obj_t *obj = lv_img_create(lv_scr_act(), NULL);
    // lv_img_set_src(obj, "/home/res/logo.png");
    // lv_obj_add_style(obj, LV_STATE_DEFAULT, &screen_style);
    // lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 0);

    // example test
    // lv_ex_get_started_1();
    // lv_ex_get_started_2();
    // lv_ex_get_started_3();

    // lv_ex_style_7();
    // lv_ex_style_8();
    // lv_ex_style_9();

    lv_demo_widgets();
    // lv_demo_benchmark();

}

void gui_exit(void)
{
  fbdev_exit();
  if (s_buf1)
    free(s_buf1), s_buf1 = NULL;
  if (s_buf1)
    free(s_buf2), s_buf2 = NULL;
}

// ==================

/* 互斥锁创建 */
static pthread_mutex_t _gui_mutex_lock = PTHREAD_MUTEX_INITIALIZER;

/* 互斥锁初始化 */
int mf_gui_mutex_lock_init()
{
 return !pthread_mutex_init(&_gui_mutex_lock, NULL);
}

/* 互斥锁加锁 */
int mf_gui_mutex_lock()
{
  return !pthread_mutex_lock(&_gui_mutex_lock);
}

/* 互斥锁解锁 */
int mf_gui_mutex_unlock()
{
 return !pthread_mutex_unlock(&_gui_mutex_lock);
}

pthread_t th;
int th_usec = 0;

void *thread(void *arg)
{
  mf_gui_mutex_lock_init();
  while (th_usec)
  {
    mf_gui_mutex_lock();
    lv_task_handler();
    usleep(th_usec);
    mf_gui_mutex_unlock();
  }
  return 0;
}

int _gui_loop_set_timer(int tv_usec)
{
  th_usec = tv_usec;
  int ret = pthread_create(&th, NULL, thread, &th_usec);
  return (ret != 0) ? -1 : 0;
}

void _gui_loop_clr_timer()
{
  int *thread_ret = NULL;
  if (th_usec)
  {
    th_usec = 0;
    pthread_join(th, (void **)&thread_ret);
  }
}

void lvgl_start()
{
  gui_init(SAMPLE_MINOR_STREAM_WIDTH, SAMPLE_MINOR_STREAM_HEIGHT);
  _gui_loop_set_timer(16*1000);
}

void lvgl_stop()
{
  _gui_loop_clr_timer();
  gui_exit();
}