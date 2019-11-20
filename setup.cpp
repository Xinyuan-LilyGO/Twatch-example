/*********************
 *      INCLUDES
 *********************/
#include "./src/board_def.h"
#include "./src/lvgl/lvgl.h"
#include "./src/drive/tft/tft.h"
#include "freertos/FreeRTOS.h"
#include "esp_freertos_hooks.h"
#include "./src/drive/fx50xx/FT5206.h"

#include <SD.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Ticker.h>
#include <WiFi.h>
#include "gui.h"
#include <Ticker.h>

/* Version check */
#if LV_VERSION_CHECK(5,3,0)

#if USE_LV_FILESYSTEM
static lv_fs_res_t pcfs_open(void *file_p, const char *fn, lv_fs_mode_t mode);
static lv_fs_res_t pcfs_close(void *file_p);
static lv_fs_res_t pcfs_read(void *file_p, void *buf, uint32_t btr, uint32_t *br);
static lv_fs_res_t pcfs_seek(void *file_p, uint32_t pos);
static lv_fs_res_t pcfs_tell(void *file_p, uint32_t *pos_p);
static void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p);
static bool touchpad_read(lv_indev_data_t *data);

#define LV_FS_OPEN()    static lv_fs_res_t pcfs_open(void *file_p, const char *fn, lv_fs_mode_t mode)
#define LV_FS_CLOSE()   static lv_fs_res_t pcfs_close(void *file_p)
#define LV_FS_READ()    static lv_fs_res_t pcfs_read(void *file_p, void *buf, uint32_t btr, uint32_t *br)
#define LV_FS_SEEK()    static lv_fs_res_t pcfs_seek(void *file_p, uint32_t pos)
#define LV_FS_TELL()    static lv_fs_res_t pcfs_tell(void *file_p, uint32_t *pos_p)

#define LV_DISP_FLUSH() static void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p)
#define LV_TOUCH_READ() static bool touchpad_read(lv_indev_data_t *data)

#endif /* USE_LV_FILESYSTEM */

#elif LV_VERSION_CHECK(6,0,0)

#if LV_USE_FILESYSTEM
static lv_fs_res_t pcfs_open(lv_fs_drv_t *drv, void *file_p, const char *fn, lv_fs_mode_t mode);
static lv_fs_res_t pcfs_close(lv_fs_drv_t *drv, void *file_p);
static lv_fs_res_t pcfs_read(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br);
static lv_fs_res_t pcfs_seek(lv_fs_drv_t *drv, void *file_p, uint32_t pos);
static lv_fs_res_t pcfs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p);

#define LV_FS_OPEN()    static lv_fs_res_t pcfs_open(lv_fs_drv_t * drv,void *file_p, const char *fn, lv_fs_mode_t mode)
#define LV_FS_CLOSE()   static lv_fs_res_t pcfs_close(lv_fs_drv_t * drv,void *file_p)
#define LV_FS_READ()    static lv_fs_res_t pcfs_read(lv_fs_drv_t * drv,void *file_p, void *buf, uint32_t btr, uint32_t *br)
#define LV_FS_SEEK()    static lv_fs_res_t pcfs_seek(lv_fs_drv_t * drv,void *file_p, uint32_t pos)
#define LV_FS_TELL()    static lv_fs_res_t pcfs_tell(lv_fs_drv_t * drv,void *file_p, uint32_t *pos_p)
#endif  /*LV_USE_FILESYSTEM */

static bool touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

#define LV_DISP_FLUSH() static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
#define LV_TOUCH_READ() static bool touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)

#endif  /* Version check */

static void lv_tick_task(void);


// #define USE_SPIFFS
#define USE_SDCARD

#ifdef USE_SPIFFS
#define LETTER "/spiffs"
#else
#define LETTER "/sd"
#endif

SPIClass SDSPI(HSPI);
Ticker sdTicker;
TTGO_TFT tft(TFT_MISO, TFT_MOSI, TFT_SCLK, TFT_CS, TFT_DC);
FT5206_Class tp;
Ticker lvglTicker;
static bool _mount = false;
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void tft_sleep()
{
    tft.sleep();
    tp.enterMonitorMode();
}

void tft_wakeup()
{
    tft.wakeup();
}

bool isSdMount()
{
    return _mount;
}

bool tryToSetupSDCard()
{
    bool ret = SD.begin(SD_CS, SDSPI, 4000000, "/sys");
    if (!ret) {
        Serial.println("[sd] begin fail");
        _mount = false;
        // sdTicker.attach_ms(2000, []() {
        //     tryToSetupSDCard();
        // });
    } else {
        Serial.println("[sd] begin pass");
        // sdTicker.detach();
        _mount = true;
    }
    return _mount;
}

void setupFilesystem()
{
#ifdef USE_SPIFFS
    SPIFFS.begin(false, "/sys");
#else
    SDSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    tryToSetupSDCard();
#endif

#if USE_LV_FILESYSTEM
    /* Add a simple drive to open images from PC*/
    lv_fs_drv_t pcfs_drv;                       /*A driver descriptor*/
    memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t));  /*Initialization*/
    pcfs_drv.file_size = sizeof(FILE *);        /*Set up fields...*/
    pcfs_drv.letter = 'P';
    pcfs_drv.open = pcfs_open;
    pcfs_drv.close = pcfs_close;
    pcfs_drv.read = pcfs_read;
    pcfs_drv.seek = pcfs_seek;
    pcfs_drv.tell = pcfs_tell;
    lv_fs_add_drv(&pcfs_drv);
#elif LV_USE_FILESYSTEM
    /* Add a simple drive to open images from PC*/
    lv_fs_drv_t pcfs_drv;                       /*A driver descriptor*/
    memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t));  /*Initialization*/
    pcfs_drv.file_size = sizeof(FILE *);        /*Set up fields...*/
    pcfs_drv.letter = 'P';
    pcfs_drv.open_cb = pcfs_open;
    pcfs_drv.close_cb = pcfs_close;
    pcfs_drv.read_cb = pcfs_read;
    pcfs_drv.seek_cb = pcfs_seek;
    pcfs_drv.tell_cb = pcfs_tell;
    lv_fs_drv_register(&pcfs_drv);
#endif
}

static void lv_tick_task(void)
{
    lv_tick_inc(portTICK_RATE_MS);
}




void stopLvglTick()
{
    lvglTicker.detach();
}

void startLvglTick()
{
    lvglTicker.attach_ms(5, []() {
        lv_tick_inc(5);
    });
}

void setupLvgl()
{
    tft.begin();

    tft.fillScreen(random(0xFFFF));

    Wire.begin(I2C_SDA, I2C_SCL);

    tp.begin(Wire);

    lv_init();

    lv_disp_drv_t disp_drv;
    lv_indev_drv_t indev_drv;
    lv_disp_drv_init(&disp_drv);

#if LV_VERSION_CHECK(5,3,0)
    /*Initialize the display*/
    disp_drv.disp_flush = disp_flush; /*Used in buffered mode (LV_VDB_SIZE != 0  in lv_conf.h)*/
    lv_disp_drv_register(&disp_drv);

    /*Initialize the touch pad*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read =  touchpad_read;
    lv_indev_drv_register(&indev_drv);

#elif LV_VERSION_CHECK(6,0,0)
    static lv_disp_buf_t disp_buf;
    static lv_color_t buf1[LV_HOR_RES_MAX * 10];                        /*A buffer for 10 rows*/
    static lv_color_t buf2[LV_HOR_RES_MAX * 10];                        /*An other buffer for 10 rows*/
    lv_disp_buf_init(&disp_buf, buf1, buf2, LV_HOR_RES_MAX * 10);   /*Initialize the display buffer*/

    disp_drv.hor_res = 240;
    disp_drv.ver_res = 240;
    disp_drv.flush_cb = disp_flush;
    /*Set a display buffer*/
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);

#endif
#if 0
    esp_register_freertos_tick_hook(lv_tick_task);
#else
    startLvglTick();
#endif
}




/**********************
 *   STATIC FUNCTIONS
 **********************/
LV_DISP_FLUSH()
{
#if LV_VERSION_CHECK(5,3,0)
    tft.flush(x1, y1, x2, y2, (const uint16_t *)color_p);
    lv_flush_ready();
#elif LV_VERSION_CHECK(6,0,0)
    tft.flush(area->x1, area->y1, area->x2, area->y2, (const uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);
#endif
}



LV_TOUCH_READ()
{
    static TP_Point p;
    data->state = tp.touched() ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    if (data->state == LV_INDEV_STATE_PR) {
        p = tp.getPoint();
        p.x = map(p.x, 0, 320, 0, 240);
        p.y = map(p.y, 0, 320, 0, 240);
    }
    /*Set the coordinates (if released use the last pressed coordinates)*/
    data->point.x = p.x;
    data->point.y = p.y;
    return false; /*Return false because no moare to be read*/
}


#if USE_LV_FILESYSTEM || LV_USE_FILESYSTEM
/**
 * Open a file from the PC
 * @param file_p pointer to a FILE* variable
 * @param fn name of the file.
 * @param mode element of 'fs_mode_t' enum or its 'OR' connection (e.g. FS_MODE_WR | FS_MODE_RD)
 * @return LV_FS_RES_OK: no error, the file is opened
 *         any error from lv_fs_res_t enum
 */
// static lv_fs_res_t pcfs_open(void *file_p, const char *fn, lv_fs_mode_t mode)
LV_FS_OPEN()
{
    if (!_mount) return LV_FS_RES_UNKNOWN;
    const char *flags = "";
    if (mode == LV_FS_MODE_WR) flags = "wb";
    else if (mode == LV_FS_MODE_RD) flags = "rb";
    else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD)) flags = "a+";

    /*Make the path relative to the current directory (the projects root folder)*/
    char buf[256];
    sprintf(buf, "/sys/%s", fn);
    Serial.printf("Open %s \n", buf);
    FILE *f = fopen(buf, flags);
    if ((long int)f <= 0) return LV_FS_RES_UNKNOWN;
    else {
        fseek(f, 0, SEEK_SET);
        /* 'file_p' is pointer to a file descriptor and
         * we need to store our file descriptor here*/
        FILE **fp = (FILE **)file_p;         /*Just avoid the confusing casings*/
        *fp = f;
    }
    return LV_FS_RES_OK;
}


/**
 * Close an opened file
 * @param file_p pointer to a FILE* variable. (opened with lv_ufs_open)
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
// static lv_fs_res_t pcfs_close(void *file_p)
LV_FS_CLOSE()
{
    if (!_mount) return LV_FS_RES_UNKNOWN;
    FILE * *fp = (FILE * *)file_p;         /*Just avoid the confusing casings*/
    fclose(*fp);
    return LV_FS_RES_OK;
}

/**
 * Read data from an opened file
 * @param file_p pointer to a FILE variable.
 * @param buf pointer to a memory block where to store the read data
 * @param btr number of Bytes To Read
 * @param br the real number of read bytes (Byte Read)
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
// static lv_fs_res_t pcfs_read(void *file_p, void *buf, uint32_t btr, uint32_t *br)
LV_FS_READ()
{
    if (!_mount) return LV_FS_RES_UNKNOWN;
    FILE * *fp = (FILE * *)file_p;         /*Just avoid the confusing casings*/
    *br = fread(buf, 1, btr, *fp);
    return LV_FS_RES_OK;
}

/**
 * Set the read write pointer. Also expand the file size if necessary.
 * @param file_p pointer to a FILE* variable. (opened with lv_ufs_open )
 * @param pos the new position of read write pointer
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
// static lv_fs_res_t pcfs_seek(void *file_p, uint32_t pos)
LV_FS_SEEK()
{
    if (!_mount) return LV_FS_RES_UNKNOWN;
    FILE **fp = (FILE * *)file_p;         /*Just avoid the confusing casings*/
    fseek(*fp, pos, SEEK_SET);
    return LV_FS_RES_OK;
}

/**
 * Give the position of the read write pointer
 * @param file_p pointer to a FILE* variable.
 * @param pos_p pointer to to store the result
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
// static lv_fs_res_t pcfs_tell(void *file_p, uint32_t *pos_p)
LV_FS_TELL()
{
    if (!_mount) return LV_FS_RES_UNKNOWN;
    FILE * *fp = (FILE * *)file_p;         /*Just avoid the confusing casings*/
    *pos_p = ftell(*fp);
    return LV_FS_RES_OK;
}

#endif


void setupStorage()
{

}