
#include <Arduino.h>
#include "ST7789_2_Defines.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "../../lvgl/lvgl.h"
#include "../../board_def.h"
#include "esp_freertos_hooks.h"

#define MAKEWORD(b1, b2, b3, b4) ((uint32_t)((b1) | ((b2) << 8) | ((b3) << 16) | ((b4) << 24)))
#define SWAPBYTES(i) ((i >> 8) | (i << 8))

static void __lcd_transfer_cb(spi_transaction_t *t);
static esp_err_t __lcd_spi_send(spi_device_handle_t spi, spi_transaction_t *t);
static void __lcd_cmd(const uint8_t cmd);
static void __lcd_data(const uint8_t *data, int len);
static void __lcd_data(const uint8_t data);

static void __transmitCmdData(uint8_t cmd, uint32_t data);
static void __transmitCmd(uint8_t cmd);
static void __transmitData(uint8_t *data, int length);
static void __setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static void __fastSendBuf(const uint16_t *buf, int point_num, bool swap);
static void __fastSendRep(uint16_t val, int rep_num);
static void __disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p);
static void lv_tick_task(void);
void lcd_init(int miso, int mosi, int sclk, int cs, int dc);

static SemaphoreHandle_t spi_mux;
static SemaphoreHandle_t _spi_mux = NULL;
static spi_device_handle_t spi_wr = NULL;
static int dma_buf_size = 1024;
static int __dc;


static void lv_tick_task(void)
{
    // lv_tick_inc(portTICK_RATE_MS);
}

void setupDisplay()
{
    // lcd_init(TFT_MISO, TFT_MOSI, TFT_SCLK, TFT_CS, TFT_DC);
    // lv_init();
    // /*Initialize the display*/
    // lv_disp_drv_t disp_drv;
    // lv_disp_drv_init(&disp_drv);
    // disp_drv.disp_flush = __disp_flush; /*Used in buffered mode (LV_VDB_SIZE != 0  in lv_conf.h)*/
    // lv_disp_drv_register(&disp_drv);
    // esp_register_freertos_tick_hook(lv_tick_task);
}

void lcd_init(int miso, int mosi, int sclk, int cs, int dc)
{
    int dma_chan = 1;
    spi_host_device_t spi_host = VSPI_HOST;
    spi_mux = xSemaphoreCreateRecursiveMutex();
    _spi_mux = xSemaphoreCreateMutex();

    //Initialize non-SPI GPIOs
    __dc = dc;
    gpio_pad_select_gpio(dc);
    gpio_set_direction((gpio_num_t)dc, GPIO_MODE_OUTPUT);
    printf("miso:%d mosi:%d sclk:%d cs:%d dc:%d\n", miso, mosi, sclk, cs, __dc);
    //Initialize SPI Bus for LCD
    spi_bus_config_t buscfg = {0};

    buscfg.miso_io_num = (gpio_num_t)miso;
    buscfg.mosi_io_num = (gpio_num_t)mosi;
    buscfg.sclk_io_num = (gpio_num_t)sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    spi_bus_initialize(spi_host, &buscfg, dma_chan);

    spi_device_interface_config_t devcfg = {0};
    // devcfg.clock_speed_hz = SPI_MASTER_FREQ_26M; //Clock out frequency
    devcfg.clock_speed_hz = SPI_MASTER_FREQ_40M; //Clock out frequency
    devcfg.mode = 0;                             //SPI mode 0
    devcfg.spics_io_num = cs;                    //CS pin
    devcfg.queue_size = 7;                       //We want to be able to queue 7 transactions at a time
    devcfg.pre_cb = __lcd_transfer_cb;           //Specify pre-transfer callback to handle D/C line
    devcfg.flags = SPI_DEVICE_NO_DUMMY;
    spi_bus_add_device(spi_host, &devcfg, &spi_wr);

    __lcd_cmd(ST7789_SLPOUT); // Sleep out
    delay(120);

    __lcd_cmd(ST7789_NORON); // Normal display mode on

    //------------------------------display and color format setting--------------------------------//
    __lcd_cmd(ST7789_MADCTL);
    //__lcd_data(0x00);
    __lcd_data(TFT_MAD_COLOR_ORDER);

    // JLX240 display datasheet
    __lcd_cmd(0xB6);
    __lcd_data(0x0A);
    __lcd_data(0x82);

    __lcd_cmd(ST7789_COLMOD);
    __lcd_data(0x55);
    delay(10);

    //--------------------------------ST7789V Frame rate setting----------------------------------//
    __lcd_cmd(ST7789_PORCTRL);
    __lcd_data(0x0c);
    __lcd_data(0x0c);
    __lcd_data(0x00);
    __lcd_data(0x33);
    __lcd_data(0x33);

    __lcd_cmd(ST7789_GCTRL); // Voltages: VGH / VGL
    __lcd_data(0x35);

    //---------------------------------ST7789V Power setting--------------------------------------//
    __lcd_cmd(ST7789_VCOMS);
    __lcd_data(0x28); // JLX240 display datasheet

    __lcd_cmd(ST7789_LCMCTRL);
    __lcd_data(0x0C);

    __lcd_cmd(ST7789_VDVVRHEN);
    __lcd_data(0x01);
    __lcd_data(0xFF);

    __lcd_cmd(ST7789_VRHS); // voltage VRHS
    __lcd_data(0x10);

    __lcd_cmd(ST7789_VDVSET);
    __lcd_data(0x20);

    __lcd_cmd(ST7789_FRCTR2);
    __lcd_data(0x0f);

    __lcd_cmd(ST7789_PWCTRL1);
    __lcd_data(0xa4);
    __lcd_data(0xa1);

    //--------------------------------ST7789V gamma setting---------------------------------------//
    __lcd_cmd(ST7789_PVGAMCTRL);
    __lcd_data(0xd0);
    __lcd_data(0x00);
    __lcd_data(0x02);
    __lcd_data(0x07);
    __lcd_data(0x0a);
    __lcd_data(0x28);
    __lcd_data(0x32);
    __lcd_data(0x44);
    __lcd_data(0x42);
    __lcd_data(0x06);
    __lcd_data(0x0e);
    __lcd_data(0x12);
    __lcd_data(0x14);
    __lcd_data(0x17);

    __lcd_cmd(ST7789_NVGAMCTRL);
    __lcd_data(0xd0);
    __lcd_data(0x00);
    __lcd_data(0x02);
    __lcd_data(0x07);
    __lcd_data(0x0a);
    __lcd_data(0x28);
    __lcd_data(0x31);
    __lcd_data(0x54);
    __lcd_data(0x47);
    __lcd_data(0x0e);
    __lcd_data(0x1c);
    __lcd_data(0x17);
    __lcd_data(0x1b);
    __lcd_data(0x1e);

    __lcd_cmd(ST7789_INVON);

    __lcd_cmd(ST7789_CASET); // Column address set
    __lcd_data(0x00);
    __lcd_data(0x00);
    __lcd_data(0x00);
    __lcd_data(0xE5); // 239

    __lcd_cmd(ST7789_RASET); // Row address set
    __lcd_data(0x00);
    __lcd_data(0x00);
    __lcd_data(0x01);
    __lcd_data(0x3F); // 319

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    delay(120);

    __lcd_cmd(ST7789_DISPON); //Display on

    printf("Init Done\n");
}

static void __lcd_transfer_cb(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level((gpio_num_t)__dc, dc);
}

static esp_err_t __lcd_spi_send(spi_device_handle_t spi, spi_transaction_t *t)
{
    xSemaphoreTake(_spi_mux, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(spi, t); //Transmit!
    xSemaphoreGive(_spi_mux);
    return res;
}

static void __lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    int dc = 0;
    spi_transaction_t t = {0};
    t.length = 8;        // Command is 8 bits
    t.tx_buffer = &cmd;  // The data is the cmd itself
    t.user = (void *)dc; // D/C needs to be set to 0
    // ret = _lcd_spi_send(spi_wr, &t);       // Transmit!
    ret = spi_device_transmit(spi_wr, &t); //Transmit!
    assert(ret == ESP_OK);                 // Should have had no issues.
}

static void __lcd_data(const uint8_t data)
{
    uint8_t d = data;
    __lcd_data(&d, 1);
}

static void __lcd_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    if (len == 0) {
        return; //no need to send anything
    }
    int dc = 1;
    spi_transaction_t t = {0};
    t.length = len * 8;  // Len is in bytes; transaction length is in bitst.
    t.tx_buffer = data;  // Data
    t.user = (void *)dc; // D/C needs to be set to 1
    // ret = _lcd_spi_send(spi_wr, &t);       // Transmit!
    ret = spi_device_transmit(spi_wr, &t); //Transmit!
    assert(ret == ESP_OK);                 // Should have had no issues.
}

static void __transmitCmdData(uint8_t cmd, uint32_t data)
{
    xSemaphoreTakeRecursive(spi_mux, portMAX_DELAY);
    __lcd_cmd(cmd);
    __lcd_data((uint8_t *)&data, 4);
    xSemaphoreGiveRecursive(spi_mux);
}

static void __transmitCmd(uint8_t cmd)
{
    xSemaphoreTakeRecursive(spi_mux, portMAX_DELAY);
    __lcd_cmd(cmd);
    xSemaphoreGiveRecursive(spi_mux);
}

static void __setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    xSemaphoreTakeRecursive(spi_mux, portMAX_DELAY);
    __transmitCmdData(ST7789_CASET, MAKEWORD(x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF));
    __transmitCmdData(ST7789_RASET, MAKEWORD(y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF));
    __transmitCmd(ST7789_RAMWR); // write to RAM
    xSemaphoreGiveRecursive(spi_mux);
}

static void __transmitData(uint8_t *data, int length)
{
    xSemaphoreTakeRecursive(spi_mux, portMAX_DELAY);
    __lcd_data((uint8_t *)data, length);
    xSemaphoreGiveRecursive(spi_mux);
}

static void __fastSendBuf(const uint16_t *buf, int point_num, bool swap)
{
    if ((point_num * sizeof(uint16_t)) <= (16 * sizeof(uint32_t))) {
        __transmitData((uint8_t *)buf, sizeof(uint16_t) * point_num);
    } else {
        int gap_point = dma_buf_size;
        uint16_t *data_buf = (uint16_t *)malloc(gap_point * sizeof(uint16_t));
        int offset = 0;
        while (point_num > 0) {
            int trans_points = point_num > gap_point ? gap_point : point_num;

            if (swap) {
                for (int i = 0; i < trans_points; i++) {
                    data_buf[i] = SWAPBYTES(buf[i + offset]);
                }
            } else {
                memcpy((uint8_t *)data_buf, (uint8_t *)(buf + offset), trans_points * sizeof(uint16_t));
            }
            __transmitData((uint8_t *)(data_buf), trans_points * sizeof(uint16_t));
            offset += trans_points;
            point_num -= trans_points;
        }
        free(data_buf);
        data_buf = NULL;
    }
}

static void __fastSendRep(uint16_t val, int rep_num)
{
    int point_num = rep_num;
    int gap_point = dma_buf_size;
    gap_point = (gap_point > point_num ? point_num : gap_point);

    uint16_t *data_buf = (uint16_t *)malloc(gap_point * sizeof(uint16_t));
    int offset = 0;
    while (point_num > 0) {
        for (int i = 0; i < gap_point; i++) {
            data_buf[i] = val;
        }
        int trans_points = point_num > gap_point ? gap_point : point_num;
        __transmitData((uint8_t *)(data_buf), sizeof(uint16_t) * trans_points);
        offset += trans_points;
        point_num -= trans_points;
    }
    free(data_buf);
    data_buf = NULL;
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    // rudimentary clipping (drawChar w/big text requires this)
    if ((x >= TFT_WIDTH) || (y >= TFT_HEIGHT)) {
        return;
    }
    if ((x + w - 1) >= TFT_WIDTH) {
        w = TFT_WIDTH - x;
    }
    if ((y + h - 1) >= TFT_HEIGHT) {
        h = TFT_HEIGHT - y;
    }
    xSemaphoreTakeRecursive(spi_mux, portMAX_DELAY);
    __setAddrWindow(x, y, x + w - 1, y + h - 1);
    __fastSendRep(SWAPBYTES(color), h * w);
    xSemaphoreGiveRecursive(spi_mux);
}

void setRotation(uint8_t r)
{
    uint8_t data;
    __lcd_cmd(TFT_MADCTL);
    switch (r % 4) {
    case 0: // Portrait
        __lcd_data(TFT_MAD_RGB);
        break;
    case 1: // Landscape (Portrait + 90)
        __lcd_data(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_RGB);
        break;
    case 2: // Inverter portrait
        __lcd_data(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_RGB);
        break;
    case 3: // Inverted landscape
        __lcd_data(TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_RGB);
        break;
    }
}

void lcd_sleep()
{
    __lcd_cmd(TFT_SLPIN);
}

void lcd_wakeup()
{
    __lcd_cmd(TFT_SLPOUT);
}

void lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint16_t *color_p)
{
    // printf("x:%d x1:%d y:%d y1:%d size:%u\n", x1, x2, y1, y2, size);
    __setAddrWindow(x1, y1, x2, y2);
    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);
    __fastSendBuf(color_p, size, false);
}

static void __disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p)
{
    __setAddrWindow(x1, y1, x2, y2);
    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);
    __fastSendBuf((const uint16_t *)color_p, size, false);
    // lv_flush_ready();
}