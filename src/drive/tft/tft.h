#ifndef __TFT_H
#define __TFT_H


// New color definitions use for all my libraries
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

void setupDisplay();
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void setRotation(uint8_t r);
void lcd_sleep();
void lcd_wakeup();
void lcd_init(int miso, int mosi, int sclk, int cs, int dc);
void lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint16_t *color_p);

#ifdef __cplusplus
class TTGO_TFT
{
public:
    TTGO_TFT(int miso, int mosi, int sclk, int cs, int dc)
    {
        _miso = miso;
        _mosi = mosi;
        _sclk = sclk;
        _cs = cs;
        _dc = dc;
    };
    ~TTGO_TFT() {};
    void begin()
    {
        lcd_init(_miso, _mosi, _sclk, _cs, _dc);
    };
    void sleep()
    {
        lcd_sleep();
    };
    void wakeup()
    {
        lcd_wakeup();
    };
    void setRotation(uint8_t r)
    {
        setRotation(r);
    };
    void fillScreen(uint16_t color)
    {
        fillRect(0, 0, 240, 240, color);
    };
    void flush(int32_t x, int32_t y, int32_t x1, int32_t y1, const uint16_t *buf)
    {
        lcd_flush(x, y, x1, y1, buf);
    }
private:
    int _miso, _mosi, _sclk, _cs, _dc;
};
#endif
#endif
