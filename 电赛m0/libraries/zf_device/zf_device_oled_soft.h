///*********************************************************************************************************************
//* 文件名称          zf_device_oled_soft
//* 说明             SSD1306 OLED 软件 SPI 驱动（独立于硬件 SPI 版本）
//*                 函数名统一加 oled_soft_ 前缀，与硬件版 oled_ 区分
//* 适用平台          MSPM0G3507
//********************************************************************************************************************/

//#ifndef _zf_device_oled_soft_h_
//#define _zf_device_oled_soft_h_

//#include "zf_common_typedef.h"

////===================================================软件 SPI 引脚配置=================================================
//#define OLED_SOFT_SPI_DELAY             (10)                                    // 软件 SPI 延时 数值越小通信越快 建议 5~50
//#define OLED_SOFT_SCK_PIN               (A12)                                   // 软件 SPI SCK 引脚
//#define OLED_SOFT_MOSI_PIN              (A9)                                    // 软件 SPI MOSI 引脚
////===================================================软件 SPI 引脚配置=================================================

//#define OLED_SOFT_RES_PIN               (A7)                                    // 复位引脚
//#define OLED_SOFT_DC_PIN                (A15)                                   // 命令/数据选择引脚
//#define OLED_SOFT_CS_PIN                (A8)                                    // 片选引脚
//#define OLED_SOFT_BRIGHTNESS            (0x7F)                                  // 亮度 0x00~0xFF 越大越亮

//#define OLED_SOFT_DEFAULT_DISPLAY_DIR   (OLED_SOFT_PORTAIT)                     // 默认竖屏
//#define OLED_SOFT_DEFAULT_DISPLAY_FONT  (OLED_SOFT_6X8_FONT)                    // 默认 6x8 字体
//#define OLED_SOFT_X_MAX                 (128)                                   // X 轴最大像素
//#define OLED_SOFT_Y_MAX                 (64)                                    // Y 轴最大像素

//#define OLED_SOFT_RES(x)                ((x) ? (gpio_high(OLED_SOFT_RES_PIN)) : (gpio_low(OLED_SOFT_RES_PIN)))
//#define OLED_SOFT_DC(x)                 ((x) ? (gpio_high(OLED_SOFT_DC_PIN))  : (gpio_low(OLED_SOFT_DC_PIN)))
//#define OLED_SOFT_CS(x)                 ((x) ? (gpio_high(OLED_SOFT_CS_PIN))  : (gpio_low(OLED_SOFT_CS_PIN)))


//typedef enum
//{
//    OLED_SOFT_PORTAIT                   = 0,                                    // 竖屏
//    OLED_SOFT_PORTAIT_180               = 1,                                    // 竖屏旋转 180°
//} oled_soft_dir_enum;

//typedef enum
//{
//    OLED_SOFT_6X8_FONT                  = 0,                                    // 6x8  字体
//    OLED_SOFT_8X16_FONT                 = 1,                                    // 8x16 字体
//    OLED_SOFT_16X16_FONT                = 2,                                    // 16x16 暂不支持
//} oled_soft_font_size_enum;


//void oled_soft_init                 (void);
//void oled_soft_clear                (void);
//void oled_soft_full                 (const uint8 color);
//void oled_soft_set_dir              (oled_soft_dir_enum dir);
//void oled_soft_set_font             (oled_soft_font_size_enum font);
//void oled_soft_draw_point           (uint16 x, uint16 y, const uint8 color);

//void oled_soft_show_string          (uint16 x, uint16 y, const char ch[]);
//void oled_soft_show_int             (uint16 x, uint16 y, const int32 dat, uint8 num);
//void oled_soft_show_uint            (uint16 x, uint16 y, const uint32 dat, uint8 num);
//void oled_soft_show_float           (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);

//void oled_soft_show_binary_image    (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
//void oled_soft_show_gray_image      (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold);
//void oled_soft_show_wave            (uint16 x, uint16 y, const uint16 *wave, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max);
//void oled_soft_show_chinese         (uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number);

//#endif
