///*********************************************************************************************************************
//* 文件名称          zf_device_oled_soft
//* 说明             SSD1306 OLED 软件 SPI 驱动 全部函数使用软件模拟 SPI 通信
//*                 函数名统一加 oled_soft_ 前缀
//* 适用平台          MSPM0G3507
//********************************************************************************************************************/

//#include "zf_common_debug.h"
//#include "zf_common_font.h"
//#include "zf_common_function.h"
//#include "zf_driver_delay.h"
//#include "zf_driver_spi.h"
//#include "zf_device_oled_soft.h"

////===================================================内部变量===================================================
//static soft_spi_info_struct           oled_soft_spi;
//static oled_soft_dir_enum             oled_soft_display_dir   = OLED_SOFT_DEFAULT_DISPLAY_DIR;
//static oled_soft_font_size_enum       oled_soft_display_font  = OLED_SOFT_DEFAULT_DISPLAY_FONT;

//#define oled_soft_spi_write_8bit(data) (soft_spi_write_8bit(&oled_soft_spi, (data)))

////===================================================内部函数===================================================
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     写数据
//// 参数说明     data            8 位数据
//// 返回参数     void
//// 备注信息     内部调用
////-------------------------------------------------------------------------------------------------------------------
//static void oled_soft_write_data (const uint8 data)
//{
//    OLED_SOFT_DC(1);
//    oled_soft_spi_write_8bit(data);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     写命令
//// 参数说明     command         8 位命令
//// 返回参数     void
//// 备注信息     内部调用
////-------------------------------------------------------------------------------------------------------------------
//static void oled_soft_write_command (const uint8 command)
//{
//    OLED_SOFT_DC(0);
//    oled_soft_spi_write_8bit(command);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     设置坐标
//// 参数说明     x               列地址 0~127
//// 参数说明     y               页地址 0~7
//// 返回参数     void
//// 备注信息     内部调用 超出范围会触发断言
////-------------------------------------------------------------------------------------------------------------------
//static void oled_soft_set_coordinate (uint8 x, uint8 y)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    oled_soft_write_command(0xb0 + y);
//    oled_soft_write_command(((x & 0xf0) >> 4) | 0x10);
//    oled_soft_write_command((x & 0x0f) | 0x00);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     DEBUG 输出初始化
//// 参数说明     void
//// 返回参数     void
//// 备注信息     内部调用
////-------------------------------------------------------------------------------------------------------------------
//static void oled_soft_debug_init (void)
//{
//    debug_output_struct info;
//    debug_output_struct_init(&info);
//    info.type_index = 1;
//    info.display_x_max = OLED_SOFT_X_MAX;
//    info.display_y_max = OLED_SOFT_Y_MAX;
//    switch(oled_soft_display_font)
//    {
//        case OLED_SOFT_6X8_FONT:
//        {
//            info.font_x_size = 6;
//            info.font_y_size = 1;
//        }break;
//        case OLED_SOFT_8X16_FONT:
//        {
//            info.font_x_size = 8;
//            info.font_y_size = 2;
//        }break;
//        case OLED_SOFT_16X16_FONT:
//        {
//            // 暂不支持
//        }break;
//    }
//    info.output_screen = oled_soft_show_string;
//    info.output_screen_clear = oled_soft_clear;
//    debug_output_init(&info);
//}

////===================================================公开函数===================================================
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     清屏
//// 参数说明     void
//// 返回参数     void
//// 使用示例     oled_soft_clear();
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_clear (void)
//{
//    uint8 y = 0, x = 0;
//    OLED_SOFT_CS(0);
//    for(y = 0; 8 > y; y ++)
//    {
//        oled_soft_write_command(0xb0 + y);
//        oled_soft_write_command(0x01);
//        oled_soft_write_command(0x10);
//        for(x = 0; OLED_SOFT_X_MAX > x; x ++)
//        {
//            oled_soft_write_data(0x00);
//        }
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     全屏填充
//// 参数说明     color           0x00 全黑 / 0xFF 全白
//// 返回参数     void
//// 使用示例     oled_soft_full(0x00);
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_full (const uint8 color)
//{
//    uint8 y = 0, x = 0;
//    OLED_SOFT_CS(0);
//    for(y = 0; 8 > y; y ++)
//    {
//        oled_soft_write_command(0xb0 + y);
//        oled_soft_write_command(0x01);
//        oled_soft_write_command(0x10);
//        for(x = 0; OLED_SOFT_X_MAX > x; x ++)
//        {
//            oled_soft_write_data(color);
//        }
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     设置显示方向
//// 参数说明     dir             方向 OLED_SOFT_PORTAIT / OLED_SOFT_PORTAIT_180
//// 返回参数     void
//// 备注信息     仅在 oled_soft_init 之前调用有效
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_set_dir (oled_soft_dir_enum dir)
//{
//    oled_soft_display_dir = dir;
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     设置显示字体
//// 参数说明     font            字体 OLED_SOFT_6X8_FONT / OLED_SOFT_8X16_FONT
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_set_font (oled_soft_font_size_enum font)
//{
//    oled_soft_display_font = font;
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     画点
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7（注意不是像素行，1 页 = 8 像素高）
//// 参数说明     color           8 位中哪一位置 1（0x01~0x80）
//// 返回参数     void
//// 使用示例     oled_soft_draw_point(0, 0, 0x01);
//// 备注信息     注意：此函数直接覆盖整字节，会清除同列其余 7 个像素
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_draw_point (uint16 x, uint16 y, const uint8 color)
//{
//    zf_assert(x < 128);
//    zf_assert(y < 8);
//    OLED_SOFT_CS(0);
//    oled_soft_set_coordinate((uint8)x, (uint8)y);
//    oled_soft_write_data(color);
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示字符串
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     ch[]            字符串
//// 返回参数     void
//// 使用示例     oled_soft_show_string(0, 0, "SEEKFREE");
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_string (uint16 x, uint16 y, const char ch[])
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    OLED_SOFT_CS(0);
//    uint8 c = 0, i = 0, j = 0;
//    while ('\0' != ch[j])
//    {
//        switch(oled_soft_display_font)
//        {
//            case OLED_SOFT_6X8_FONT:
//            {
//                c = ch[j] - 32;
//                if(x > 126)
//                {
//                    x = 0;
//                    y ++;
//                }
//                oled_soft_set_coordinate((uint8)x, (uint8)y);
//                for(i = 0; 6 > i; i ++)
//                {
//                    oled_soft_write_data(ascii_font_6x8[c][i]);
//                }
//                x += 6;
//                j ++;
//            }break;
//            case OLED_SOFT_8X16_FONT:
//            {
//                c = ch[j] - 32;
//                if(x > 120)
//                {
//                    x = 0;
//                    y ++;
//                }
//                oled_soft_set_coordinate((uint8)x, (uint8)y);
//                for(i = 0; 8 > i; i ++)
//                {
//                    oled_soft_write_data(ascii_font_8x16[c][i]);
//                }
//                oled_soft_set_coordinate((uint8)x, (uint8)(y + 1));
//                for(i = 0; 8 > i; i ++)
//                {
//                    oled_soft_write_data(ascii_font_8x16[c][i + 8]);
//                }
//                x += 8;
//                j ++;
//            }break;
//            case OLED_SOFT_16X16_FONT:
//            {
//                // 暂不支持
//            }break;
//        }
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示有符号整数
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     dat             数据 int32
//// 参数说明     num             显示位数 1~10
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_int (uint16 x, uint16 y, const int32 dat, uint8 num)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(0 < num);
//    zf_assert(10 >= num);
//    int32 dat_temp = dat;
//    int32 offset = 1;
//    char data_buffer[12];
//    memset(data_buffer, 0, 12);
//    memset(data_buffer, ' ', num + 1);
//    if(10 > num)
//    {
//        for(; 0 < num; num --)
//        {
//            offset *= 10;
//        }
//        dat_temp %= offset;
//    }
//    func_int_to_str(data_buffer, dat_temp);
//    oled_soft_show_string(x, y, (const char *)&data_buffer);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示无符号整数
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     dat             数据 uint32
//// 参数说明     num             显示位数 1~10
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_uint (uint16 x, uint16 y, const uint32 dat, uint8 num)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(0 < num);
//    zf_assert(10 >= num);
//    uint32 dat_temp = dat;
//    int32 offset = 1;
//    char data_buffer[12];
//    memset(data_buffer, 0, 12);
//    memset(data_buffer, ' ', num);
//    if(10 > num)
//    {
//        for(; 0 < num; num --)
//        {
//            offset *= 10;
//        }
//        dat_temp %= offset;
//    }
//    func_uint_to_str(data_buffer, dat_temp);
//    oled_soft_show_string(x, y, (const char *)&data_buffer);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示浮点数
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     dat             数据 double
//// 参数说明     num             整数位数 1~8
//// 参数说明     pointnum        小数位数 1~6
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_float (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(0 < num);
//    zf_assert(8 >= num);
//    zf_assert(0 < pointnum);
//    zf_assert(6 >= pointnum);
//    double dat_temp = dat;
//    double offset = 1.0;
//    char data_buffer[17];
//    memset(data_buffer, 0, 17);
//    memset(data_buffer, ' ', num + pointnum + 2);
//    for(; 0 < num; num --)
//    {
//        offset *= 10;
//    }
//    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
//    func_double_to_str(data_buffer, dat_temp, pointnum);
//    oled_soft_show_string(x, y, data_buffer);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示二值图像
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     *image          图像数组（每 8 个纵向像素压缩为 1 字节）
//// 参数说明     width           图像实际宽度
//// 参数说明     height          图像实际高度
//// 参数说明     dis_width       显示宽度  [0, 128]
//// 参数说明     dis_height      显示高度  [0, 64]
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_binary_image (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(NULL != image);
//    uint32 i = 0, j = 0, z = 0;
//    uint8 dat = 0;
//    uint32 width_index = 0, height_index = 0;
//    OLED_SOFT_CS(0);
//    dis_height = dis_height - dis_height % 8;
//    dis_width  = dis_width  - dis_width  % 8;
//    for(j = 0; j < dis_height; j += 8)
//    {
//        oled_soft_set_coordinate(x + 0, (uint16)(y + j / 8));
//        height_index = j * height / dis_height;
//        for(i = 0; i < dis_width; i += 8)
//        {
//            width_index = i * width / dis_width / 8;
//            for(z = 0; 8 > z; z ++)
//            {
//                dat = 0;
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 0) & (0x80 >> z)) { dat |= 0x01; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 1) & (0x80 >> z)) { dat |= 0x02; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 2) & (0x80 >> z)) { dat |= 0x04; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 3) & (0x80 >> z)) { dat |= 0x08; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 4) & (0x80 >> z)) { dat |= 0x10; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 5) & (0x80 >> z)) { dat |= 0x20; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 6) & (0x80 >> z)) { dat |= 0x40; }
//                if(*(image + height_index * width / 8 + width_index + width / 8 * 7) & (0x80 >> z)) { dat |= 0x80; }
//                oled_soft_write_data(dat);
//            }
//        }
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示 8bit 灰度图像（可二值化）
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     *image          图像数组（每像素 1 字节灰度）
//// 参数说明     width           图像实际宽度
//// 参数说明     height          图像实际高度
//// 参数说明     dis_width       显示宽度  [0, 128]
//// 参数说明     dis_height      显示高度  [0, 64]
//// 参数说明     threshold       二值化阈值 0=不二值化
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_gray_image (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(NULL != image);
//    int16 i = 0, j = 0;
//    uint8 dat = 0;
//    uint32 width_index = 0, height_index = 0;
//    OLED_SOFT_CS(0);
//    dis_height = dis_height - dis_height % 8;
//    for(j = 0; j < dis_height; j += 8)
//    {
//        oled_soft_set_coordinate(x + 0, y + j / 8);
//        height_index = j * height / dis_height;
//        for(i = 0; i < dis_width; i ++)
//        {
//            width_index = i * width / dis_width;
//            dat = 0;
//            if(*(image + height_index * width + width_index + width * 0) > threshold) { dat |= 0x01; }
//            if(*(image + height_index * width + width_index + width * 1) > threshold) { dat |= 0x02; }
//            if(*(image + height_index * width + width_index + width * 2) > threshold) { dat |= 0x04; }
//            if(*(image + height_index * width + width_index + width * 3) > threshold) { dat |= 0x08; }
//            if(*(image + height_index * width + width_index + width * 4) > threshold) { dat |= 0x10; }
//            if(*(image + height_index * width + width_index + width * 5) > threshold) { dat |= 0x20; }
//            if(*(image + height_index * width + width_index + width * 6) > threshold) { dat |= 0x40; }
//            if(*(image + height_index * width + width_index + width * 7) > threshold) { dat |= 0x80; }
//            oled_soft_write_data(dat);
//        }
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示波形
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     *wave           波形数据数组
//// 参数说明     width           波形实际宽度
//// 参数说明     value_max       波形最大值
//// 参数说明     dis_width       显示宽度  [0, 128]
//// 参数说明     dis_value_max   显示高度  [0, 64]
//// 返回参数     void
//// 备注信息     注意：会清除同列其余像素
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_wave (uint16 x, uint16 y, const uint16 *wave, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(NULL != wave);
//    uint32 i = 0;
//    uint32 width_index = 0, value_max_index = 0;
//    uint8 dis_h = 0;
//    uint32 x_temp = 0;
//    uint32 y_temp = 0;
//    OLED_SOFT_CS(0);
//    for(y_temp = 0; y_temp < dis_value_max; y_temp += 8)
//    {
//        oled_soft_set_coordinate(x + 0, (uint16)(y + y_temp / 8));
//        for(x_temp = 0; x_temp < dis_width; x_temp ++)
//        {
//            oled_soft_write_data(0x00);
//        }
//    }
//    for(i = 0; i < dis_width; i ++)
//    {
//        width_index = i * width / dis_width;
//        value_max_index = *(wave + width_index) * (dis_value_max - 1) / value_max;
//        dis_h = (uint8)((dis_value_max - 1) - value_max_index);
//        oled_soft_set_coordinate((uint16)(i + x), dis_h / 8 + y);
//        dis_h = (0x01 << dis_h % 8);
//        oled_soft_write_data(dis_h);
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     显示汉字
//// 参数说明     x               列坐标 0~127
//// 参数说明     y               页坐标 0~7
//// 参数说明     size            字号（取模时的点阵大小，如 16）
//// 参数说明     *chinese_buffer 字模数组
//// 参数说明     number          汉字个数
//// 返回参数     void
//// 备注信息     PCtoLCD2002 取模：阴码、逐行式、顺向
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_show_chinese (uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number)
//{
//    zf_assert(128 > x);
//    zf_assert(8 > y);
//    zf_assert(NULL != chinese_buffer);
//    int16 i = 0, j = 0, k = 0;
//    OLED_SOFT_CS(0);
//    for(i = 0; i < number; i ++)
//    {
//        for(j = 0; j < (size / 8); j ++)
//        {
//            oled_soft_set_coordinate(x + i * size, y + j);
//            for(k = 0; 16 > k; k ++)
//            {
//                oled_soft_write_data(*chinese_buffer);
//                chinese_buffer ++;
//            }
//        }
//    }
//    OLED_SOFT_CS(1);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     OLED 初始化
//// 参数说明     void
//// 返回参数     void
//// 使用示例     oled_soft_init();
//// 备注信息     使用软件 SPI 驱动 SSD1306
////-------------------------------------------------------------------------------------------------------------------
//void oled_soft_init (void)
//{
//    // 软件 SPI 初始化
//    soft_spi_init(&oled_soft_spi, 0, OLED_SOFT_SPI_DELAY, OLED_SOFT_SCK_PIN, OLED_SOFT_MOSI_PIN, SOFT_SPI_PIN_NULL, SOFT_SPI_PIN_NULL);

//    // GPIO 初始化
//    gpio_init(OLED_SOFT_RES_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    gpio_init(OLED_SOFT_DC_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    gpio_init(OLED_SOFT_CS_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);

//    oled_soft_set_dir(oled_soft_display_dir);

//    // 硬件复位
//    OLED_SOFT_CS(0);
//    OLED_SOFT_RES(0);
//    system_delay_ms(50);
//    OLED_SOFT_RES(1);

//    // SSD1306 初始化命令序列
//    oled_soft_write_command(0xae);      // 关闭显示

//    oled_soft_write_command(0x00);      // 低列地址 = 0
//    oled_soft_write_command(0x10);      // 高列地址 = 0

//    oled_soft_write_command(0x40);      // 起始行地址 = 0

//    oled_soft_write_command(0x81);      // 对比度控制
//    oled_soft_write_command(OLED_SOFT_BRIGHTNESS);

//    if(OLED_SOFT_PORTAIT == oled_soft_display_dir)
//    {
//        oled_soft_write_command(0xa1);  // 列映射：正常（左→右）
//        oled_soft_write_command(0xc8);  // 行扫描：正常（上→下）
//    }
//    else
//    {
//        oled_soft_write_command(0xa0);  // 列映射：反置
//        oled_soft_write_command(0xc0);  // 行扫描：反置
//    }

//    oled_soft_write_command(0xa6);      // 正常显示（非反色）

//    oled_soft_write_command(0xa8);      // 设置 MUX 比
//    oled_soft_write_command(0x3f);      // 1/64

//    oled_soft_write_command(0xd3);      // 显示偏移
//    oled_soft_write_command(0x00);      // 不偏移

//    oled_soft_write_command(0xd5);      // 时钟分频 / 振荡频率
//    oled_soft_write_command(0x80);

//    oled_soft_write_command(0xd9);      // 预充电周期
//    oled_soft_write_command(0xf1);

//    oled_soft_write_command(0xda);      // COM 引脚硬件配置
//    oled_soft_write_command(0x12);

//    oled_soft_write_command(0xdb);      // VCOMH 电平
//    oled_soft_write_command(0x40);

//    oled_soft_write_command(0x20);      // 寻址模式
//    oled_soft_write_command(0x02);      // 页寻址模式

//    oled_soft_write_command(0x8d);      // 充电泵设置
//    oled_soft_write_command(0x14);      // 启用充电泵（3.3V 供电必须）

//    oled_soft_write_command(0xa4);      // 正常显示模式（非全屏点亮）

//    oled_soft_write_command(0xa6);      // 非反色显示

//    oled_soft_write_command(0xaf);      // 打开显示

//    OLED_SOFT_CS(1);

//    oled_soft_clear();
//    oled_soft_set_coordinate(0, 0);
//    oled_soft_debug_init();
//}
