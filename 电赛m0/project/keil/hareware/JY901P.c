#include "zf_common_headfile.h"

//#include "JY901P.h"

///* ========== 管脚定义 ========== */
//#define JY901_UART_TX       UART1_TX_B4
//#define JY901_UART_RX       UART1_RX_B5

///* ========== 波特率 ========== */
//#define JY901_BAUD         57600 

///* ========== 协议常量 ========== */
//#define JY901_HEADER        0x55
//#define JY901_TYPE_ANGLE    0x53

///* ========== 换算系数 ========== */
//#define JY901_ANGLE_SCALE   (180.0f / 32768.0f)

///* ========== 缓冲区大小 ========== */
//#define JY901_RX_BUF_SIZE   11

///* ========== 接收缓冲区 ========== */
//static uint8  rx_buf[JY901_RX_BUF_SIZE];
//static uint8  rx_index = 0;

///* ========== 角度输出 ========== */
//float gyro_roll  = 0.0f;
//float gyro_pitch = 0.0f;
//float gyro_yaw   = 0.0f;

///* ========== 帧就绪标志（中断 → 主循环） ========== */
//static volatile uint8 rx_frame_ready = 0;

///* ========== 诊断计数器 ========== */
//static volatile uint32_t diag_rx_bytes   = 0;
//static volatile uint32_t diag_headers    = 0;
//static volatile uint32_t diag_good       = 0;

///* ========== 前向声明 ========== */
//static void jy901s_callback(uint32 event, void *ptr);
//static void parse_angle_packet(void);
//static volatile uint8 diag_last_byte = 0;
///* ================================================================
// * uart_gyro_init  —— 初始化 UART1，注册接收中断
// * ================================================================ */
//void uart_gyro_init(void)
//{
//    //uart_init(UART_1, JY901_BAUD, JY901_UART_TX, JY901_UART_RX);
//	  uart1_mfclk_init(9600);
//    uart_set_callback(UART_1, jy901s_callback, NULL);
//    uart_set_interrupt_config(UART_1, UART_INTERRUPT_CONFIG_RX_ENABLE);
//}

///* ================================================================
// * jy901s_callback  —— 串口中断回调（每个字节触发一次）
// * ================================================================ */
//static void jy901s_callback(uint32 event, void *ptr)
//{
//    uint8 data;
//     
//    /* event 是中断状态标志，不是数据 */
//    if (!(event & UART_INTERRUPT_STATE_RX))
//        return;

//    /* 从 UART 接收寄存器读出真正的数据字节 */
//    if (!uart_query_byte(UART_1, &data))
//        return;
//    diag_last_byte = data;
//    diag_rx_bytes++;

//    /* 收到 0x55 包头 → 开始存新帧 */
//    if (data == JY901_HEADER) {
//        rx_index = 0;
//        diag_headers++;
//    }

//    if (rx_index < JY901_RX_BUF_SIZE) {
//        rx_buf[rx_index++] = data;
//    }

//    /* 收满 11 字节，若为首尾正确的角度包则通知主循环 */
//    if (rx_index >= JY901_RX_BUF_SIZE) {
//        if (rx_buf[0] == JY901_HEADER && rx_buf[1] == JY901_TYPE_ANGLE) {
//            rx_frame_ready = 1;
//        }
//        rx_index = 0;
//    }
//}

///* ================================================================
// * parse_angle_packet  —— 解析 11 字节角度包
// *
// *  字节布局: [0]=0x55 [1]=0x53
// *            [2]=RollL  [3]=RollH
// *            [4]=PitchL [5]=PitchH
// *            [6]=YawL   [7]=YawH
// *            [8]=TL [9]=TH [10]=SUM
// * ================================================================ */
//static void parse_angle_packet(void)
//{
//    int16 roll_raw  = (int16)(((uint16)rx_buf[3] << 8) | rx_buf[2]);
//    int16 pitch_raw = (int16)(((uint16)rx_buf[5] << 8) | rx_buf[4]);
//    int16 yaw_raw   = (int16)(((uint16)rx_buf[7] << 8) | rx_buf[6]);

//    gyro_roll  = (float)roll_raw  * JY901_ANGLE_SCALE;
//    gyro_pitch = (float)pitch_raw * JY901_ANGLE_SCALE;
//    gyro_yaw   = (float)yaw_raw   * JY901_ANGLE_SCALE;

//    diag_good++;
//}

///* ================================================================
// * JY901P_Controll  —— 主循环中调用，处理帧
// * ================================================================ */
//void JY901P_Controll(void)
//{
//    if (rx_frame_ready) {
//        rx_frame_ready = 0;
//        parse_angle_packet();
//    }
//}

///* ================================================================
// * JY901P_DiagDisplay  —— OLED 诊断显示
// *
// *   第 0 行: RX 收到的总字节数
// *   第 1 行: H 包头次数  G 成功解析次数
// *   第 3~4 行: Roll / Pitch / Yaw
// * ================================================================ */
//void JY901P_DiagDisplay(void)
//{
//    char str[20];

//    sprintf(str, "RX:%-6u",  (unsigned int)diag_rx_bytes);
//    tft180_show_string(0, 0, str);

//    sprintf(str, "H:%-4u G:%-4u", (unsigned int)diag_headers, (unsigned int)diag_good);
//    tft180_show_string(0, 16, str);

//    tft180_show_string(0, 32, "Roll  Pitch  Yaw");
//    tft180_show_float(0, 48, gyro_roll,  3, 1);
//    tft180_show_float(16, 48, gyro_pitch, 3, 1);
//    tft180_show_float(32, 48, gyro_yaw,  3, 1);
//     sprintf(str, "last:0x%02X", diag_last_byte);
//      tft180_show_string(0, 67, str);
//}
//float angle_diff(float cur, float tgt)
//{
//    float d = cur - tgt;
//    while (d >  180.0f) d -= 360.0f;
//    while (d < -180.0f) d += 360.0f;
//    return d;
//}
#include "JY901P.h"

/* ========== 硬件 ========== */
#define JY901_UART_TX       UART1_TX_B4
#define JY901_UART_RX       UART1_RX_B5
#define JY901_BAUD          9600

/* ========== 协议 ========== */
#define JY901_HEADER        0x55
#define JY901_TYPE_ANGLE    0x53
#define JY901_RX_BUF_SIZE   11
#define JY901_ANGLE_SCALE   (180.0f / 32768.0f)

/* ========== 输出 ========== */
float gyro_roll  = 0.0f;
float gyro_pitch = 0.0f;
float gyro_yaw   = 0.0f;

/* ========== 接收缓冲 ========== */
static uint8_t  rx_buf[JY901_RX_BUF_SIZE];
static uint8_t  rx_index = 0;
static volatile uint8_t rx_frame_ready = 0;

/* ========== 诊断 ========== */
static volatile uint32_t diag_bytes  = 0;
static volatile uint32_t diag_header = 0;
static volatile uint32_t diag_good   = 0;
static volatile uint8_t  diag_last   = 0;

/* ========== 前向声明 ========== */
static void callback(uint32_t event, void *ptr);
static void parse_angle(void);

/* ================================================================
 * uart_gyro_init
 * ================================================================ */
void uart_gyro_init(void)
{
    uart1_mfclk_init(9600);
    uart_set_callback(UART_1, callback, NULL);
    uart_set_interrupt_config(UART_1, UART_INTERRUPT_CONFIG_RX_ENABLE);

    /* ---------- 切换 6 轴模式（注释掉即用出厂 9 轴） ---------- */
    uint8_t unlock[] = { 0xFF, 0xAA, 0x69, 0x88, 0xB5 };
    uint8_t cmd[]    = { 0xFF, 0xAA, 0x24, 0x00 };       // 寄存器 0x24 = 0x00 → 6 轴
    for (int i = 0; i < 5; i++) DL_UART_Main_transmitDataBlocking(UART1, unlock[i]);
    system_delay_ms(50);
    for (int i = 0; i < 4; i++) DL_UART_Main_transmitDataBlocking(UART1, cmd[i]);
    system_delay_ms(50);
}

/* ================================================================
 * callback
 * ================================================================ */
static void callback(uint32_t event, void *ptr)
{
    uint8_t data;

    if (!(event & UART_INTERRUPT_STATE_RX))
        return;
    if (!uart_query_byte(UART_1, &data))
        return;

    diag_last   = data;
    diag_bytes++;

    if (data == JY901_HEADER) {
        rx_index = 0;
        diag_header++;
    }

    if (rx_index < JY901_RX_BUF_SIZE)
        rx_buf[rx_index++] = data;

    if (rx_index >= JY901_RX_BUF_SIZE) {
        if (rx_buf[0] == JY901_HEADER && rx_buf[1] == JY901_TYPE_ANGLE)
            rx_frame_ready = 1;
        rx_index = 0;
    }
}

/* ================================================================
 * parse_angle
 * ================================================================ */
static void parse_angle(void)
{
    int16_t r = (int16_t)( ((uint16_t)rx_buf[3] << 8) | rx_buf[2] );
    int16_t p = (int16_t)( ((uint16_t)rx_buf[5] << 8) | rx_buf[4] );
    int16_t y = (int16_t)( ((uint16_t)rx_buf[7] << 8) | rx_buf[6] );

    gyro_roll  = (float)r * JY901_ANGLE_SCALE;
    gyro_pitch = (float)p * JY901_ANGLE_SCALE;
    gyro_yaw   = (float)y * JY901_ANGLE_SCALE;

    diag_good++;
}

/* ================================================================
 * JY901P_Controll
 * ================================================================ */
void JY901P_Controll(void)
{
    if (rx_frame_ready) {
        rx_frame_ready = 0;
        parse_angle();
    }
}

/* ================================================================
 * JY901P_DiagDisplay
 * ================================================================ */
void JY901P_DiagDisplay(void)
{
    char str[20];

    sprintf(str, "RX:%-6u",  (unsigned int)diag_bytes);
    tft180_show_string(0, 0, str);

    sprintf(str, "H:%-4u G:%-4u", (unsigned int)diag_header, (unsigned int)diag_good);
    tft180_show_string(0, 16, str);

    tft180_show_string(0, 32, "Roll  Pitch  Yaw");
    tft180_show_float(0,  48, gyro_roll,  3, 1);
    tft180_show_float(16, 48, gyro_pitch, 3, 1);
    tft180_show_float(32, 48, gyro_yaw,   3, 1);

    sprintf(str, "last:0x%02X", diag_last);
    tft180_show_string(0, 67, str);
}

/* ================================================================
 * angle_diff
 * ================================================================ */
float angle_diff(float cur, float tgt)
{
    float d = cur - tgt;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
