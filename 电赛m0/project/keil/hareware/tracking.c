
#include "zf_common_headfile.h"
#include "tracking.h"

/* ================================================================
 * 协议帧定义
 *
 *   帧格式:  #  D0  D1  ...  D11  !
 *   共 14 字节，数据域 12 个 ASCII '0'/'1'
 * ================================================================ */
#define START_BYTE   '#'
#define END_BYTE     '!'

/* ================================================================
 * 全局输出
 * ================================================================ */
volatile uint8_t tracking_data[TRACKING_CHANNELS] = {0};
volatile uint8_t tracking_ready = 0;

/* 诊断计数器 */
volatile uint32_t tracking_diag_bytes  = 0;
volatile uint32_t tracking_diag_frames = 0;

/* ================================================================
 * 接收状态机
 * ================================================================ */
static uint8_t ascii_buf[TRACKING_CHANNELS];
static volatile uint8_t buf_index = 0;
static volatile uint8_t receiving  = 0;

/* ================================================================
 * tracking_parse_byte  —— 协议状态机
 *
 *  状态机: IDLE → (收到 #) → RECEIVING → (收到 !) → 校验并输出
 * ================================================================ */
static void tracking_parse_byte(uint8_t rx)
{
    if (!receiving) {
        if (rx == START_BYTE) {
            receiving  = 1;
            buf_index  = 0;
        }
        return;
    }

    if (rx == END_BYTE) {
        for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
            if (ascii_buf[i] == '0')
                tracking_data[i] = 0;
            else if (ascii_buf[i] == '1')
                tracking_data[i] = 1;
            else {
                receiving = 0;
                return;
            }
        }
        tracking_ready  = 1;
        tracking_diag_frames++;
        receiving = 0;
        return;
    }

    if (rx == '0' || rx == '1') {
        if (buf_index < TRACKING_CHANNELS) {
            ascii_buf[buf_index++] = rx;
        } else {
            receiving = 0;
        }
    } else {
        receiving = 0;
    }
}

/* ================================================================
 * tracking_callback  —— 逐飞库 UART2 RX 回调
 *
 *  由 isr.c 的 UART2_IRQHandler 通过 uart_callback_list 分发过来。
 *  每收到一个字节触发一次。
 * ================================================================ */
static void tracking_callback(uint32 event, void *ptr)
{
    (void)ptr;

    if (!(event & UART_INTERRUPT_STATE_RX))
        return;

    uint8_t rx = DL_UART_Main_receiveData(UART2);
    tracking_diag_bytes++;
    tracking_parse_byte(rx);
}

/* ================================================================
 * uart2_mfclk_init  —— UART2 硬件初始化（MFCLK 时钟源）
 *
 *  管脚: PB17(TX) / PB18(RX)，MFCLK，16X 过采样
 * ================================================================ */
void uart2_mfclk_init(uint32_t baud)
{
    uint32_t div, frac;

    DL_UART_Main_reset(UART2);
    DL_UART_Main_enablePower(UART2);

    afio_init(B17, GPO, (gpio_af_enum)((UART2_TX_B17 >> UART_PIN_AF_OFFSET) & UART_PIN_AF_MASK), GPO_AF_PUSH_PULL);
    afio_init(B18, GPI, (gpio_af_enum)((UART2_RX_B18 >> UART_PIN_AF_OFFSET) & UART_PIN_AF_MASK), GPI_PULL_UP);

    DL_UART_Main_ClockConfig clk = {
        .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    DL_UART_Main_setClockConfig(UART2, &clk);

    DL_UART_Main_Config cfg = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
    };
    DL_UART_Main_init(UART2, &cfg);

    DL_UART_Main_setOversampling(UART2, DL_UART_OVERSAMPLING_RATE_16X);

    if (baud == 9600)   { div = 26; frac = 3;  }
    if (baud == 115200) { div = 2;  frac = 11; }
    if (baud == 19200)  { div = 13; frac = 1;  }
    if (baud == 38400)  { div = 6;  frac = 33; }

    DL_UART_Main_setBaudRateDivisor(UART2, div, frac);
    DL_UART_Main_enable(UART2);
}

/* ================================================================
 * tracking_init  —— 循迹传感器初始化
 *
 *  硬件走 MFCLK，中断走 isr.c 已有的 UART2_IRQHandler 回调链。
 * ================================================================ */
void tracking_init(uint32_t baud)
{
    uart2_mfclk_init(baud);

    /* 注册到逐飞库回调链（isr.c 的 UART2_IRQHandler 会分发） */
    uart_set_callback(UART_2, tracking_callback, NULL);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

/* ================================================================
 * tracking_center  —— 计算黑线中心位置（加权质心法）
 *
 *  返回值 -11.0 ~ +11.0，左负右正，5.5 为正中
 * ================================================================ */
float tracking_center(void)
{
    float sum_weight = 0.0f;
    float sum_val    = 0.0f;

    for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
        if (tracking_data[i]) {
            float pos = (float)i * 2.0f - 11.0f;
            sum_weight += pos;
            sum_val    += 1.0f;
        }
    }

    if (sum_val == 0.0f)
        return 0.0f;

    return sum_weight / sum_val;
}
