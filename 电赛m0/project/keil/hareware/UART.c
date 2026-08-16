#include "zf_common_headfile.h"
void uart1_mfclk_init(uint32_t baud)
{
    uint32_t div, frac;

    DL_UART_Main_reset(UART1);
    DL_UART_Main_enablePower(UART1);

    /* 引脚用逐飞的 afio_init — 它知道 B4/B5 的正确 AF 值 */
    afio_init(B4, GPO, (gpio_af_enum)((UART1_TX_B4 >> UART_PIN_AF_OFFSET) & UART_PIN_AF_MASK), GPO_AF_PUSH_PULL);
    afio_init(B5, GPI, (gpio_af_enum)((UART1_RX_B5 >> UART_PIN_AF_OFFSET) & UART_PIN_AF_MASK), GPI_PULL_UP);

    /* 时钟源切成 MFCLK */
    DL_UART_Main_ClockConfig clk = {
        .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    DL_UART_Main_setClockConfig(UART1, &clk);

    /* 8N1 */
    DL_UART_Main_Config cfg = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
    };
    DL_UART_Main_init(UART1, &cfg);

    DL_UART_Main_setOversampling(UART1, DL_UART_OVERSAMPLING_RATE_16X);

    /* MFCLK 频率不确定，先用 32MHz 算（SYSOSC 默认值） */
    if(baud == 9600)    { div = 26; frac = 3; }
    if(baud == 115200)  { div = 2;  frac = 11; }
    if(baud == 19200)   { div = 13; frac = 1;  }
    if(baud == 38400)   { div = 6;  frac = 33; }
		
		
		DL_UART_Main_setBaudRateDivisor(UART1, div, frac);
    DL_UART_Main_enable(UART1);
}
