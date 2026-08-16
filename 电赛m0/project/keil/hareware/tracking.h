#ifndef __TRACKING_H
#define __TRACKING_H

#include "zf_common_headfile.h"

/* 传感器通道数 */
#define TRACKING_CHANNELS  12

/* 12 路传感器原始数据：0=白(离线) 1=黑(压线) */
extern volatile uint8_t tracking_data[TRACKING_CHANNELS];

/* 新帧就绪标志：ISR 置 1，主循环读后清 0 */
extern volatile uint8_t tracking_ready;

/* 诊断计数器 */
extern volatile uint32_t tracking_diag_bytes;
extern volatile uint32_t tracking_diag_frames;

/* UART2 初始化（MFCLK 时钟源，PB17 TX / PB18 RX） */
void uart2_mfclk_init(uint32_t baud);

/* 初始化循迹传感器（UART2 + RX 中断） */
void tracking_init(uint32_t baud);

/* 计算黑线中心位置，返回值 -11.0 ~ +11.0（左负右正，索引 5.5 为正中） */
float tracking_center(void);

#endif
