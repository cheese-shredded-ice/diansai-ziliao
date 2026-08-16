//#ifndef __ENCODER_H
//#define __ENCODER_H

//#include <stdint.h>

//// ---- 编码器右 (TIMG8 硬件 QEI: B21=A相, B22=B相) ----
//void    EncoderR_Init(void);
//int32_t EncoderR_GetCount(void);
//void    EncoderR_Clear(void);

//// ---- 编码器左 (GPIO 中断软件解码: B26=A相, B27=B相) ----
//void    EncoderL_Init(void);
//int32_t EncoderL_GetCount(void);
//void    EncoderL_Clear(void);
//void    EncoderL_ISR(void);

//// ---- 通用 ----
//float   CalcRPM(int32_t pulse_diff, uint32_t time_ms);

//#endif
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void    EncoderR_Init(void);
int32_t EncoderR_GetCount(void);
void    EncoderR_Clear(void);

void    EncoderL_Init(void);
int32_t EncoderL_GetCount(void);
void    EncoderL_Clear(void);

float   CalcRPM(int32_t pulse_diff, uint32_t time_ms);

#endif
