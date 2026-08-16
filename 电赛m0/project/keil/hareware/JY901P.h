
//#ifndef __JY901P_DEBUG_H
//#define __JY901P_DEBUG_H

//#include "zf_common_headfile.h"

//void uart_gyro_init(void);
//void JY901P_Controll(void);
//void JY901P_DiagDisplay(void);
//float angle_diff(float cur, float tgt);
//extern float gyro_roll;
//extern float gyro_pitch;
//extern float gyro_yaw;

//#endif
#ifndef __JY901P_H
#define __JY901P_H

#include "zf_common_headfile.h"

/* 角度输出（主循环读取） */
extern float gyro_roll;
extern float gyro_pitch;
extern float gyro_yaw;

/* 角度差计算，归一化到 [-180, 180] */
float angle_diff(float cur, float tgt);

/* 初始化 UART1 + 注册中断，可选切换 6 轴模式 */
void uart_gyro_init(void);

/* 主循环中周期性调用，解析新帧 */
void JY901P_Controll(void);

/* TFT180 诊断显示 */
void JY901P_DiagDisplay(void);

#endif