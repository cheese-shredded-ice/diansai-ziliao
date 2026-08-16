#include "zf_common_headfile.h" 
#include "PID.h"
#include "Encoder.h"
#include "Motor.h"
#include "Sensor.h"



#define SPEED_TARGET        800     // 目标速度（每20ms脉冲数）
#define STEERING_TARGET     0.0f    // 目标偏移量（居中）

//static PID_t SpeedPID;         // 速度环 PID  → 控制车整体速度
//static PID_t SteeringPID;      // 转向环 PID  → 控制车转向
//PID_t SpeedPID_L={
//	.Kp=10,
//	.Ki=0.00,
//	.Kd=0,
//	.OutMax=9000,
//  .OutMin=-5000,
//	.Target =400,
//};	
//PID_t SpeedPID={
//	.Kp=10,
//	.Ki=0,
//	.Kd=0,
//	.OutMax=9000,
//  .OutMin=-5000,
//	.Target =1700,
//};	
//PID_t SpeedPID_R={
//	.Kp=10,
//	.Ki=0.00,
//	.Kd=0,
//	.OutMax=9000,
//  .OutMin=-5000,
//	.Target =400,
//};	

//PID_t SteeringPID={
//	.Kp=3,
//	.Ki=0,
//	.Kd=0.0,
//	.OutMax=200,
//  .OutMin=-200,
//	
//};	
// ========== 速度相关变量 ==========
static int32_t speedL = 0;      // 左轮编码器值（每20ms）
static int32_t speedR = 0;      // 右轮编码器值
static float actual_speed = 0;  // 实际速度（左右轮平均）

// ========== 转向相关变量 ==========
static float offset = 0;        // 灰度偏移量

// ========== PWM 输出 ==========
static int32_t pwm_base = 0;    // 速度环输出（基础油门）
static int32_t pwm_steer = 0;   // 转向环输出（差速补偿）
static int32_t pwm_left = 0;    // 左轮最终PWM
static int32_t pwm_right = 0;   // 右轮最终PWM


//void Control_Init(void)
//{
//    
//    PID_Init(&SpeedPID );
//    PID_Init(&SteeringPID);
//}


// 20ms执行一次，在主函数定时中断

//void Control_Update(void)
//{
//    // 读取传感器 
//    // 读编码器（每次读取后自动清零，所以得到的是"每20ms脉冲数"）
//    speedL = EncoderL_GetCount();
//    speedR = EncoderR_GetCount();
//    
//    // 读灰度传感器,计算偏移量
//    read_grayscale();
//    offset = grayscale_calculate();
//    
//    // 速度环 PID 
//    // 实际速度 = 左右轮平均
//    actual_speed = (float)(speedL + speedR) * 0.5f;
//    
//    SpeedPID.Target = (float)SPEED_TARGET;
//    SpeedPID.Actual = actual_speed;
//    PID_Update(&SpeedPID);
//    pwm_base = (int32_t)SpeedPID.Out;      // 基础油门
//    
//    // ========== 3. 转向环 PID ==========
//    SteeringPID.Target = STEERING_TARGET;  // 目标偏移 = 0（居中）
//    SteeringPID.Actual = offset;           // 实际偏移
//    PID_Update(&SteeringPID);
//    pwm_steer = (int32_t)SteeringPID.Out;  // 转向补偿
//    
//    // ========== 4. 合成左右轮 PWM ==========
//    // 差速转向：左加右减 → 右转；左减右加 → 左转
//    pwm_left  = pwm_base + pwm_steer;
//    pwm_right = pwm_base - pwm_steer;
//    
//    //  限幅（0~PWM_DUTY_MAX）为10000
//    if(pwm_left  > PWM_DUTY_MAX) pwm_left  = PWM_DUTY_MAX;
//    if(pwm_left  < -PWM_DUTY_MAX) pwm_left  = -PWM_DUTY_MAX;
//    if(pwm_right > PWM_DUTY_MAX) pwm_right = PWM_DUTY_MAX;
//    if(pwm_right < -PWM_DUTY_MAX) pwm_right = -PWM_DUTY_MAX;
//    
//    // ========== 6. 输出到电机 ==========
//    Motor_SetPWML(pwm_left);
//    Motor_SetPWMR(pwm_right);
//}