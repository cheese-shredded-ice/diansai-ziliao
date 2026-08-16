#include "zf_common_headfile.h"
void PWM_Init(void){
	//pwm_init(PWM_TIM_G8_CH0_B6, 50000,PWM_DUTY_MAX /3);
	pwm_init(PWM_TIM_G6_CH1_B7 ,50000,0);
	pwm_init(PWM_TIM_G6_CH0_B6, 50000,0);
}

void PWM_SetLeft(uint32_t duty){
	pwm_set_duty(PWM_TIM_G6_CH0_B6,duty);
}	
void PWM_SetRight(uint32_t duty){
	pwm_set_duty(PWM_TIM_G6_CH1_B7,duty);
}		