#include "zf_common_headfile.h"
#include "PWM.h"
void Motor_Init(void){
	gpio_init(B9,GPO,0,GPO_PUSH_PULL);
	gpio_init(B8,GPO,0,GPO_PUSH_PULL);
	gpio_init(B10,GPO,0,GPO_PUSH_PULL);
	gpio_init(B11,GPO,0,GPO_PUSH_PULL);
	gpio_init(B12, GPO, 1, GPO_PUSH_PULL);
	
	PWM_Init();
}

void Motor_SetPWML(int32_t PWM){
	if(PWM>=0){
		//PWM_SetLeft(PWM);
		gpio_set_level(B10,0);
		gpio_set_level(B11,1);
	  //PWM_SetLeft(PWM);
		pwm_set_duty(PWM_TIM_G6_CH0_B6,PWM_DUTY_MAX /10000*PWM);
	}else
	{//PWM_SetLeft(-PWM);
		gpio_set_level(B10,1);
   gpio_set_level(B11,0);
   PWM_SetLeft(-PWM);
	}
}	

void Motor_SetPWMR(int32_t PWM){
	if(PWM>=0){
		//PWM_SetRight(PWM);
		gpio_set_level(B8,1);
		gpio_set_level(B9,0);
	  //PWM_SetRight(PWM);
		pwm_set_duty(PWM_TIM_G6_CH1_B7,PWM_DUTY_MAX /10000*PWM);
	}else
	{//PWM_SetRight(-PWM);
		gpio_set_level(B8,0);
   gpio_set_level(B9,1);
   PWM_SetRight(-PWM);
	}
}	