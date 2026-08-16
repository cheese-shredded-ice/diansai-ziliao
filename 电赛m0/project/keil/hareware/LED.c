//#include "RTE_Components.h"             // Component selection
#include "zf_common_headfile.h"
void LED_Init(void){
	gpio_init(B2, GPO, 0, GPO_PUSH_PULL);
	gpio_init(B14, GPO, 0, GPO_PUSH_PULL);
	gpio_init(B19, GPO, 0, GPO_PUSH_PULL);
}
void LED1_On(void){
	gpio_set_level (B2,1);
}

void LED1_Off(void){
	gpio_set_level (B2,0);
}	
void LED2_On(void){
	gpio_set_level (B14,1);
}
void LED2_Off(void){
	gpio_set_level (B14,0);
}	

void LED3_On(void){
	gpio_set_level (B19,1);
}
void LED3_Off(void){
	gpio_set_level (B19,0);
}	