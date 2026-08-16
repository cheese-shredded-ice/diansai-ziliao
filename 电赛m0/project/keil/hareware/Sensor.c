#include "zf_common_headfile.h"
//#define GRAY_SENSOR_COUNT   8  //传感器数量

//uint8_t gray_value[GRAY_SENSOR_COUNT] = {0};

//static gpio_pin_enum gray_pins[GRAY_SENSOR_COUNT] = {
//    A27,A26,A25,A24,A22,B25,B24,B20
//};//从左到右

//static const float gray_weights[GRAY_SENSOR_COUNT] = {
//    -3.5f, -2.5f, -1.5f, -0.5f,
//     0.5f,  1.5f,  2.5f,  3.5f
//};//传感器权重

//void gpio_grayscale_init(void)
//{
//    // A27在最左边
//    gpio_init(A27, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(A26, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(A25, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(A24, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(A22, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(B25, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(B24, GPI, 0,GPI_PULL_DOWN);
//    gpio_init(B20, GPI, 0,GPI_PULL_DOWN);
//}

///**
// * @brief 读取8路灰度传感器的电平状态
// * @note  gpio_read(port, pin) 返回 0=低电平(白地) 或 1=高电平(黑线)
// */
//void read_grayscale(void)
//{
//    gray_value[0] = gpio_get_level(A27);
//    gray_value[1] = gpio_get_level(A26);
//    gray_value[2] = gpio_get_level(A25);
//    gray_value[3] = gpio_get_level(A24);
//    gray_value[4] = gpio_get_level(A22);
//    gray_value[5] = gpio_get_level(B25);
//    gray_value[6] = gpio_get_level(B24);
//    gray_value[7] = gpio_get_level(B20);
//}
//float grayscale_calculate(void){
//	float sum_weighted=0.0f;
//	float sum_active=0.0f;
//	int i=0;
//	for(i=0;i<GRAY_SENSOR_COUNT;i++){
//		float line_detected=1.0f-(float)gray_value[i];
//		sum_weighted +=line_detected*gray_weights[i];
//		sum_active+=line_detected;
//	}
//	if(sum_active <0.5f){
//		return 0.0f;
//	}
//	return sum_weighted/sum_active ;
//}	

	
