#include "zf_common_headfile.h"
#include "LED.h"
#include "zf_device_oled.h"
#include "ti_msp_dl_config.h" 

// ===== 电赛H题：车载平衡滚球运动控制系统 =====
int8_t TrueFlag=0;           // 当前选中的任务编号 (1~6)
int8_t onFlag=0;             // 任务是否已启动
int8_t task_done[7]={0};     // 各任务完成标志 [1]~[6]，0=未完成 1=已完成
uint32_t task_start_ms=0;    // 任务启动时刻 (ms)
int8_t  t2_line_clear=0;     // 任务2：是否已驶离起跑线
uint8_t t2_mid_cnt=0;        // 任务2：中间传感器连续检测到线的次数
uint32_t task_elapsed_ms=0;  // 任务已用时间 (ms)

int8_t yaw_locked=0;
int8_t yaw_second_locked=0;
static volatile int8_t SenFlag=0;
static volatile int8_t brake_flag = 0;
static volatile uint8_t disp_data[12];   
static volatile float   disp_center=0;     
static volatile uint8_t disp_flag;   

int8_t key_event=0;
int8_t CountBlack=1;
int32_t lastL, lastR;
int32_t SpeedL=0,SpeedR=0;
float Line_er=0;
float initial_yaw=0,steer=0;
float yaw_er=0;
float yaw1_er=0;
float yaw2_er=0;
float steer_yaw;
int32_t steer_kp=9;//陀螺仪
int32_t PID_L,PID_R;
int32_t base_L=900,base_R=900;
int32_t target_L=0,target_R=0;
static float   Line_kp  = 100.0f;    // P：线偏一格，转向量多大
static float   Line_kd  = 5.0f;     // D：线在回正，提前减速
static float   last_center = 0;  
static uint32_t uart_watchdog_ms = 0;        // 距上次收到帧过了多少 ms
static uint32_t last_frames     = 0;         // 上一次的帧计数
static volatile int8_t SenFlag_prev = 0;
static volatile int8_t SenFlag_rise = 0;//白到黑
static volatile int8_t SenFlag_fall =0;//黑道白
static volatile int8_t CountSenFlag_rise=0;//计算
static volatile int8_t CountSenFlag_fall=0;//计算
static volatile int8_t SenFlag_rise_latched = 0;//锁住
static volatile int8_t SenFlag_fall_latched = 0;//锁住
float yaw1=0;
float yaw2=0;
static volatile int8_t  SenFlag_tmp   = 1;       // 中间值
static volatile uint8_t  debounce_cnt = 0;       // 连续同值计数

int8_t Flag =1;              // 选择界面光标 (1~6)

void xunxian(void);
void task2(void);

void pit_key_callback(uint32 event, void *ptr)
{
   // gpio_set_level (B2,0) ;

   
		(void)event;
		(void)ptr;
		
		key_scanner();       // ← 按键扫描放在 PIT 中断里
		JY901P_Controll();
	  if (key_event == 0) {                    // 上一事件没被主循环取走就不覆盖  
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) { key_event = 1; key_clear_state(KEY_1); }
 	      if (key_get_state(KEY_2) == KEY_SHORT_PRESS) { key_event = 2; key_clear_state(KEY_2); }
	      if (key_get_state(KEY_3) == KEY_SHORT_PRESS) { key_event = 3; key_clear_state(KEY_3); }
	      if (key_get_state(KEY_4) == KEY_SHORT_PRESS) { key_event = 4; key_clear_state(KEY_4); }
		}
		
		
		 uart_watchdog_ms += 10;
		 if (onFlag) { task_elapsed_ms += 10; }  // 任务运行计时
		 if (tracking_ready) {
        tracking_ready = 0;
			  uint8_t white_count = 0;
        for (uint8_t i = 0; i < 12; i++) {
            disp_data[i] = tracking_data[i];
					  if (disp_data[i] == 0) {white_count++;}
						
        }
//				SenFlag=0;
//				SenFlag = 0;
//        for (uint8_t i = 0; i < 12; i++) {
//           if (disp_data[i]) { SenFlag = 1; break; }
//        }
				int8_t sen_raw = (white_count >= 12) ? 0 : 1;
				if (sen_raw == SenFlag_tmp) {
        debounce_cnt++;
          if (debounce_cnt >= 4) {
              SenFlag     = sen_raw;        // 确认切换
              debounce_cnt = 0;
             }
           } else {
        debounce_cnt = 0;
        SenFlag_tmp  = sen_raw;           // 换方向，重新计数
       }	
			}
        /* 存档：当前变成下一帧的"上一帧" */
        SenFlag_prev = SenFlag;
			 
	if (SenFlag_rise) SenFlag_rise_latched = 1;    // 只置 1
      if (SenFlag_fall) SenFlag_fall_latched = 1;
			
			
			
			 if (uart_watchdog_ms >= 200) {     // 超过 500ms 没收到帧
        uart_watchdog_ms = 0;
        tracking_init(115200);         // 软复位 UART2
    }
			
    int32_t nowL = EncoderL_GetCount();
    int32_t nowR = EncoderR_GetCount();
    
    SpeedL = nowL - lastL;   // 这 10ms 内的脉冲增量
    SpeedR = nowR - lastR;
    
    lastL = nowL;
    lastR = nowR;
     //key_scanner();
      //int32_t PID_L,PID_R;
    PID_SET_TARGET_VALVE(&PID_InitMotor_LEFT_Structure,target_L);
	PID_SET_TARGET_VALVE(&PID_InitMotor_RIGHT_Structure,target_R);
    	
    PID_L = PID_SPEED_CIRCLE(&PID_InitMotor_LEFT_Structure,SpeedL);
    PID_R = PID_SPEED_CIRCLE(&PID_InitMotor_RIGHT_Structure,SpeedR);
    
    if (brake_flag==1) {
		PID_L=0;
		PID_R =0;
		PID_InitMotor_LEFT_Structure.Integral   = 0;
		PID_InitMotor_LEFT_Structure.Last_ERROR = 0;
		PID_InitMotor_RIGHT_Structure.Integral   = 0;
		PID_InitMotor_RIGHT_Structure.Last_ERROR = 0; 
	}                
    Motor_SetPWML((int32_t)PID_R);
    Motor_SetPWMR((int32_t)PID_L);
    
}

int main(void)
{
    PID_Init();
	SYSCFG_DL_init();  
	clock_init(SYSTEM_CLOCK_80M);
	uart_gyro_init();
	EncoderR_Init();	
	EncoderL_Init();
    lastL = EncoderL_GetCount();   // ← 在中断开之前存一次初值
    lastR = EncoderR_GetCount(); 
	
	pit_ms_init(PIT_TIM_G0, 10, pit_key_callback, NULL); 
	tft180_init();
	tft180_set_font(TFT180_8X16_FONT  );
	gpio_init(B2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	gpio_set_level (B2,1) ;
	Motor_Init (); 
	key_init(10);
	tracking_init(115200);               // ← ④ 加在 interrupt_global_enable 之前
   // ===== 界面初始化 =====
	tft180_show_string(0,   0,  "Task:");
	tft180_show_string(0,  17,  "Time:");
	tft180_show_string(108, 17,  "s");
	tft180_show_string(0,  34,  "Ball:");
	tft180_show_string(56, 34,  "Yaw:");
	tft180_show_string(0,  52,  "Tar:");
	tft180_show_string(0,  87,  "Spd:");
	tft180_show_string(0, 103,  "R:");
	tft180_show_string(25,103,  "F:");
	tft180_show_string(50,103,  "S:");
	interrupt_global_enable(0);

    while (1)
    {

        int8_t ev = key_event;
			  key_event = 0;//取走事件
			 if (SenFlag_rise_latched) {SenFlag_rise_latched = 0; CountSenFlag_rise++;}
			 if (SenFlag_fall_latched) {SenFlag_fall_latched = 0; CountSenFlag_fall++;} 
			
			// ===== 行0: 任务选择 + 状态 =====
			tft180_show_int(45, 0, Flag, 1);
			if (onFlag) tft180_show_string(60, 0, "OK ");
			else        tft180_show_string(60, 0, "   ");

			// ===== 行1: 计时 =====
			tft180_show_float(45, 17, (double)task_elapsed_ms / 1000.0, 2, 2);  // 秒.10ms

			// ===== 行2: 球位 + 偏航 =====
			tft180_show_float(30, 34, disp_center, 3, 1);
			tft180_show_float(70, 34, gyro_yaw, 3, 1);

			// ===== 行3: 目标值 =====
			tft180_show_int(30, 52, target_L, 5);
			tft180_show_int(75, 52, target_R, 5);

			// ===== 行4: 12路传感器 =====
			for (uint8_t i = 0; i < 12; i++) {
				tft180_show_int(i * 8, 70, disp_data[i], 1);
			}

			// ===== 行5: 速度 =====
			tft180_show_int(30, 87, SpeedL, 4);
			tft180_show_int(75, 87, SpeedR, 4);


			
				 if (ev) {
			    if (onFlag == 0) { 
                    if      (ev == 1) { Flag++; if (Flag > 6) Flag = 6; } 
			        else if (ev == 2) { Flag--; if (Flag < 1) Flag = 1; }
			        else if (ev == 3) { 
                        TrueFlag = Flag; onFlag = 1; yaw_locked = 0; 
                        task_elapsed_ms = 0;
                        brake_flag = 0;
                        t2_line_clear = 0; t2_mid_cnt = 0;
                        gpio_init(B12, GPO, 1, GPO_PUSH_PULL);
                        CountSenFlag_rise = 0; CountSenFlag_fall = 0; SenFlag_prev = SenFlag;
                    }
			    }
                // KEY_4 任何时候都能复位
                if (ev == 4) { 
                    brake_flag = 1; onFlag = 0; target_L = 0; target_R = 0; 
                    Flag = 1; SenFlag = 0;
                    task_elapsed_ms = 0;
                }
			}

			if (onFlag == 1 && TrueFlag >= 1 && TrueFlag <= 6) {
				gpio_set_level(B2, 0);  // 任务运行指示灯

				// ========== 任务1：图传装置验证 ==========
				if (TrueFlag == 1 && !task_done[1]) {
					// TODO: 图传画面显示 + 录像功能确认
					// 完成条件：图传画面正常，手动按键标记完成
					// task_done[1] = 1; brake_flag = 1;
				}

				// ========== 任务2：循线一圈（A→A），≤20s，停车≤2cm ==========
				else if (TrueFlag == 2 && !task_done[2]) {
					task2();
				}

				// ========== 任务3：静止摆球控制 O→+5cm→-5cm，≤5s，误差≤1cm ==========
				else if (TrueFlag == 3 && !task_done[3]) {
					// TODO: 摆杆控制小球 O→+5cm→折返→-5cm→稳定
					// 完成条件：球停在-5cm附近稳定
					// task_done[3] = 1; brake_flag = 1;
				}

				// ========== 任务4：A→B，球稳定在中心，AB≤8s，误差≤1cm ==========
				else if (TrueFlag == 4 && !task_done[4]) {
					// TODO: 循线从A到B，同时摆杆控制球在O附近
					// 完成条件：到达B点
					// task_done[4] = 1; brake_flag = 1;
				}

				// ========== 任务5：A→A一圈，球稳定在中心，≤30s，误差≤1cm ==========
				else if (TrueFlag == 5 && !task_done[5]) {
					// TODO: 循线一圈，同时摆杆控制球在O附近
					// 完成条件：到达A点停车线，停车
					// task_done[5] = 1; brake_flag = 1;
				}

				// ========== 任务6：A→A一圈，球稳定在任意指定位置，≤30s，误差≤1cm ==========
				else if (TrueFlag == 6 && !task_done[6]) {
					// TODO: 循线一圈，同时摆杆控制球在指定位置
					// 完成条件：到达A点停车线，停车
					// task_done[6] = 1; brake_flag = 1;
				}
			}

       
    }

       
}       

void xunxian(void)
{
	disp_center = tracking_center();
	float d_center = disp_center - last_center;
    last_center    = disp_center;
	Line_er=Line_kp*disp_center+Line_kd * d_center;
	target_L=base_L-Line_er; 
	target_R=base_R+Line_er;
}

void task2(void)
{
	xunxian();  // 循线PID

	// 统计12路传感器检测到黑线的数量
	uint8_t black_cnt = 0;
	for (uint8_t i = 0; i < 12; i++) {
		if (disp_data[i] == 1) black_cnt++;
	}

	// 10秒内不触发停车（避让起跑线等）
	if (task_elapsed_ms < 15000) return;

	// 任意3路黑线 → 刹车
	if (black_cnt >= 3) {
		task_done[2] = 1;
		brake_flag = 1;
		onFlag = 0;          // 暂停计时
	}
}