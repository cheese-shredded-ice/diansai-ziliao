#ifndef __PID_H
#define __PID_H

//typedef struct {
//	float Target;
//	float Actual;
//	float Out;
//	
//	float Kp;
//	float Ki;
//	float Kd;
//	
//	float Error0;
//	float Error1;
//	float ErrorInt;
//	
//	float OutMax;
//	float OutMin;
//} PID_t;

typedef struct {
    float Target_Valve;
    float Actual_Valve;
    float ERROR;
    float Last_ERROR;
    float Integral;
    float Kp;
    float Ki;
    float Kd;
} PID_InitTypeDef;

//void PID_Update(PID_t *p);
//void PID_Init(PID_t *P);

void PID_Init();
void PID_SET_TARGET_VALVE(PID_InitTypeDef *PIDSTRUCTURE,float Target_Temp);
float PID_SPEED_CIRCLE(PID_InitTypeDef *Speed,float Actural_Temp);
extern PID_InitTypeDef PID_InitMotor_LEFT_Structure, PID_InitMotor_RIGHT_Structure;

#endif
