#include "zf_common_headfile.h"                  // Device header
#include "PID.h"
//void PID_Init(PID_t *P){
//	P->Target=0;
//	P->Actual=0;
//	P->Error0=0;
//	P->Error1=0;
//	P->ErrorInt=0;
//	P->Out=0;
//}	

PID_InitTypeDef PID_InitMotor_LEFT_Structure,PID_InitMotor_RIGHT_Structure;

void PID_Init(void)
{
	PID_InitMotor_LEFT_Structure.Target_Valve=0.0;
	PID_InitMotor_LEFT_Structure.Actual_Valve=0.0;
	PID_InitMotor_LEFT_Structure.ERROR=0.0;
	PID_InitMotor_LEFT_Structure.Last_ERROR=0.0;
	PID_InitMotor_LEFT_Structure.Integral=0.0;
	PID_InitMotor_LEFT_Structure.Kp=10,
	PID_InitMotor_LEFT_Structure.Ki=0;
	PID_InitMotor_LEFT_Structure.Kd=1;
	
	PID_InitMotor_RIGHT_Structure.Target_Valve=0.0;
	PID_InitMotor_RIGHT_Structure.Actual_Valve=0.0;
	PID_InitMotor_RIGHT_Structure.ERROR=0.0;
	PID_InitMotor_RIGHT_Structure.Last_ERROR=0.0;
	PID_InitMotor_RIGHT_Structure.Integral=0.0;
	PID_InitMotor_RIGHT_Structure.Kp=10;
	PID_InitMotor_RIGHT_Structure.Ki=0;
	PID_InitMotor_RIGHT_Structure.Kd=1;
}

void PID_SET_TARGET_VALVE(PID_InitTypeDef *PIDSTRUCTURE,float Target_Temp)
{
	PIDSTRUCTURE->Target_Valve=Target_Temp;
}

float PID_SPEED_CIRCLE(PID_InitTypeDef *Speed,float Actural_Temp)
{
	Speed->ERROR=Speed->Target_Valve-Actural_Temp;
	Speed->Integral+=Speed->ERROR;
	Speed->Actual_Valve=Speed->Kp*Speed->ERROR+Speed->Ki*Speed->Integral+Speed->Kd*(Speed->ERROR-Speed->Last_ERROR);
	Speed->Last_ERROR=Speed->ERROR;
	if(Speed->Actual_Valve >=9000){Speed->Actual_Valve = 9000;}
	if(Speed->Actual_Valve <=-9000){Speed->Actual_Valve = -9000;}
	return Speed->Actual_Valve;
}


//void PID_Update(PID_t *p)
//{
//	p->Error1 = p->Error0;
//	p->Error0 = p->Target - p->Actual;
//	
//	if (p->Ki != 0)
//	{
//		p->ErrorInt += p->Error0;
//	}
//	else
//	{
//		p->ErrorInt = 0;
//	}
//	
//	p->Out = p->Kp * p->Error0
//		   + p->Ki * p->ErrorInt
//		   + p->Kd * (p->Error0 - p->Error1);
//	
//	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
//	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
//  if (p->Out >= p->OutMax && p->Error0 > 0)  p->ErrorInt -= p->Error0;
//	if (p->Out <= p->OutMin && p->Error0 < 0)  p->ErrorInt -= p->Error0;
//}


//#include "zf_common_headfile.h"                  // Device header
//#include "PID.h"
//void PID_Init(PID_t *P){
//	P->Target=0;
//	P->Actual=0;
//	P->Error0=0;
//	P->Error1=0;
//	P->ErrorInt=0;
//	P->Out=0;
//}	
//	
//void PID_Update(PID_t *p)
//{
//	p->Error1 = p->Error0;
//	p->Error0 = p->Target - p->Actual;

//	if (p->Ki != 0)
//	{
//		p->ErrorInt += p->Error0;
//	}
//	else
//	{
//		p->ErrorInt = 0;
//	}

//	p->Out = p->Kp * p->Error0
//	       + p->Ki * p->ErrorInt
//	       + p->Kd * (p->Error0 - p->Error1);

//	if (p->Out > p->OutMax)  p->Out = p->OutMax;
//	if (p->Out < p->OutMin)  p->Out = p->OutMin;

//	// 积分限幅：输出被掐住了就停积分，防止饱和
//	if (p->Out >= p->OutMax && p->Error0 > 0)  p->ErrorInt -= p->Error0;
//	if (p->Out <= p->OutMin && p->Error0 < 0)  p->ErrorInt -= p->Error0;
//}

