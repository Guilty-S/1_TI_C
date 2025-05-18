#include "PID.h"

#define LOWPASS_FILTER	0.7

//***********************************************************位置式PID************************************************************//
float Num2Abs(float x)
{
    if(x < 0)
        return -x;
    return x;
}

void Limit_Out(float *Output,float Limit_Min,float Limit_Max)
{
    if(*Output <= Limit_Min)
        *Output = Limit_Min;
    else if(*Output >= Limit_Max)
        *Output = Limit_Max;
}

PID_InitTypeDef Turn_PID;

void PID_Init(PID_InitTypeDef *PID_Struct, float Kpa,float Kpb,float Kd,float gkd,float Out_Limit)		//PID初始化
{
	PID_Struct->Kpa = Kpa;  
    PID_Struct->Kpb = Kpb;
	PID_Struct->Kd = Kd;
    PID_Struct->gkd = gkd;
	
	PID_Struct->Err = 0;
	PID_Struct->Err_last = 0;
	
	PID_Struct->PID_Out = 0;
	PID_Struct->Out_Limit = Out_Limit;
}


void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val)		//PID计算
{ 
	PID_Struct->Err = Exp_Val-Act_Val;		//err值为期望偏差与当前偏差的差值		
    //计算最终输出
//	PID_Struct->PID_Out = (PID_Struct->Err*PID_Struct->Err*PID_Struct->Err)*PID_Struct->Kpa +
//                              PID_Struct->Err *  PID_Struct->Kpb +
//									(PID_Struct->Err - PID_Struct->Err_last)*(PID_Struct->Kd);

    PID_Struct->PID_Out = PID_Struct->Err * PID_Struct->Kpa + (PID_Struct->Err - PID_Struct->Err_last) * (PID_Struct->Kd);// + gz * PID_Struct->gkd
    
    PID_Struct->Err_last = PID_Struct->Err;	//更新上一次err
    
	//输出限幅
	if(PID_Struct->PID_Out > PID_Struct->Out_Limit)
		PID_Struct->PID_Out = PID_Struct->Out_Limit; 
	else if(PID_Struct->PID_Out < -PID_Struct->Out_Limit)
		PID_Struct->PID_Out = -PID_Struct->Out_Limit;
}




//***********************************************************增量式PID**************************************************//

PID_Incremental Left_Wheel;
PID_Incremental Right_Wheel;

void PID_Incremental_Init(PID_Incremental *pid, float Kp, float Ki, float Kd,float Oout_Limit,unsigned char use_lowpass_filter)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	
    pid->error = 0;
    pid->last_error = 0;
    pid->last_last_error = 0;
    pid->last_out = 0;
    
    pid->out = 0;
    pid->outmax = Oout_Limit;
    pid->outmin = -Oout_Limit;
    pid->use_lowpass_filter = use_lowpass_filter;
    pid->lowpass_filter_factor = 0.3;
}


float PID_Incremental_Calc(PID_Incremental *pid, float setpoint, float input_value)
{
	float derivative;
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = setpoint - input_value;
    derivative = (pid->error - 2 * pid->last_error + pid->last_last_error);
            
    pid->P_Out = ((pid->error)-(pid->last_error))*(pid->Kp);		//比例控制部分	P_Out=KP*(err-err_last)

	pid->I_Out = (pid->error)*(pid->Ki);												//积分控制部分	I_Out=KI*error
	
	pid->D_Out = derivative*(pid->Kd);		//微分控制部分	D_Out=KD*(err-2*err_last+err_prev)    
    
    pid->out += (pid->P_Out)+(pid->I_Out)+(pid->D_Out);

    //低通滤波
    if(pid->use_lowpass_filter)
    {
        pid->out = pid->last_out * pid->lowpass_filter_factor + pid->out * (1 - pid->lowpass_filter_factor);
    }

     //输出限幅
   	Limit_Out(&pid->out,pid->outmin,pid->outmax);//输出限幅
    pid->last_out = pid->out;

    return pid->out;
}


