#include "my_main.h"
#include "main.h"
#include "stdlib.h"
#include "pid.h"
PID_InitTypeDef Turn_PID_1;
PID_InitTypeDef Turn_PID_2;
#define PWM_MAX 7200
#define PWM_MIN -7200
int dead_area =1000;
int mode_time=0;
int last_time=0;
int flag_stop=1;
int flag_y=1;
int count=0;
int k=0;
int t_1=0;
int t_2=0;
int t_3=0;
int t_5s=0;
int t_360=0;
int Encoder_Integral_x=-300;
extern int key_real;
extern int angle;
int flag_5s=0;
int flag_360=0;
int flag=1;
int flag_pid=0;
int flag_x=1;
int Encoder_Cnt;
int Encoder_Aim=0;
float Encoder_Integral;
int cnt=0;
int flag_2=0;
extern int angle1;
extern int angle2;
extern int key;
//----ADC-----
uint16_t adc_value=0;
uint16_t adc[5] = {0};

/**************************************************************************
函数功能：单位时间读取ADC
入口参数：TIM
返回  值：ADC
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        adc_value = ADC_Read();
        Encoder_Cnt = encoder_speed();
        Encoder_Integral += Encoder_Cnt;
        if(key_real==4||key_real==8)
        {
            PID_Calculate(&Turn_PID_1,2350-angle1,adc_value); //0~4000
            PID_Calculate(&Turn_PID_2,Turn_PID_1.PID_Out,-Encoder_Cnt); //-15~15
            Load(Turn_PID_2.PID_Out);
        }
        if(key_real==5)
        {
            if (angle2 < -151 || angle2 > 151)
            {
                flag_pid = 1;
            } else {
                flag_pid = 0;
            }
            t_1++;
            if (t_1 < 38 && !flag_pid)
            {
                Load(3375);
            } else if (t_1 >= 35 && !flag_pid)
            {
                if (t_1 > 70) {
                    t_1 = 0;
                }
                Load(-3500);
            }
            if (flag_pid)
            {
                Turn_PID_2.Kd = -100;
                PID_Calculate(&Turn_PID_1, 2350 - angle1, adc_value); //0~4000
                PID_Calculate(&Turn_PID_2, Turn_PID_1.PID_Out, -Encoder_Cnt); //-15~15
                Load(Turn_PID_2.PID_Out);
            }
        }
        if(key_real==7)
        {
            t_5s++;
            if(t_5s>=500)
            {
                flag_5s=1;
            }
            if(!flag_5s)
            {
                PID_Calculate(&Turn_PID_1, 2350 - angle1, adc_value); //0~4000
                PID_Calculate(&Turn_PID_2, Turn_PID_1.PID_Out, -Encoder_Cnt); //-15~15
                Load(Turn_PID_2.PID_Out);
            }
            else
            {
                count++;
                if(count>20)
                {
                    Encoder_Aim++;
                }
                PID_Calculate(&Turn_PID_1, 2350 - angle1, adc_value); //0~4000
                PID_Calculate(&Turn_PID_2, Encoder_Aim+Turn_PID_1.PID_Out, -Encoder_Cnt); //-15~15
                Load(Turn_PID_2.PID_Out);
            }
        }
    }
}
uint16_t ADC_Read(void)
{
    int sum = 0, max = 0, min = 4095,adc_result = 0;
    uint8_t i=0;
    HAL_ADC_Start(&hadc1);     //启动ADC转换
    HAL_ADC_PollForConversion(&hadc1, 12);   //等待转换完成，50为最大等待时间，单位为ms
    //平滑均值滤波
    for(i=1;i<=5;i++)
    {
        adc[i] = HAL_ADC_GetValue(&hadc1);
        sum += adc[i];
        if(adc[i] > max) max = adc[i];
        if(adc[i] < min) min = adc[i];
    }
    adc_result = (sum - max - min) / 3.0;
//	sum = KalmanFilter(adc);
    return adc_result;
}
/**************************************************************************
函数功能：卡尔曼滤波
入口参数：inData
返回  值：inData
**************************************************************************/
//卡尔曼滤波

/**************************************************************************
函数功能：电机转动控制函数
入口参数：闭环控制最终输出值
**************************************************************************/


/**************************************************************************
函数功能：限制电机速度
入口参数：闭环控制最终输出值
**************************************************************************/

uint32_t encoder_speed(void)
{
    uint32_t speed;

    speed=(short)__HAL_TIM_GET_COUNTER(&htim3);//获得编码器脉冲
    __HAL_TIM_SET_COUNTER(&htim3,0);	//编码器脉冲清0
    return speed;
}
int abs(int p)
{
    int q;
    q=p>0?p:(-p);
    return q;
}
void Load(int motorA)
{
    if (motorA < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, abs(motorA)+dead_area);
}
void Limit(int *motoA)
{
    if (*motoA > PWM_MAX)*motoA = PWM_MAX;
    if (*motoA < PWM_MIN)*motoA = PWM_MIN;
}