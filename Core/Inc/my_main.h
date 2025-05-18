#ifndef MY_MAIN_H
#define MY_MAIN_H


#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "OLED.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

int Gray_PID(int adc_value,int aim_value);
int GetVelocity(int Encoder_left,int AimVelocity);
void Control(void);
void Load(int motorA);
void Limit(int *motorA);
uint32_t encoder_speed(void);
uint16_t ADC_Read(void);

#endif
