/*
	A simple Driver for Servo motor as well as for BLDC
	must be modefied to fullfill your application
	Author: M.Khalil
	Date:		30.04.2023
*/
#ifndef _SERVOBLDC_H_
#define _SERVOBLDC_H_

#include "stm32f1xx_hal.h"
#include "tim.h"

/*--- The number of Motors will be used ----*/
#define Motor_Num		2

typedef struct
{
	GPIO_TypeDef*		Motor_GPIO;
	uint16_t				Motor_PIN;
	TIM_TypeDef*		TIM_Instance;
	uint32_t*				CCRx;
	uint32_t				TIM_PWM_CH;
	float						Min_Pulse;
	float						Max_Pulse;
}Motor_CfgType;

typedef struct
{
	uint16_t	Period_min;
	uint16_t	Period_max;
}Motor_Info;

/* ----- An Array to store motor configuratin ---- */
extern const Motor_CfgType Motor_CfgParam[Motor_Num];



long map(long x, long in_min, long in_max, long out_min, long out_max);

void motorInit(uint16_t Motor_Instance);

uint16_t getMinpulse(uint16_t Motor_Instance);

uint16_t getMaxpule(uint16_t Motor_Instance);

void moveToangle(uint16_t Motor_Instance, long angle);

void sweep(uint16_t Motor_Instance);

uint32_t getARRvalue(uint16_t Motor_Instance);

#endif
