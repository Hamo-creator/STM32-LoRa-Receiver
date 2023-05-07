
#include "ServoBLDC.h"


/* ---- Motor_Num must be modified when adding more pins and timer (Motors) ---- */
const Motor_CfgType Motor_CfgParam[Motor_Num] = 
{
	{
		GPIOA,
		GPIO_PIN_8,
		TIM1,
		&TIM1->CCR1,
		TIM_CHANNEL_1,
		1.0,
		2.0
	},
	{
		GPIOA,
		GPIO_PIN_9,
		TIM1,
		&TIM1->CCR2,
		TIM_CHANNEL_2,
		1.0,
		2.0
	}
};


static Motor_Info Motor_InfoParam[Motor_Num] = {0};


/* ---- This functon maps the ADC values to specific PWM values to write to the motor ---- */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}


/* ---- Initializing the Motors ---- */

void motorInit(uint16_t Motor_Instance)
{
	uint32_t ARR_Value;
	ARR_Value = getARRvalue(Motor_Instance);
	Motor_InfoParam[Motor_Instance].Period_min = (ARR_Value * (Motor_CfgParam[Motor_Instance].Min_Pulse / 20.0));
	Motor_InfoParam[Motor_Instance].Period_max = (ARR_Value * (Motor_CfgParam[Motor_Instance].Max_Pulse / 20.0));
	
	HAL_TIM_PWM_Start(&htim1, Motor_CfgParam[Motor_Instance].TIM_PWM_CH);
	HAL_TIM_PWM_Start(&htim2, Motor_CfgParam[Motor_Instance].TIM_PWM_CH);
}


/* ---- Return minimum pulse ---- */
uint16_t getMinpulse(uint16_t Motor_Instance)
{
	return(Motor_InfoParam[Motor_Instance].Period_min);
}


/* ---- Return maximum pulse ---- */
uint16_t getMaxpule(uint16_t Motor_Instance)
{
	return(Motor_InfoParam[Motor_Instance].Period_max);
}
	

/*---- Moves a motor to specific angle ---- */
/*
	TIM_Instamce:	Timer to be used
	Channel:			Timer Channel to be ued
	angle:				Angle to which the motor will be moved
*/
void moveToangle(uint16_t Motor_Instance, long angle)
{
	*(Motor_CfgParam[Motor_Instance].CCRx) = angle;
	HAL_Delay(1);
}


/*---- Sweeps a motor from 0° to 180° and back agaim ----*/
void sweep(uint16_t Motor_Instance)
{
	long angle = 500;
	while(angle <= 1000)
	{
		*(Motor_CfgParam[Motor_Instance].CCRx) = angle;
		HAL_Delay(1);
		angle++;
	}
	HAL_Delay(250);
	while(angle >= 500)
	{
		*(Motor_CfgParam[Motor_Instance].CCRx) = angle;
		HAL_Delay(1);
		angle--;
	}
	HAL_Delay(250);
}


/* ---- Get the value of ARR ---- */
uint32_t getARRvalue(uint16_t Motor_Instance)
{
	return (Motor_CfgParam[Motor_Instance].TIM_Instance->ARR);
}
	
