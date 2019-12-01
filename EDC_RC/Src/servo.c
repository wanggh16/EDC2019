#include "main.h"
#include "stm32f1xx_hal.h"
#include "servo.h"

//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;


void user_Servo_Init(void)
{
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	//HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
}

void Set_Servo(uint8_t servo,uint16_t pwm)//servo 1-5  pwm 500-2500
{
		//if(pwm>2500)pwm=2500;
	  //else if(pwm<500)pwm=500;
		switch(servo)
		{
			//case 1:TIM8->CCR1=pwm;break;
			//case 2:TIM8->CCR2=pwm;break;
			//case 3:TIM8->CCR3=pwm;break;
			case 4:TIM8->CCR4=pwm;break;
			//case 5:TIM2->CCR3=pwm;break;
			default:break;
		}
}

void Servo_task(servotype* sv)
{
	if (sv->s1>2500)sv->s1=2500;
	else if(sv->s1<500)sv->s1=500;
	TIM8->CCR1=sv->s1;
	if (sv->s2>2500)sv->s2=2500;
	else if(sv->s2<500)sv->s2=500;
	TIM8->CCR2=sv->s2;
	if (sv->s3>2500)sv->s3=2500;
	else if(sv->s3<500)sv->s3=500;
	TIM8->CCR3=sv->s3;
	if (sv->s4>2500)sv->s4=2500;
	else if(sv->s4<500)sv->s4=500;
	TIM8->CCR4=sv->s4;
}
