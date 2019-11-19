#ifndef __MOTOR_H__
#define __MOTOR_H__

#define DRVTIM TIM1
#define DUTY_MAX 1000

#define MT_P_BANK GPIOA
#define MT_N_BANK GPIOB

#define MT_1P_PIN LL_GPIO_PIN_8
#define MT_2P_PIN LL_GPIO_PIN_9
#define MT_3P_PIN LL_GPIO_PIN_10
#define MT_4P_PIN LL_GPIO_PIN_11

#define MT_1N_PIN LL_GPIO_PIN_13
#define MT_2N_PIN LL_GPIO_PIN_14
#define MT_3N_PIN LL_GPIO_PIN_15
#define MT_4N_PIN LL_GPIO_PIN_12

//#define RBTH 100

typedef struct{
	int32_t target;
	int32_t cnt;
	int32_t cnt_last;
	float err_last;
	float err_int;
	float kp;
	float ki;
	float kd;
	int32_t outlimit;
	float intlimit;
}pidtype;

typedef struct{
	uint8_t motor_en;
	uint8_t motor_update;
	uint8_t pid_en;
	uint8_t pid_update_period;
}mt_ctrltype;

typedef struct{
	uint32_t time;
	int16_t x;
	int16_t y;
	int16_t r;
	uint8_t cal_speed;

}speed3axistype;

__STATIC_INLINE void Set_speed(int16_t *speedarr,mt_ctrltype *ctrl,pidtype *mt)
{
	uint8_t i;
	for(i=0;i<4;++i)
	{
		mt[i].target = speedarr[i];
	}
	if(ctrl->pid_en == 0)
		ctrl->motor_update = 1;
}

__STATIC_INLINE void Motor_PID_Disable(mt_ctrltype *ctrl)
{
	ctrl->pid_en = 0;
}

__STATIC_INLINE void Motor_unlock(mt_ctrltype *ctrl)
{
	ctrl->motor_en = 1;
}

__STATIC_INLINE void Motor_update(mt_ctrltype *ctrl)
{
	ctrl->motor_update = 1;
}

void user_Motor_Init(void);
void Motor_task(mt_ctrltype *ctrl,pidtype *mt);
void Motor_lock(mt_ctrltype *ctrl);
void Get_cnt(pidtype *mt);
void Motor_PID_Enable(mt_ctrltype *ctrl,pidtype *mt,float kp,float ki,float kd);

#endif 
