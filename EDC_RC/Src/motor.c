#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "config.h"

//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/*Encoder for motor 1*/
int16_t cnt1 = 0;
/*end Encoder for motor 1*/
float currerr[4];
//int32_t speedwatch[4];
//int32_t currspeedwatch[4];

void Motor_task(mt_ctrltype *ctrl,pidtype *mt)
{
	static uint32_t time_next = 0;
	uint32_t time_curr;
	int32_t  speed[4];
	uint8_t i;
	
	if(ctrl->motor_en == 0)//motors disable
		return;
	if(ctrl->pid_en == 0 )//no pid
	{
		if(ctrl->motor_update == 0)
			return;
		for(i=0;i<4;++i)//open loop
			#if MOTOR_REVERSE == 0
			speed[i] = mt[i].target;
			#else
			speed[i] = -mt[i].target;
			#endif
		
		//set pwm
		if(speed[3] >= 0)//keep decay mode the same
		{
			if(speed[0] >= 0)//0 ch1
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_ResetOutputPin(MT_P_BANK, MT_1P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_ResetOutputPin(MT_N_BANK, MT_1N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
				speed[0] = -speed[0];
			}
			speed[0] = (speed[0] > DUTY_MAX ? DUTY_MAX:speed[0]);
			LL_TIM_OC_SetCompareCH1(DRVTIM,(uint32_t)speed[0]);
				
			if(speed[1] >= 0)//1 ch2
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_ResetOutputPin(MT_N_BANK, MT_2N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_ResetOutputPin(MT_P_BANK, MT_2P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
				speed[1] = -speed[1];
			}
			speed[1] = (speed[1] > DUTY_MAX ? DUTY_MAX:speed[1]);
			LL_TIM_OC_SetCompareCH2(DRVTIM,(uint32_t)speed[1]);
				
			if(speed[2] >= 0)//2 ch3
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3N,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_ResetOutputPin(MT_P_BANK, MT_3P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_ResetOutputPin(MT_N_BANK, MT_3N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
				speed[2] = -speed[2];
			}
			speed[2] = (speed[2] > DUTY_MAX ? DUTY_MAX:speed[2]);
			LL_TIM_OC_SetCompareCH3(DRVTIM,(uint32_t)speed[2]);
				
			//3 ch4
			LL_GPIO_ResetOutputPin(MT_N_BANK, MT_4N_PIN);
			LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
			speed[3] = (speed[3] > DUTY_MAX ? DUTY_MAX:speed[3]);
			LL_TIM_OC_SetCompareCH4(DRVTIM,(uint32_t)speed[3]);
		
		}
		
		else
		{
			if(speed[0] >= 0)//0 ch1
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_SetOutputPin(MT_N_BANK, MT_1N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_SetOutputPin(MT_P_BANK, MT_1P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
				speed[0] = -speed[0];
			}
			speed[0] = (speed[0] > DUTY_MAX ? DUTY_MAX:speed[0]);
			LL_TIM_OC_SetCompareCH1(DRVTIM,(uint32_t)speed[0]);
				
			if(speed[1] >= 0)//1 ch2
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_SetOutputPin(MT_P_BANK, MT_2P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_SetOutputPin(MT_N_BANK, MT_2N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
				speed[1] = -speed[1];
			}
			speed[1] = (speed[1] > DUTY_MAX ? DUTY_MAX:speed[1]);
			LL_TIM_OC_SetCompareCH2(DRVTIM,(uint32_t)speed[1]);
				
			if(speed[2] >= 0)//2 ch3
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_SetOutputPin(MT_N_BANK, MT_3N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3N,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_SetOutputPin(MT_P_BANK, MT_3P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
				speed[2] = -speed[2];
			}
			speed[2] = (speed[2] > DUTY_MAX ? DUTY_MAX:speed[2]);
			LL_TIM_OC_SetCompareCH3(DRVTIM,(uint32_t)speed[2]);
				
			//3 ch4
			LL_GPIO_SetOutputPin(MT_N_BANK, MT_4N_PIN);
			LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);
			speed[3] = (speed[3] > DUTY_MAX ? DUTY_MAX:speed[3]);
			LL_TIM_OC_SetCompareCH4(DRVTIM,(uint32_t)-speed[3]);
		}
		ctrl->motor_update = 0;
		
		
	}
	else//pid mode
	{
		time_curr = HAL_GetTick();
		if(time_curr >= time_next)
		{
			for(i=0;i<4;++i)
			{
				//currspeedwatch[i]=(float)(mt[i].cnt-mt[i].cnt_last)/(time_curr-time_next+ctrl->pid_update_period)*1000.0/3.08;
				currerr[i]=(float)mt[i].target*3.08-(float)(mt[i].cnt-mt[i].cnt_last)/(time_curr-time_next+ctrl->pid_update_period)*1000.0; //3080pps
				mt[i].err_int += currerr[i];
				mt[i].err_int = (mt[i].err_int > mt[i].intlimit)?mt[i].intlimit:mt[i].err_int;
				mt[i].err_int = (mt[i].err_int < -mt[i].intlimit)?(-mt[i].intlimit):mt[i].err_int;
				mt[i].cnt_last = mt[i].cnt;
				
				#if MOTOR_REVERSE == 0
				//speedwatch[i]=speed[i]=currerr[i]*(mt[i].kp)+mt[i].err_int*(mt[i].ki)+(currerr[i]-mt[i].err_last)*(mt[i].kd);
				speed[i]=currerr[i]*(mt[i].kp)+mt[i].err_int*(mt[i].ki)+(currerr[i]-mt[i].err_last)*(mt[i].kd);
				
				#else
				//speedwatch[i]=speed[i]=-(currerr[i]*(mt[i].kp)+mt[i].err_int*(mt[i].ki)+(currerr[i]-mt[i].err_last)*(mt[i].kd));
				speed[i]=-(currerr[i]*(mt[i].kp)+mt[i].err_int*(mt[i].ki)+(currerr[i]-mt[i].err_last)*(mt[i].kd));
				#endif
				
				mt[i].err_last = currerr[i];
				speed[i] = (speed[i] > mt[i].outlimit)?mt[i].outlimit:speed[i];
				speed[i] = (speed[i] < -mt[i].outlimit)?(-mt[i].outlimit):speed[i];
			}
			time_next = time_curr + ctrl->pid_update_period;
			
			//set pwm
			{
				if(speed[0] >= 0)//0 ch1
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
				}
				else
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
					speed[0] = -speed[0];
				}
				speed[0] = (speed[0] > DUTY_MAX ? DUTY_MAX:speed[0]);
				LL_TIM_OC_SetCompareCH1(DRVTIM,(uint32_t)speed[0]);
				
				if(speed[1] >= 0)//1 ch2
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
				}
				else
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
					speed[1] = -speed[1];
				}
				speed[1] = (speed[1] > DUTY_MAX ? DUTY_MAX:speed[1]);
				LL_TIM_OC_SetCompareCH2(DRVTIM,(uint32_t)speed[1]);
				
				if(speed[2] >= 0)//2 ch3
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
				}
				else
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
					speed[2] = -speed[2];
				}
				speed[2] = (speed[2] > DUTY_MAX ? DUTY_MAX:speed[2]);
				LL_TIM_OC_SetCompareCH3(DRVTIM,(uint32_t)speed[2]);
				
				if(speed[3] >= 0)//3 ch4
				{
					LL_GPIO_ResetOutputPin(MT_N_BANK, MT_4N_PIN);
					LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
				}
				else
				{
					LL_GPIO_SetOutputPin(MT_N_BANK, MT_4N_PIN);
					LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);
					speed[3] = -speed[3];
				}
				speed[3] = (speed[3] > DUTY_MAX ? DUTY_MAX:speed[3]);
				LL_TIM_OC_SetCompareCH4(DRVTIM,(uint32_t)speed[3]);
			}
		}
	}
}

/*Encoder for motor 1*/
void EXTI2_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) ==
				HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
			++cnt1;
		else
			--cnt1;
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == 
				HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
			--cnt1;
		else
			++cnt1;
	}
}
/*end Encoder for motor 1*/

void Get_cnt(pidtype *mt)
{

	#if ENCODER_REVERSE == 0
	mt[0].cnt += (int32_t)cnt1;
	mt[2].cnt += (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim3)));
	mt[1].cnt += (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim4)));
	mt[3].cnt += (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim5)));
	#else
	mt[0].cnt -= (int32_t)cnt1;
	mt[2].cnt -= (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim3)));
	mt[1].cnt -= (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim4)));
	mt[3].cnt -= (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim5)));
	#endif

	cnt1 = 0;
	LL_TIM_SetCounter(htim3.Instance, 0);
	LL_TIM_SetCounter(htim4.Instance, 0);
	LL_TIM_SetCounter(htim5.Instance, 0);

}

static void pid_Motor_Init(void)
{
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH3N);
	
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3N,LL_TIM_OCPOLARITY_LOW);
	
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_1P_PIN);
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_2P_PIN);
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_3P_PIN);
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_4P_PIN);
	
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_1N_PIN);
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_2N_PIN);
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_3N_PIN);
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_4N_PIN);

	LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_4P_PIN, LL_GPIO_MODE_ALTERNATE);
	
	LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_4N_PIN, LL_GPIO_MODE_OUTPUT);	
}

void Motor_PID_Enable(mt_ctrltype *ctrl,pidtype *mt,float kp,float ki,float kd)
{
	uint8_t i;
	for(i=0;i<4;++i)
	{
		mt[i].cnt_last = mt[i].cnt;
		mt[i].err_int = 0;
		mt[i].err_last = 0;
		mt[i].kp = kp;
		mt[i].ki = ki;
		mt[i].kd = kd;
		mt[i].outlimit = 1000;
		mt[i].intlimit = 20000;
	}
	ctrl->pid_update_period = 2;
	ctrl->pid_en = 1;
	pid_Motor_Init();
}

void Motor_lock(mt_ctrltype *ctrl)
{
	LL_GPIO_SetOutputPin(MT_N_BANK, MT_4N_PIN);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_4P_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
	
	LL_TIM_DisableAllOutputs(DRVTIM);
	ctrl->motor_en = 0;
	ctrl->motor_update = 0;
}

void user_Motor_Init(void)
{
	LL_TIM_EnableAllOutputs(DRVTIM);
	LL_TIM_EnableCounter(DRVTIM);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH3N);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}
