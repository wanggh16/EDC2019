#include "main.h"
#include "stm32f1xx_hal.h"
#include "config.h"
#include "motor.h"
#include "servo.h"
#include "bluetooth.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
uint8_t bt_cnt = 0;
uint8_t btRxbuf[10]={0};

extern UART_HandleTypeDef huart3;
uint8_t data_cnt = 0;
uint8_t dataRxbuf[50]={0};

extern int8_t move_en;
extern uint32_t newtime;
extern char state;

extern int8_t big_x;
extern int8_t big_y;

uint8_t wrong = 0;
extern int16_t inityaw;

char smalltobig(int16_t x, int16_t lower, int16_t upper)
{
	int8_t xinblock = ((float)(x-lower)*5)/((float)(upper-lower)) + 0.5;
	if (xinblock < 0) xinblock = 0;
	if (xinblock > 5) xinblock = 5;
	return xinblock;
}

void BT_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed)
{
	/*state mechime : 
	0:free ,set dma
	1:wait to recieved start frame 
	2:start frame recieved,set dma for body
	3:waiting for body(dma)
	4:data recieved complete*/		
	switch(bt_cnt)
	{
		case 0:
			btRxbuf[0] = 0;
			HAL_UART_Receive_DMA(&huart2,btRxbuf,1);
			bt_cnt = 1;
		break;
		case 1:
			break;
		case 2:
			HAL_UART_Receive_DMA(&huart2,&btRxbuf[1],2);//9);
			bt_cnt = 3;
		break;
		case 3:
		break;
		case 4:
			//if(0xfe == btRxbuf[1] && 0x01 == btRxbuf[2] && 0x00 == btRxbuf[9])
			if(0x30 == btRxbuf[1])
			{ 
				//speed->y = *(int16_t *)&btRxbuf[3];
				//speed->x = *(int16_t *)&btRxbuf[5];
				//speed->r = -*(int16_t *)&btRxbuf[7];
				if (btRxbuf[2]=='m') {
					#if CVDEBUG==1
					move_en = 1;
					#else
					move_en = -1;
					#endif
				}
				else if (btRxbuf[2]=='x') move_en = 0;
				else if (btRxbuf[2]=='w') big_y++;//{speed->y = 500;speed->x = 0;speed->r = 0;}
				else if (btRxbuf[2]=='s') big_y--;//{speed->y = -500;speed->x = 0;speed->r = 0;}
				else if (btRxbuf[2]=='a') big_x--;//{speed->y = 0;speed->x = -500;speed->r = 0;}
				else if (btRxbuf[2]=='d') big_x++;//{speed->y = 0;speed->x = 500;speed->r = 0;}
				else if (btRxbuf[2]=='q') {speed->y = 0;speed->x = 0;speed->r = -500;}
				else if (btRxbuf[2]=='e') {speed->y = 0;speed->x = 0;speed->r = 500;}
				/*
				else	//servo command
				{
					speed->y = 0;speed->x = 0;speed->r = 0;
					if (btRxbuf[2]=='1') servo->s1 += 50;
					else if (btRxbuf[2]=='2') servo->s1 -= 50;
					else if (btRxbuf[2]=='3') servo->s2 += 50;
					else if (btRxbuf[2]=='4') servo->s2 -= 50;
					else if (btRxbuf[2]=='5') servo->s3 += 50;
					else if (btRxbuf[2]=='6') servo->s3 -= 50;
					else if (btRxbuf[2]=='7') servo->s4 += 50;
					else if (btRxbuf[2]=='8') servo->s4 -= 50;
				}
				*/
				speed->cal_speed = 1;
				speed->time = HAL_GetTick();
			}
			bt_cnt = 0;
			break;
		default:
			bt_cnt = 0;		
	}
	
	#if OFFLINE_DECT == 1
	if(speed->time+TIMEOUT_TH <=  HAL_GetTick())
	{
		speed->y = 0;
		speed->x = 0;
		speed->r = 0;
		speed->cal_speed = 1;
		speed->time = HAL_GetTick();
	}
	#endif
}

void data_task(gameinfo *info)
{
	/*state mechime : 
	0:free ,set dma
	1:wait to recieved start frame 
	2:start frame recieved,set dma for body
	3:waiting for body(dma)
	4:data recieved complete*/		
	switch(data_cnt)
	{
		case 0:
			dataRxbuf[0] = 0;
			HAL_UART_Receive_DMA(&huart3,dataRxbuf,1);
			data_cnt = 1;
		break;
		case 1:
			break;
		case 2:
			HAL_UART_Receive_DMA(&huart3,&dataRxbuf[1],17);
			data_cnt = 3;
		break;
		case 3:
		break;
		case 4:
			if (0xfe == dataRxbuf[1])
			{
				info->matchstate = (dataRxbuf[8]&0xC0)==0x40?1:0;
				info->yaw = (int16_t)(dataRxbuf[2] + ((uint16_t)dataRxbuf[3]<<7)) - 2000;
				info->yaw = info->yaw - inityaw;
				if (info->yaw >= 1800) info->yaw -= 3600;
				else if (info->yaw < -1800) info->yaw += 3600;
				info->cvxf = dataRxbuf[4];
				info->cvxb = dataRxbuf[5];
				info->cvxl = dataRxbuf[6];
				info->cvxr = dataRxbuf[7];
				info->AX = dataRxbuf[10] + (((uint16_t)dataRxbuf[8]&0x08)?0x100:0);
				info->AY = dataRxbuf[11] + (((uint16_t)dataRxbuf[8]&0x04)?0x100:0);
				info->BX = dataRxbuf[12] + (((uint16_t)dataRxbuf[8]&0x02)?0x100:0);
				info->BY = dataRxbuf[13] + (((uint16_t)dataRxbuf[8]&0x01)?0x100:0);
				info->P1X = dataRxbuf[14] + (((uint16_t)dataRxbuf[9]&0x80)?0x100:0);
				info->P1Y = dataRxbuf[15] + (((uint16_t)dataRxbuf[9]&0x40)?0x100:0);
				info->P2X = dataRxbuf[16] + (((uint16_t)dataRxbuf[9]&0x20)?0x100:0);
				info->P2Y = dataRxbuf[17] + (((uint16_t)dataRxbuf[9]&0x10)?0x100:0);
				info->renewed = 1;
				
				//kpx,kpr>0
				if (info->cvxf > 200 && info->cvxb > 200) info->cvxpos = 0;
				else if (info->cvxf > 200 && info->cvxb < 200) info->cvxpos = 79 - (int16_t)info->cvxb;
				else if (info->cvxf < 200 && info->cvxb > 200) info->cvxpos = (int16_t)info->cvxf - 81;
				else if (info->cvxf < 200 && info->cvxb < 200) info->cvxpos = (int16_t)info->cvxf - (int16_t)info->cvxb - 2;
				if (info->cvxl > 200 && info->cvxr > 200) info->cvxpos1 = 0;
				else if (info->cvxl > 200 && info->cvxr < 200) info->cvxpos1 = 77 - (int16_t)info->cvxr;
				else if (info->cvxl < 200 && info->cvxr > 200) info->cvxpos1 = (int16_t)info->cvxl - 83;
				else if (info->cvxl < 200 && info->cvxr < 200) info->cvxpos1 = (int16_t)info->cvxl - (int16_t)info->cvxr - 6;
				if (info->cvxf < 200 && info->cvxb < 200) info->cvangle = (int16_t)info->cvxf + (int16_t)info->cvxb - 160;
				else info->cvangle = 0;
				if (info->cvxl < 200 && info->cvxr < 200) info->cvangle1 = (int16_t)info->cvxl + (int16_t)info->cvxr - 160;
				else info->cvangle1 = 0;
				
				int16_t oldX = info->X;
				int16_t oldY = info->Y;
				#if IamcarA
				info->X = info->AX;
				info->Y = info->AY;
				#else
				info->X = info->BX;
				info->Y = info->BY;
				#endif
				info->Y = 280 - info->Y;
				info->P1Y = 280 - info->P1Y;
				info->P2Y = 280 - info->P2Y;
				//防上位机坐标乱跳
				#if CVDEBUG == 0
				if (state < 10)
				{
					if (info->X <= 0 || info->X >= 280 || info->Y <= 0 || info->Y >= 280)// || (oldX-info->X)*(oldX-info->X) + (oldY-info->Y)*(oldY-info->Y) > 30)
					{
						wrong++;
						info->X = oldX;
						info->Y = oldY;
						if (wrong > 3) info->renewed = 0;
					}
					else wrong = 0;
				}
				#endif
				//转换格点坐标
				info->P1BX = smalltobig(info->P1X, 56, 206);
				info->P1BY = smalltobig(info->P1Y, 72, 222);
				info->P2BX = smalltobig(info->P2X, 56, 206);
				info->P2BY = smalltobig(info->P2Y, 72, 222);
				newtime = HAL_GetTick();
			}
			data_cnt = 0;
			break;
		default:
			data_cnt = 0;		
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		switch(bt_cnt)
		{
			case 1:
			if(0x31 == btRxbuf[0])
				bt_cnt = 2;
			else
				bt_cnt = 0;
			break;
			case 3:
				bt_cnt = 4;
			break;
		}
	}
	else if(huart == &huart3)
	{
		switch(data_cnt)
		{
			case 1:
			if(0xff == dataRxbuf[0])
				data_cnt = 2;
			else
				data_cnt = 0;
			break;
			case 3:
				data_cnt = 4;
			break;
		}
	}
}

/* USER CODE BEGIN 1 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
/* USER CODE END 1 */
