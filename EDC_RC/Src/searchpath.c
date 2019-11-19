#include "main.h"
#include "stm32f1xx_hal.h"
#include "searchpath.h"
#include "motor.h"

typedef struct{
	uint32_t time;
	uint8_t senser;
}optictype;
uint8_t bufindex = 0;
optictype optic[BUFSIZE];

void Searchpath(int16_t *speed,motortype *mt)
{
	uint8_t senser = 0;
	static uint8_t senser_last = 0;
	speed[0] = 0;
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SENSER_BANK,SENSER_L))
		senser |= 0x04;
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SENSER_BANK,SENSER_M))
		senser |= 0x02;
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SENSER_BANK,SENSER_R))
		senser |= 0x01;
	
	if(senser != senser_last)
	{
		++bufindex;
		bufindex &= BUFMASK;
		optic[bufindex].time = HAL_GetTick();
		optic[bufindex].senser = senser;
	}
	
	switch(senser)
	{
		case 0:
			speed[1] = 1000;
			speed[2] = 0;
			break;
		
		case 1:
			speed[1] = 500;
			speed[2] = -1000;
			break;
		
		case 2:
			speed[1] = 1000;
			speed[2] = 0;
			break;
		
		case 3:
			speed[1] = 500;
			speed[2] = -1000;
			break;
		
		case 4:
			speed[1] = 500;
			speed[2] = 1000;
			break;
		
		case 5:
			speed[1] = 1000;
			speed[2] = 0;
			break;
		
		case 6:
			speed[1] = 500;
			speed[2] = 1000;
			break;

		case 7:
			speed[1] = 1000;
			speed[2] = 0;
			break;		
	}
}
