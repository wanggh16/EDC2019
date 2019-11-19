#include "main.h"
#include "stm32f1xx_hal.h"
#include "tcs34725.h"

#pragma pack(2)
extern I2C_HandleTypeDef HIIC;
//HAL_StatusTypeDef err;
//uint8_t id;
uint8_t gain = 0;
/*
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//tcsbusy = 0;
}*/

void TCS_task(colortype *clr)
{
	static uint32_t time_next = 0;
	uint32_t time_curr = HAL_GetTick();
	
	if(time_curr >= time_next)
	{
		tcs_Read_reg(&HIIC,TCS_DAT|TCS_WORD_MODE,(uint8_t *)clr,8);
		time_next = time_curr + 30;
	}
}

void TCS_Getcolor(colortype *clr)
{
	uint16_t max,min;
	int16_t r,g,b;
	/*get max and min*/
	if(clr->rdata >= clr->gdata)
	{
		max = clr->rdata;
		min = clr->gdata;
	}
	else
	{
		min = clr->rdata;
		max = clr->gdata;
	}
	if(clr->bdata > max)
		max = clr->bdata;
	else if(clr->bdata < min)
		min = clr->bdata;
	
	r = clr->rdata;
	g = clr->gdata;
	b = clr->bdata;
	/*hsv*/
	if(max == min)
    	clr->color_h = 0;
  else
	{
		if(r == max)
		{
			if(g >= b)
	    		clr->color_h = 60*(g - b)/(max - min);
	    else
	    		clr->color_h = 60*(g - b)/(max - min) + 360;  	
		}
	  else if(g == max)
	    	clr->color_h = 60*(b - r)/(max-min) + 120;
	  else if(b == max)
	    	clr->color_h = 60*(r - g)/(max-min) + 240;
    }    
   	
   	if(max == 0)
   		clr->color_s = 0;
   	else
   		clr->color_s = 1.0- (float)min / (float)max;
    
    clr->color_v = max;

		/*AGC*/
		if(clr->cdata < 0x000f && gain < 3)
		{
			++gain;
			tcs_Write_reg(&HIIC,TCS_CTRL,gain);
		}
		else if(clr->cdata > 0xFFF0 && gain > 0)
		{
			--gain;
			tcs_Write_reg(&HIIC,TCS_CTRL,gain);
		}
			
}

void TCS_Init(void)
{
	//tcs_Read_reg(&HIIC,TCS_ID,&id,1);
	//while(tcsbusy == 1);
	tcs_Write_reg(&HIIC,TCS_ATIME,TCS_24MS);
	tcs_Write_reg(&HIIC,TCS_EN,TCS_AEN|TCS_PON);
	HAL_Delay(3);
}
