#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "mecanum.h"

void cal_mecanum(speed3axistype *speed,mt_ctrltype *ctrl,pidtype *mt)
{
	int16_t x,y,r;
	int16_t RF,LF,RB,LB;
	int16_t xrf,yrf,rrf;//????
  int16_t xlf,ylf,rlf;
  int16_t xrb,yrb,rrb;
	int16_t xlb,ylb,rlb;
	int16_t max;
	float tmp;

	y=speed->y;
	x=speed->x;
	r=speed->r-speed->y/30;
	
	/*????x,y,r??*/
	xrf=-x;
	xlf=-x;
	xrb=x;
	xlb=x;
								
	yrf=y;
	ylf=-y;
	yrb=y;
	ylb=-y;
								
	rrf=-r;
	rlf=-r;
	rrb=-r;
	rlb=-r;										 
				
	/*????*/
	RF=xrf+yrf+rrf;
	LF=xlf+ylf+rlf;
	RB=xrb+yrb+rrb;
	LB=xlb+ylb+rlb;												

 if(RF>=DUTY_MAX||LF>=DUTY_MAX||RB>=DUTY_MAX||LB>=DUTY_MAX)//?????????????
 {
	 max=RF;
	 if(LF>max)max=LF;
	 if(RB>max)max=RB;
	 if(LB>max)max=LB;
	
	 tmp=(float)DUTY_MAX/(float)max;//??
	 RF=RF*tmp;
	 LF=LF*tmp;
	 RB=RB*tmp;
	 LB=LB*tmp; 
	}			
	//changed by wgh
	mt[0].target=LB;
	mt[1].target=RB;
	mt[2].target=LF;
	mt[3].target=RF;
	//mt[0].target=LF;
	//mt[1].target=RF;
	//mt[2].target=LB;
	//mt[3].target=RB;
	
	ctrl->motor_update = 1;
}
