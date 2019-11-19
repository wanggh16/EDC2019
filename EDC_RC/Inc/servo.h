#ifndef __SERVO_H__
#define __SERVO_H__

void user_Servo_Init(void);
void Set_Servo(uint8_t servo,uint16_t pwm);

typedef struct{
	uint16_t s1;
	uint16_t s2;
	uint16_t s3;
	uint16_t s4;
}servotype;

void Servo_task(servotype* sv);

#endif
