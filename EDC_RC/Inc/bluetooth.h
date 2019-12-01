#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
//#define UART_PORT huart2

typedef struct{
	char renewed;
	char matchstate;
	int16_t yaw;
	int16_t X;
	int16_t Y;
	int8_t P1BX;
	int8_t P1BY;
	int8_t P2BX;
	int8_t P2BY;
	int16_t AX;
	int16_t AY;
	int16_t BX;
	int16_t BY;
	int16_t P1X;
	int16_t P1Y;
	int16_t P2X;
	int16_t P2Y;
	int16_t BALLX;
	int16_t BALLY;
	int8_t cvstate;
	uint8_t cvxpos;
	uint8_t cvangle;
	uint8_t cvypos;
}gameinfo;

void BT_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed);
void data_task(gameinfo *info);
char smalltobig(int16_t x, int16_t lower, int16_t upper);
#endif
