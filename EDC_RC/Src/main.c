/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "motor.h"
//#include "ps2.h"
#include "mecanum.h"
//#include "ultrasonic.h"
#include "oled.h"
#include "bluetooth.h"
//#include "battery.h"
#include "servo.h"
//#include "mpu9250.h"
#include "config.h"
#include "math.h"
#include "maze_pathfinding.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*use global variables for easier online debug*/
mt_ctrltype common_mt_ctrl = {0,0,0,2};
pidtype motors[4]={{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
speed3axistype speed_xyr={0,0,0,0};
gameinfo info={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//servotype servos={1500,1500,800,1700};

int16_t inityaw = 0;
char state = 0;
int8_t move_en = 0;
uint32_t newtime = 0;
char findline1 = 0, findline2 = 0;
char slowdown1 = 0, slowdown2 = 0;

//uint8_t debugpid = 0;
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//delay微秒
/*
static void uDelay(uint32_t t)
{
	uint32_t i;
	for(i=0;i<t;++i)
		__nop();
}
*/
//计算角度差，范围-1799~1800
int16_t diffyaw(int16_t yawp,int16_t yawn)
{
	int16_t ans = yawn - yawp;
	if (ans >= 1800) ans -= 3600;
	else if (ans < -1800) ans += 3600;
	return ans;
}
//计算到目标绝对角度
int16_t getangle(int16_t carx,int16_t cary,int16_t targetx,int16_t targety)
{
	return 572.96*atan2(targety-cary, targetx-carx);
}
//计算到目标距离
int16_t getdist(int16_t carx,int16_t cary,int16_t targetx,int16_t targety)
{
	return sqrt((targetx-carx)*(targetx-carx)+(targety-cary)*(targety-cary));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*for display*/
	/*to lowwer the oled flash frequency*/
	#define PRINT_PERIOD 500;
	uint32_t time_print = 100;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	Set_blink(2,1000);//ms
	user_Motor_Init();
	Motor_unlock(&common_mt_ctrl);
	
	#if PID_EN == 1
	//Motor_PID_Enable(&common_mt_ctrl,motors,1.045, 0.09, 0);
	Motor_PID_Enable(&common_mt_ctrl,motors,1.2, 0.15, 0);
	#endif
	
	//OLED_Init();
	//OLED_Clear();
	//OLED_ShowString(0,0,(uint8_t *)"lyhnb",8);
	//uDelay(1);
	
	user_Servo_Init();
	//Set_Servo(3,600);
	#if BALLMODE==1
	Set_Servo(4,1600);
	#else
	Set_Servo(4,0);
	#endif
	
	//主状态机
	#define IDLE 0
	#define UP_FORWARD 1
	#define UP_TURN 2
	#define CV_FORWARD 11
	#define CV_STOPSOON 13
	#define CV_TURNLEFT 14
	#define CV_TURNRIGHT 15
	#define CV_TURNBACK 16
	#define CV_LEFT 17
	#define CV_RIGHT 18
	#define CV_BACK 19
	#define BLIND_FORWARD 21
	state = IDLE;
	
	//迷宫内小车方向
	#define UP 1
	#define LEFT 2
	#define DOWN 3
	#define RIGHT 4
	int8_t dir = UP;
	
	//红外传感器引脚
	#define IR_LEFT_F GPIO_PIN_12
	#define IR_RIGHT_F GPIO_PIN_13
	#define IR_LEFT_B GPIO_PIN_8
	#define IR_RIGHT_B GPIO_PIN_10	//PORT B,NOT C!!!
	#define IR_BACK GPIO_PIN_15
	#define IR_FRONT GPIO_PIN_14
	
	//迷宫内行进速度、比例系数、90度转向实际大小
	#define SPEED_FORWARD 1600
	#define SPEED_FORWARD_SLOW 300
	#define SPEED_TURN 1200
	#define SPEED_TURN_SLOW 500
	#define SPEED_LR 600
	#define SPEED_LR_SLOW 300
	#define SPEED_BACK 600
	#define SPEED_BACK_SLOW 300
	#define KP_X 6
	#define KP_R 8
	#define KD_X 40
	//#define KD_R 10
	#define TURN_SLOW_YAW 550
	#define TURN_STOP_YAW 800
	
	//外面没有动态规划，可以写死
	#if BALLMODE == 1
	uint16_t target_list [20][2] = {{44, 23}, {44, 42}, {22, 56}, {23, 248}, {16, 258}, {22, 192}, {40, 190}, {224, 42}, {224, 17}, {48, 17}, {24, 17}, {15, 17}};
	#else
	uint16_t target_list [20][2] = {{257, 233},{240, 233},{227, 258},{94, 258},{90, 240}};
	#endif
	//正走还是倒车
	char reverse = 0;
	//当前走到了哪个点
	char stage = 0;
	char nextstate = 0;
	
	//当前目标点坐标
	uint16_t TARGETX = 0;
	uint16_t TARGETY = 0;
	
	//迷宫内当前格点坐标
	int8_t big_x = 0;
	int8_t big_y = 0;
	
	//角度和距离临时变量
	int16_t dst = 0;
	int16_t dy = 0;
	int16_t tempyaw = 0;
	uint8_t cvxpos_last= 80;
	//uint8_t cvangle_last;
	
	//连续看到横线次数
	char continous = 0;
	char slowdown = 0;
	
	//出迷宫
	char arrived = 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//接收蓝牙和其他数据
		BT_task(&common_mt_ctrl,motors,&speed_xyr);
		data_task(&info);
		
		if(HAL_GetTick() >= time_print)
		{
			//printf("1X:%d,1Y:%d\n",info.P1BX,info.P1BY);
			//printf("2X:%d,2Y:%d\n",info.P2BX,info.P2BY);
			//printf("X:%d,Y:%d\n",info.X,info.Y);
			//printf("bigX:%d,bigY:%d\n",big_x,big_y);
			//printf("yaw:%d\n",info.yaw);
			//printf("dir:%d\n",dir);
			//printf("state:%d\n",state);
			//printf("a:%d\n",info.cvangle1);
			printf("x:%d\n",info.cvxpos1);
			//printf("y:%d\n",info.cvypos1);
			printf("%d%d%d%d%d%d",HAL_GPIO_ReadPin(GPIOC,IR_LEFT_B),HAL_GPIO_ReadPin(GPIOC,IR_LEFT_F),HAL_GPIO_ReadPin(GPIOC,IR_FRONT),HAL_GPIO_ReadPin(GPIOC,IR_RIGHT_F),HAL_GPIO_ReadPin(GPIOB,IR_RIGHT_B),HAL_GPIO_ReadPin(GPIOC,IR_BACK));
			time_print = HAL_GetTick() + PRINT_PERIOD;
		}
		if (info.renewed && info.matchstate && move_en == -1) move_en = 1;
		//else move_en = 0;
		//50ms更新一次
		if(info.renewed && move_en == 1)
		{
			speed_xyr.cal_speed = 1;
			switch(state)
			{
				//初始状态
				case IDLE:
					#if BALLMODE == 1
					inityaw = info.yaw;
					#else
					inityaw = info.yaw + 900;
					if (inityaw > 1800) inityaw -= 3600;
					#endif
					state = UP_TURN;
					TARGETX = target_list[stage][0];
					TARGETY = target_list[stage][1];
					#if CVDEBUG == 1
					state = CV_LEFT;
					#endif
				break;
				//有上位机，转向，准备前往下一点
				case UP_TURN:
					tempyaw = getangle(info.X, info.Y, TARGETX, TARGETY);
					dy = diffyaw(tempyaw, info.yaw);
					if (reverse) dy = diffyaw(1800, dy);
					dst = getdist(info.X, info.Y, TARGETX, TARGETY);
					speed_xyr.x = 0;
					speed_xyr.y = 0;
					if (dy > 400) speed_xyr.r = 800;
					else if (dy < -400) speed_xyr.r = -800;
					else if (dy > 60 || dy < -60) speed_xyr.r = 2*dy;
					else state = UP_FORWARD;
				break;
				//前往下一点
				case UP_FORWARD:
					tempyaw = getangle(info.X, info.Y, TARGETX, TARGETY);
					dy = diffyaw(tempyaw, info.yaw);
					if (reverse) dy = diffyaw(1800, dy);
					dst = getdist(info.X, info.Y, TARGETX, TARGETY);
					if (dst > 30 && dy < 600 && dy > -600)
					{
						speed_xyr.x = 1200*sin(dy/572.96);
						speed_xyr.y = 1200*cos(dy/572.96);
						speed_xyr.r = 0;
					}
					else if (dst > 30)
					{
						state = UP_TURN;
					}
					else if (dst > 10)
					{
						speed_xyr.x = 40*dst*sin(dy/572.96);
						speed_xyr.y = 40*dst*cos(dy/572.96);
						speed_xyr.r = 0;
					}
					else if (dst > 4)
					{
						speed_xyr.x = 40*dst*sin(dy/572.96);
						speed_xyr.y = 40*dst*cos(dy/572.96);
						speed_xyr.r = 0;
					}
					else
					{
						state = UP_TURN;
						stage++;
						if (stage > 19) {stage = 0;move_en = 0;}
						//倒车去小球位置，放钓鱼竿
						//if (TARGETX==10 && TARGETY==194)
						if (TARGETX==44 && TARGETY==42)
						{
							reverse = 1;
						}
						else if (TARGETX==22 && TARGETY==56)
						{
							Set_Servo(4,1000);
						}
						//抓完球恢复正着走，收钓鱼竿
						else if (TARGETX==16 && TARGETY==258)
						{
							reverse = 0;
							Set_Servo(4,1600);
						}
						//进迷宫
						else if (TARGETX==40 && TARGETY==190)
						{
							state = CV_FORWARD;
							dir = RIGHT;
							big_x = -1;
							big_y = 4;
							ClearWalls();
						}
						else if (TARGETX==90 && TARGETY==240)
						{
							state = CV_FORWARD;
							dir = DOWN;
							big_x = 1;
							big_y = 6;
							ClearWalls();
						}
						else if (TARGETX==224 && TARGETY==17)
						{
							reverse = 1;
						}
						else if (TARGETX==15 && TARGETY==17)
						{
							Set_Servo(4,1000);
							//Set_Servo(3,2400);
							move_en = 0;
						}
						TARGETX = target_list[stage][0];
						TARGETY = target_list[stage][1];
					}
					if (reverse)
					{
						speed_xyr.x = -speed_xyr.x;
						speed_xyr.y = -speed_xyr.y;
					}
				break;
				//正常视觉巡线
				case CV_FORWARD:
					if (!(info.cvstate&0x01) && 45 < info.cvxpos && info.cvxpos < 115) speed_xyr.y = SPEED_FORWARD;
					else speed_xyr.y = 0;
					if (!(info.cvstate&0x01) && 45 < info.cvangle && info.cvangle < 135)
					{
						speed_xyr.x = KP_X*(info.cvxpos - 79.5) + KD_X*(info.cvxpos - cvxpos_last);
						speed_xyr.r = -KP_R*(info.cvangle - 90);
						cvxpos_last = info.cvxpos;
					}
					else
					{
						speed_xyr.x = 0;
						speed_xyr.r = 0;
						cvxpos_last = 80;
					}
					if (slowdown1 == 1 && slowdown2 == 1)
					{
						slowdown1 = 0;
						slowdown2 = 0;
						state = CV_STOPSOON;
						findline1 = 0;
						findline2 = 0;
						
						if (dir==UP) big_y++;
						else if (dir==LEFT) big_x--;
						else if (dir==DOWN) big_y--;
						else if (dir==RIGHT) big_x++;
						
						//迷宫寻路
						char leftblock = !(HAL_GPIO_ReadPin(GPIOC,IR_LEFT_F));
						char rightblock = !(HAL_GPIO_ReadPin(GPIOC,IR_RIGHT_F));
						char frontblock = !HAL_GPIO_ReadPin(GPIOC,IR_FRONT);
						char wallrefresh = (leftblock << 2) + (frontblock << 1) + rightblock;
						
						#if BALLMODE == 1
						uint8_t decision = PathFinding(dir,big_x + 1,big_y + 1,5,1,wallrefresh);
						#else
						uint8_t decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh);
						#endif
						
						if (decision == 255)
						{
							ClearWalls();
							decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh);
						}
						switch(decision){
							case 0://arrived
								if (dir == RIGHT)
								{
									nextstate = CV_TURNRIGHT;
									arrived = 1;
								}
							break;
							default:
								nextstate = decision;
							break;
						}
						#if CVDEBUG==1
						nextstate = CV_LEFT;
						#endif
					}
				break;
				//看到交叉点，决定下一步怎么走
				case CV_STOPSOON:
					speed_xyr.y = SPEED_FORWARD_SLOW;
					if (nextstate == CV_FORWARD) speed_xyr.y = SPEED_FORWARD; 
					if (!(info.cvstate&0x01) && 45 < info.cvangle && info.cvangle < 135)
					{
						speed_xyr.x = KP_X*(info.cvxpos - 80);
						speed_xyr.r = -KP_R*(info.cvangle - 90);
					}
					else
					{
						speed_xyr.x = 0;
						speed_xyr.r = 0;
					}
					if (findline1 == 1 && findline2 == 1)
					{
						findline1 = 0;
						findline2 = 0;
						slowdown1 = 0;
						slowdown2 = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						tempyaw = info.yaw;
						state = nextstate;
						if (nextstate == CV_FORWARD) speed_xyr.y = SPEED_FORWARD; 
					}
				break;
				//左转
				case CV_TURNLEFT:
					speed_xyr.r = -SPEED_TURN;
					if (diffyaw(tempyaw, info.yaw)>TURN_SLOW_YAW)
						speed_xyr.r = -SPEED_TURN_SLOW;
					if (diffyaw(tempyaw, info.yaw)>TURN_STOP_YAW)
					{
						speed_xyr.r = 0;
						dir++;
						if (arrived) state = BLIND_FORWARD;
						else state = CV_FORWARD;
						slowdown1 = 0;
						slowdown2 = 0;
						cvxpos_last = 80;
					}
				break;
				//右转
				case CV_TURNRIGHT:
					speed_xyr.r = SPEED_TURN;
					if (diffyaw(tempyaw, info.yaw)<-TURN_SLOW_YAW)
						speed_xyr.r = SPEED_TURN_SLOW;
					if (diffyaw(tempyaw, info.yaw)<-TURN_STOP_YAW)
					{
						speed_xyr.r = 0;
						dir--;
						if (arrived) state = BLIND_FORWARD;
						else state = CV_FORWARD;
						slowdown1 = 0;
						slowdown2 = 0;
						cvxpos_last = 80;
					}
				break;
				//掉头
				case CV_TURNBACK:
					speed_xyr.r = SPEED_TURN;
					if (diffyaw(tempyaw, info.yaw)< -TURN_SLOW_YAW - 900)
						speed_xyr.r = SPEED_TURN_SLOW;
					if (diffyaw(tempyaw, info.yaw)< -TURN_STOP_YAW - 900 || diffyaw(tempyaw, info.yaw) > 900)
					{
						speed_xyr.r = 0;
						dir += 2;
						if (arrived) state = BLIND_FORWARD;
						else state = CV_FORWARD;
						slowdown1 = 0;
						slowdown2 = 0;
						cvxpos_last = 80;
					}
				break;
				//左平移
				case CV_LEFT:
					if (slowdown) speed_xyr.x = -SPEED_LR_SLOW;
					else if (!(info.cvstate&0x04) && 55 < info.cvxpos1 && info.cvxpos1 < 105) speed_xyr.x = -SPEED_LR;
					else speed_xyr.x = 0;
					if (!(info.cvstate&0x04) && 45 < info.cvangle1 && info.cvangle1 < 135)
					{
						speed_xyr.y = -4*(info.cvxpos1 - 80);
						speed_xyr.r = -3*(info.cvangle1 - 90);
					}
					else
					{
						speed_xyr.y = 0;
						speed_xyr.r = 0;
					}
					if (!(info.cvstate&0x01) && info.cvxpos < 40)
					{
						slowdown = 1;
					}
					if (!(info.cvstate&0x01) && 40 <= info.cvxpos && slowdown)
					{
						slowdown = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						if (dir==UP) big_x--;
						else if (dir==LEFT) big_y--;
						else if (dir==DOWN) big_x++;
						else if (dir==RIGHT) big_y++;
						#if CVDEBUG
						state = CV_BACK;
						#endif
					}
				break;
				//右平移
				case CV_RIGHT:
					if (slowdown) speed_xyr.x = SPEED_LR_SLOW;
					else if (!(info.cvstate&0x04) && 55 < info.cvxpos1 && info.cvxpos1 < 105) speed_xyr.x = SPEED_LR;
					else speed_xyr.x = 0;
					if (!(info.cvstate&0x04) && 45 < info.cvangle1 && info.cvangle1 < 135)
					{
						speed_xyr.y = -4*(info.cvxpos1 - 80);
						speed_xyr.r = -3*(info.cvangle1 - 90);
					}
					else
					{
						speed_xyr.y = 0;
						speed_xyr.r = 0;
					}
					if (!(info.cvstate&0x01) && info.cvxpos > 120)
					{
						slowdown = 1;
					}
					if (!(info.cvstate&0x01) && 120 >= info.cvxpos && slowdown)
					{
						slowdown = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						if (dir==UP) big_x++;
						else if (dir==LEFT) big_y++;
						else if (dir==DOWN) big_x--;
						else if (dir==RIGHT) big_y--;
						#if CVDEBUG
						state = CV_FORWARD;
						#endif
					}
				break;
				//倒车
				case CV_BACK:
					if (slowdown) speed_xyr.y = -SPEED_BACK_SLOW;
					else if (!(info.cvstate&0x01) && 55 < info.cvxpos && info.cvxpos < 105) {speed_xyr.y = -SPEED_BACK;continous++;}
					else speed_xyr.y = 0;
					if (!(info.cvstate&0x01) && 45 < info.cvangle && info.cvangle < 135)
					{
						speed_xyr.x = 6*(info.cvxpos - 79.5);
						speed_xyr.r = -4*(info.cvangle - 90);
					}
					else
					{
						speed_xyr.x = 0;
						speed_xyr.r = 0;
					}
					if (!(info.cvstate&0x04) && info.cvxpos1 > 120)
					{
						slowdown = 1;
					}
					if (continous==20) {findline1=0;findline2=0;}
					if (findline1 == 1 && findline2 == 1 && continous > 20)
					{
						continous = 0;
						findline1 = 0;
						findline2 = 0;
						slowdown = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						if (dir==UP) big_y--;
						else if (dir==LEFT) big_x++;
						else if (dir==DOWN) big_y++;
						else if (dir==RIGHT) big_x--;
						#if CVDEBUG
						state = CV_RIGHT;
						#endif
					}
				break;
				//冲出迷宫的最后一步
				case BLIND_FORWARD:
					speed_xyr.y = SPEED_FORWARD;
					if (!(info.cvstate&0x01) && 45 < info.cvangle && info.cvangle < 135)
					{
						speed_xyr.x = KP_X*(info.cvxpos - 79.5);
						speed_xyr.r = -KP_R*(info.cvangle - 90);
					}
					else
					{
						speed_xyr.x = 0;
						speed_xyr.r = 0;
					}
					continous++;
					if (continous > 24)
					{
						state = UP_TURN;
						continous = 0;
					}
				break;
				default:
					state = IDLE;
			}
			info.renewed = 0;
			if (dir > 4) dir -= 4;
			else if (dir < 1) dir += 4;
		}
		if(move_en != 1 || (HAL_GetTick() - newtime > 130))
		{
			speed_xyr.cal_speed = 1;
			speed_xyr.x = 0;
			speed_xyr.y = 0;
			speed_xyr.r = 0;
		}
		if(speed_xyr.cal_speed != 0)
		{
			cal_mecanum(&speed_xyr,&common_mt_ctrl,motors);
			speed_xyr.cal_speed = 0;
		}
		Get_cnt(motors);
		Motor_task(&common_mt_ctrl,motors);
		//Servo_task(&servos);
		
		#if IWDG_EN == 1
		HAL_IWDG_Refresh(&hiwdg);
		#endif
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);//timing debug
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 3;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration  
  PB13   ------> TIM1_CH1N
  PB14   ------> TIM1_CH2N
  PB15   ------> TIM1_CH3N
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  PA10   ------> TIM1_CH3
  PA11   ------> TIM1_CH4 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC5 
                           PC8 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (state == CV_STOPSOON || state == CV_BACK)
  {
    if (GPIO_Pin == GPIO_PIN_3) findline1 = 1;
		if (GPIO_Pin == GPIO_PIN_4) findline2 = 1;
  }
	else if (state == CV_FORWARD)
  {
    if (GPIO_Pin == GPIO_PIN_6) slowdown1 = 1;
		if (GPIO_Pin == GPIO_PIN_7) slowdown2 = 1;
  } 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
