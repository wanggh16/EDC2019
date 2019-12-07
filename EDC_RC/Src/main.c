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
  * OF THIS SOFTWARE, EVEN if ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
//#include "oled.h"
#include "bluetooth.h"
//#include "battery.h"
#include "servo.h"
//#include "mpu9250.h"
#include "config.h"
#include "math.h"
#include "maze_pathfinding.h"
#include <stdio.h>
#include <stdlib.h>

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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*use global variables for easier online debug*/
extern uint8_t data_cnt;
mt_ctrltype common_mt_ctrl = {0,0,0,2};
pidtype motors[4]={{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
speed3axistype speed_xyr={0,0,0,0};
gameinfo info={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//servotype servos={1500,1500,800,1700};

int16_t inityaw = 0;
char state = 0;
int8_t move_en = 0;
uint32_t newtime = 0;
char find_left = 0, find_right = 0, find_left_f = 0, find_right_f = 0, find_left_b = 0, find_right_b = 0, find_front = 0, find_back = 0;
//迷宫内当前格点坐标
int8_t big_x = 0;
int8_t big_y = 0;
//uint8_t debugpid = 0;
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
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
void clear_all_irs(){find_left = 0;find_right = 0;find_left_f = 0;find_right_f = 0;find_left_b = 0;find_right_b = 0;find_front = 0;find_back = 0;}
void clear_all_irs_except_fb(){find_left = 0;find_right = 0;find_left_f = 0;find_right_f = 0;find_left_b = 0;find_right_b = 0;}
void clear_all_irs_except_lr(){find_left_f = 0;find_right_f = 0;find_left_b = 0;find_right_b = 0;find_front = 0;find_back = 0;}
//void clear_all_irs_except_f(){find_left = 0;find_right = 0;find_left_b = 0;find_right_b = 0;find_front = 0;find_back = 0;}
//void clear_all_irs_except_b(){find_left = 0;find_right = 0;find_left_f = 0;find_right_f = 0;find_front = 0;find_back = 0;}
//void clear_all_irs_except_l(){find_left = 0;find_right = 0;find_right_f = 0;find_right_b = 0;find_front = 0;find_back = 0;}
//void clear_all_irs_except_r(){find_left = 0;find_right = 0;find_left_f = 0;find_left_b = 0;find_front = 0;find_back = 0;}
//void clear_fb(){find_front = 0;find_back = 0;}
//void clear_lr(){find_left = 0;find_right = 0;}
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
	Motor_PID_Enable(&common_mt_ctrl,motors,1.2, 0.10, 0);
	#endif
	
	user_Servo_Init();
	//Set_Servo(3,600);
	#if BALLMODE==1
	Set_Servo(4,1000);
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
	
	//红外传感器引脚 PORT C
	#define IR_LEFT_F GPIO_PIN_12
	#define IR_RIGHT_F GPIO_PIN_13
	#define IR_LEFT_B GPIO_PIN_8
	#define IR_RIGHT_B GPIO_PIN_10	//PORT B,NOT C!!!
	#define IR_BACK GPIO_PIN_15
	#define IR_FRONT GPIO_PIN_14
	
	//小红外 PORT C
	#define ir_left_f GPIO_PIN_7
	#define ir_right_f GPIO_PIN_6
	#define ir_left GPIO_PIN_4
	#define ir_right GPIO_PIN_3
	#define ir_front GPIO_PIN_9 //PORT B
	#define ir_back GPIO_PIN_8 //PORT B
	#define ir_left_b GPIO_PIN_1
	#define ir_right_b GPIO_PIN_0 //PORT B
	
	//迷宫内行进速度、比例系数、90度转向实际大小
	#define SPEED_FORWARD 1300
	#define SPEED_FORWARD_SLOW 300
	#define SPEED_TURN 1200
	#define SPEED_TURN_SLOW 500
	#define SPEED_LR 1000
	#define SPEED_LR_SLOW 300
	#define SPEED_BACK 1300
	#define SPEED_BACK_SLOW 300
	#define KP_X 4                                                         
	#define KP_X1 5                                                          
	#define KP_YAW 2.5
	#define KD_YAW 6
	#define YAW_PERMIT 120
	#define TURN_SLOW_YAW 500
	#define TURN_STOP_YAW 800
	#define SPEED_CORR 900
	#define SPEED_FORWARD_BALL 800
	
	//外面没有动态规划，可以写死
	#if BALLMODE == 1
	uint16_t target_list [20][2] = {{42, 18}, {44, 38}, {22, 52}, {24, 244}, {19, 260}, {20, 192}, {38, 190}, {234, 39}, {234, 19}, {48, 18}, {24, 18}, {15, 18}};
	#else
	//uint16_t target_list [20][2] = {{257, 237},{240, 235},{227, 258},{93, 260},{93, 240}};
	uint16_t target_list [20][2] = {{262, 237},{240, 235},{240, 104},{230, 103}};
	#endif
	//正走还是倒车
	char reverse = 0;
	//当前走到了哪个点
	char stage = 0;
	char nextstate = 0;
	
	//当前目标点坐标
	uint16_t TARGETX = 0;
	uint16_t TARGETY = 0;
	
	//角度和距离临时变量
	int16_t dst = 0;
	int16_t dy = 0;
	int16_t tempyaw = 0;
	int16_t yaw_last = 0;
	int16_t yaw_diff = 0;
	int16_t yaw_err = 0;
	//uint8_t cvangle_last;
	
	//连续看到横线次数
	unsigned char continous = 0;
	char slowdown = 0;
	char preslowdown = 0;
	int8_t rand = 0;
	char noslowdown = 0;
	char stopped = 0;
	//出迷宫
	char arrived = 0;
	int anti = 0;
	int anti1 = 0;
	
	#if CVDEBUG == 1
	char states[] = {CV_FORWARD, CV_LEFT, CV_BACK, CV_RIGHT, CV_LEFT, CV_RIGHT, CV_FORWARD, CV_BACK, CV_TURNLEFT, CV_TURNRIGHT, CV_TURNBACK, CV_LEFT, CV_TURNBACK, CV_BACK};
	char statedebug = 1;
	#endif
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
		if (HAL_GetTick() >= time_print && move_en != 1)
		{
			#if CVDEBUG == 1
			printf("debugstage:%d\n", statedebug);
			#endif
			printf("mode:%d\n", BALLMODE);
			printf("state:%d %d %d\n",info.renewed,move_en,state);
			//printf("1X:%d,1Y:%d\n",info.P1BX,info.P1BY);
			//printf("2X:%d,2Y:%d\n",info.P2BX,info.P2BY);
			printf("X:%d,Y:%d\n",info.X,info.Y);
			printf("bigX:%d,bigY:%d\n",big_x,big_y);
			printf("yaw:%d %d\n",info.yaw,yaw_diff);
			//printf("dir:%d\n",dir);
			printf("cv:%d %d %d %d\n",info.cvxf,info.cvxb,info.cvxl,info.cvxr);
			//printf("cvp:%d %d %d %d\n",info.cvxpos,info.cvangle,info.cvxpos1,info.cvangle1);
			printf("t:%d\n",HAL_GetTick() - newtime);
			printf("%d%d%d%d%d%d\n",HAL_GPIO_ReadPin(GPIOC,IR_LEFT_B),HAL_GPIO_ReadPin(GPIOC,IR_LEFT_F),HAL_GPIO_ReadPin(GPIOC,IR_FRONT),HAL_GPIO_ReadPin(GPIOC,IR_RIGHT_F),HAL_GPIO_ReadPin(GPIOB,IR_RIGHT_B),HAL_GPIO_ReadPin(GPIOC,IR_BACK));
			printf("%d%d%d%d%d%d%d%d\n",HAL_GPIO_ReadPin(GPIOC,ir_left),HAL_GPIO_ReadPin(GPIOC,ir_left_f),HAL_GPIO_ReadPin(GPIOB,ir_front),HAL_GPIO_ReadPin(GPIOC,ir_right_f),HAL_GPIO_ReadPin(GPIOC,ir_right),HAL_GPIO_ReadPin(GPIOB,ir_right_b),HAL_GPIO_ReadPin(GPIOB,ir_back),HAL_GPIO_ReadPin(GPIOC,ir_left_b));
			time_print = HAL_GetTick() + PRINT_PERIOD;
		}
		if (info.renewed && info.matchstate && (move_en == -1)) move_en = 1;
		//else move_en = 0;
		//20ms更新一次
		if(info.renewed && move_en == 1)
		{
			#if CVDEBUG == 1
			if (statedebug > 14) statedebug = 1;
			//printf("%d ",state);
			#endif
			//printf("%d ",state);
			yaw_diff = diffyaw(yaw_last, info.yaw);
			yaw_last = info.yaw;
			if (dir == UP) yaw_err = diffyaw(900, info.yaw);
			else if (dir == RIGHT) yaw_err = info.yaw;
			else if (dir == DOWN) yaw_err = diffyaw(-900, info.yaw);
			else if (dir == LEFT) yaw_err = diffyaw(1800, info.yaw);
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
					state = CV_FORWARD;
					clear_all_irs();
					dir = RIGHT;
					printf("only once\n");
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
					if (TARGETX == 240 && TARGETY == 235) 
					{
						state = UP_FORWARD;
						break;
					}
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
					if (dst > 40 && dy < 600 && dy > -600)
					{
						speed_xyr.x = 1200*sin(dy/572.96);
						speed_xyr.y = 1200*cos(dy/572.96);
						speed_xyr.r = 0;
					}
					else if (dst > 40)
					{
						state = UP_TURN;
					}
					else if (dst > 10)
					{
						speed_xyr.x = 30*dst*sin(dy/572.96);
						speed_xyr.y = 30*dst*cos(dy/572.96);
						speed_xyr.r = 0;
					}
					else if (dst > 4)
					{
						speed_xyr.x = 30*dst*sin(dy/572.96);
						speed_xyr.y = 30*dst*cos(dy/572.96);
						speed_xyr.r = 0;
					}
					else
					{
						state = UP_TURN;
						stage++;
						//倒车去小球位置，放钓鱼竿
						//if (TARGETX==10 && TARGETY==194)
						if (TARGETX==44 && TARGETY==38)
						{
							reverse = 1;
						}
						else if (TARGETX==22 && TARGETY==52)
						{
							Set_Servo(4,1400);
						}
						//抓完球恢复正着走，收钓鱼竿
						else if (TARGETX==19 && TARGETY==260)
						{
							reverse = 0;
							Set_Servo(4,1000);
						}
						//进迷宫
						else if (TARGETX==38 && TARGETY==190)
						{
							state = CV_FORWARD;
							dir = RIGHT;
							big_x = -1;
							big_y = 4;
							ClearWalls();
							clear_all_irs();
							preslowdown = 1;
						}
						else if (TARGETX==93 && TARGETY==240)
						{
							state = CV_FORWARD;
							dir = DOWN;
							big_x = 1;
							big_y = 6;
							ClearWalls();
							clear_all_irs();
							preslowdown = 1;
						}
						else if (TARGETX==230 && TARGETY==103)
						{
							state = CV_FORWARD;
							dir = LEFT;
							big_x = 6;
							big_y = 1;
							ClearWalls();
							clear_all_irs();
							preslowdown = 1;
						}
						else if (TARGETX==234 && TARGETY==19)
						{
							reverse = 1;
							Set_Servo(4,1000);
						}
						else if (TARGETX==15 && TARGETY==18)
						{
							Set_Servo(4,1400);
							//Set_Servo(3,2400);
							move_en = -3;
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
					if ((!stopped) && -40 < info.cvxpos && info.cvxpos < 40 && -YAW_PERMIT < yaw_err && yaw_err < YAW_PERMIT)
					{
						#if BALLMODE == 1
						speed_xyr.y = SPEED_FORWARD_BALL;
						#else
						speed_xyr.y = SPEED_FORWARD;
						#endif
						continous++;
						speed_xyr.x = KP_X*info.cvxpos;
						//speed_xyr.r = KP_R*info.cvangle + KD_YAW*yaw_diff;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					else
					{
						speed_xyr.y = 0;
						speed_xyr.x = KP_X1*info.cvxpos;
						//speed_xyr.r = KP_R1*info.cvangle;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					//speed_xyr.x = KP_X*info.cvxpos;
					//speed_xyr.r = KP_R*info.cvangle;
					if (anti1 > 0){speed_xyr.x = SPEED_CORR;anti1--;}
					else if (anti1 < 0){speed_xyr.x = -SPEED_CORR;anti1++;}
					
					if ((find_left_b && find_right_b) && (!preslowdown))
					{
						preslowdown = 1;
						//clear_all_irs_except_f();
						clear_all_irs();
					}
					if ((find_left_f && find_right_f && preslowdown) || continous > 100)
					{
						continous = 0;
						clear_all_irs_except_lr();
						state = CV_STOPSOON;
						
						if (dir==UP) big_y++;
						else if (dir==LEFT) big_x--;
						else if (dir==DOWN) big_y--;
						else if (dir==RIGHT) big_x++;
						#if BIGDEBUG == 1
						printf("bigX:%d,bigY:%d\n",big_x,big_y);
						#endif
						
						//迷宫寻路
						char leftblock = !(HAL_GPIO_ReadPin(GPIOC,IR_LEFT_F));
						char rightblock = !(HAL_GPIO_ReadPin(GPIOC,IR_RIGHT_F));
						char frontblock = !(HAL_GPIO_ReadPin(GPIOC,IR_FRONT));
						char wallrefresh = (leftblock << 2) + (frontblock << 1) + rightblock;
						
						#if BALLMODE == 1
						uint8_t decision = PathFinding(dir,big_x + 1,big_y + 1,5,1,wallrefresh);
						#else
						uint8_t decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						#endif
						
						if (decision == 255)
						{
							ClearWalls();
							decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
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
						nextstate = states[statedebug];
						statedebug++;
						#endif
						if (nextstate == CV_FORWARD) speed_xyr.y = SPEED_FORWARD;
						else speed_xyr.y = SPEED_FORWARD_SLOW;
					}
				break;
				//看到交叉点，决定下一步怎么走
				case CV_STOPSOON:
					if ((!stopped) && -40 < info.cvxpos && info.cvxpos < 40 && -YAW_PERMIT < yaw_err && yaw_err < YAW_PERMIT)
					{
						if (nextstate == CV_FORWARD) speed_xyr.y = SPEED_FORWARD;
						else speed_xyr.y = SPEED_FORWARD_SLOW;
						continous++;
						speed_xyr.x = KP_X*info.cvxpos;
						//speed_xyr.r = KP_R*info.cvangle + KD_YAW*yaw_diff;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					else
					{
						speed_xyr.y = 0;
						speed_xyr.x = KP_X1*info.cvxpos;
						//speed_xyr.r = KP_R1*info.cvangle;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					//speed_xyr.x = KP_X*info.cvxpos;
					//speed_xyr.r = KP_R*info.cvangle;
					
					if ((find_left && find_right) || (find_left_b && find_right_b) || continous > 100)
					{
						clear_all_irs();
						preslowdown = 0;
						continous = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						tempyaw = info.yaw;
						state = nextstate;
						if (nextstate == CV_FORWARD) speed_xyr.y = SPEED_FORWARD;
						else if (state == CV_TURNLEFT || state == CV_TURNRIGHT || state == CV_TURNBACK) speed_xyr.y = 0;
						else {
							speed_xyr.y = -SPEED_CORR;
							anti = 1;
						}
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
						clear_all_irs();
						preslowdown = 0;
						slowdown = 0;
						if (arrived) {state = BLIND_FORWARD;break;}
						else state = CV_FORWARD;
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
						clear_all_irs();
						preslowdown = 0;
						slowdown = 0;
						if (arrived) {state = BLIND_FORWARD;break;}
						else state = CV_FORWARD;
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
						clear_all_irs();
						preslowdown = 0;
						slowdown = 0;
						if (arrived) {state = BLIND_FORWARD;break;}
						else state = CV_FORWARD;
					}
				break;
				//左平移
				case CV_LEFT:
					if (find_right_f && find_right_b && (!preslowdown))
					{
						preslowdown = 1;
						//clear_all_irs_except_l();
						clear_all_irs();
					}
					if (find_left_f && find_left_b && preslowdown && (!slowdown))
					{
						slowdown = 1;
						clear_all_irs_except_fb();
					}
					if ((!stopped) && -36 < info.cvxpos1 && info.cvxpos1 < 36 && -YAW_PERMIT < yaw_err && yaw_err < YAW_PERMIT) 
					{
						if (slowdown && (!noslowdown)) {speed_xyr.x = -SPEED_LR_SLOW;if (rand == 2) {continous++;rand = 0;} else rand++;}
						else {speed_xyr.x = -SPEED_LR;continous++;}
						speed_xyr.y = KP_X*info.cvxpos1;
						//speed_xyr.r = KP_R*info.cvangle1 + KD_YAW*yaw_diff;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					else
					{
						speed_xyr.x = 0;
						speed_xyr.y = KP_X1*info.cvxpos1;
						//speed_xyr.r = KP_R1*info.cvangle1;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					//speed_xyr.y = KP_X*info.cvxpos1;
					//speed_xyr.r = KP_R*info.cvangle1;
					if (anti > 0){speed_xyr.y = -SPEED_CORR;anti--;}
					else if (anti < 0){speed_xyr.y = SPEED_CORR;anti++;}
					
					if ((slowdown && ((find_front && find_back) || (find_right_f && find_right_b))) || continous > 100)
					{
						rand = 0;
						clear_all_irs();
						slowdown = 0;
						preslowdown = 0;
						continous = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						if (dir==UP) big_x--;
						else if (dir==LEFT) big_y--;
						else if (dir==DOWN) big_x++;
						else if (dir==RIGHT) big_y++;
						#if BIGDEBUG == 1
						printf("bigX:%d,bigY:%d\n",big_x,big_y);
						#endif
						
						//迷宫寻路
						char leftblock = !(HAL_GPIO_ReadPin(GPIOC,IR_LEFT_B));
						char rightblock = 0;
						char frontblock = !HAL_GPIO_ReadPin(GPIOC,IR_FRONT);
						char backblock = !HAL_GPIO_ReadPin(GPIOC,IR_BACK);
						char wallrefresh = (backblock << 3) + (leftblock << 2) + (frontblock << 1) + rightblock;
						
						uint8_t decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						if (decision == 255)
						{
							ClearWalls();
							decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						}
						state = decision;
						#if CVDEBUG==1
						state = states[statedebug];
						statedebug++;
						#endif
						//if (noslowdown) continous = 1;
						if (state == CV_LEFT) speed_xyr.x = -SPEED_LR;
						else if (state == CV_TURNLEFT || state == CV_TURNRIGHT || state == CV_TURNBACK) speed_xyr.x = 0;
						else {
							speed_xyr.x = SPEED_CORR;
							anti1 = 1;
						}
					}
				break;
				//右平移
				case CV_RIGHT:
					if (find_left_f && find_left_b && (!preslowdown))
					{
						preslowdown = 1;
						//clear_all_irs_except_r();
						clear_all_irs();
					}
					if (find_right_f && find_right_b && preslowdown && (!slowdown))
					{
						slowdown = 1;
						clear_all_irs_except_fb();
					}
					if ((!stopped) && -36 < info.cvxpos1 && info.cvxpos1 < 36 && -YAW_PERMIT < yaw_err && yaw_err < YAW_PERMIT) 
					{
						if (slowdown && (!noslowdown)) {speed_xyr.x = SPEED_LR_SLOW;if (rand == 2) {continous++;rand = 0;} else rand++;}
						else {speed_xyr.x = SPEED_LR;continous++;}
						speed_xyr.y = KP_X*info.cvxpos1;
						//speed_xyr.r = KP_R*info.cvangle1 + KD_YAW*yaw_diff;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					else
					{
						speed_xyr.x = 0;
						speed_xyr.y = KP_X1*info.cvxpos1;
						//speed_xyr.r = KP_R1*info.cvangle1;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					//speed_xyr.y = KP_X*info.cvxpos1;
					//speed_xyr.r = KP_R*info.cvangle1;
					if (anti > 0){speed_xyr.y = -SPEED_CORR;anti--;}
					else if (anti < 0){speed_xyr.y = SPEED_CORR;anti++;}
					
					if ((slowdown && ((find_front && find_back) || (find_left_f && find_left_b))) || continous > 100)
					{
						rand = 0;
						clear_all_irs();
						slowdown = 0;
						preslowdown = 0;
						continous = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						if (dir==UP) big_x++;
						else if (dir==LEFT) big_y++;
						else if (dir==DOWN) big_x--;
						else if (dir==RIGHT) big_y--;
						#if BIGDEBUG == 1
						printf("bigX:%d,bigY:%d\n",big_x,big_y);
						#endif
						
						//迷宫寻路
						char leftblock = 0;
						char rightblock = !(HAL_GPIO_ReadPin(GPIOB,IR_RIGHT_B));
						char frontblock = !HAL_GPIO_ReadPin(GPIOC,IR_FRONT);
						char backblock = !HAL_GPIO_ReadPin(GPIOC,IR_BACK);
						char wallrefresh = (backblock << 3) + (leftblock << 2) + (frontblock << 1) + rightblock;
						
						uint8_t decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						if (decision == 255)
						{
							ClearWalls();
							decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						}
						state = decision;
						#if CVDEBUG==1
						state = states[statedebug];
						statedebug++;
						#endif
						//if (noslowdown) continous = 1;
						if (state == CV_RIGHT) speed_xyr.x = SPEED_LR;
						else if (state == CV_TURNLEFT || state == CV_TURNRIGHT || state == CV_TURNBACK) speed_xyr.x = 0;
						else {
							speed_xyr.x = -SPEED_CORR;
							anti1 = -1;
						}
					}
				break;
				//倒车
				case CV_BACK:
					if (find_left_f && find_right_f && (!preslowdown))
					{
						preslowdown = 1;
						//clear_all_irs_except_b();
						clear_all_irs();
					}
					if (find_left_b && find_right_b && preslowdown && (!slowdown))
					{
						slowdown = 1;
						clear_all_irs_except_lr();
					}
					if ((!stopped) && -40 < info.cvxpos && info.cvxpos < 40 && -YAW_PERMIT < yaw_err && yaw_err < YAW_PERMIT) 
					{
						#if CVDEBUG == 1
						noslowdown = 0;
						#endif
						if (slowdown && (!noslowdown)) {speed_xyr.y = -SPEED_BACK_SLOW;if (rand == 2) {continous++;rand = 0;} else rand++;}
						else {speed_xyr.y = -SPEED_BACK;continous++;}
						speed_xyr.x = KP_X*info.cvxpos;
						//speed_xyr.r = KP_R*info.cvangle + KD_YAW*yaw_diff;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					else
					{
						speed_xyr.y = 0;
						speed_xyr.x = KP_X1*info.cvxpos;
						//speed_xyr.r = KP_R1*info.cvangle;
						speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					}
					//speed_xyr.x = KP_X*info.cvxpos;
					//speed_xyr.r = KP_R*info.cvangle;
					
					if ((slowdown && ((find_left && find_right) || (find_left_f && find_right_f))) || continous > 100)
					{
						clear_all_irs();
						rand = 0;
						continous = 0;
						slowdown = 0;
						preslowdown = 0;
						speed_xyr.x = 0;
						speed_xyr.y = 0;
						speed_xyr.r = 0;
						if (dir==UP) big_y--;
						else if (dir==LEFT) big_x++;
						else if (dir==DOWN) big_y++;
						else if (dir==RIGHT) big_x--;
						#if BIGDEBUG == 1
						printf("bigX:%d,bigY:%d\n",big_x,big_y);
						#endif
						
						//迷宫寻路
						char leftblock = !(HAL_GPIO_ReadPin(GPIOC,IR_LEFT_B));
						char rightblock = !(HAL_GPIO_ReadPin(GPIOB,IR_RIGHT_B));
						char frontblock = 0;
						char backblock = !HAL_GPIO_ReadPin(GPIOC,IR_BACK);
						char wallrefresh = (backblock << 3) + (leftblock << 2) + (frontblock << 1) + rightblock;
						
						uint8_t decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						if (decision == 255)
						{
							ClearWalls();
							decision = PersonFinding(dir,big_x + 1,big_y + 1,info.P1BX + 1,info.P1BY + 1,info.P2BX + 1,info.P2BY + 1,wallrefresh,&noslowdown);
						}
						state = decision;
						#if CVDEBUG==1
						state = states[statedebug];
						statedebug++;
						#endif
						if (state == CV_BACK) speed_xyr.y = -SPEED_BACK;
						else if (state == CV_TURNLEFT || state == CV_TURNRIGHT || state == CV_TURNBACK) speed_xyr.y = 0;
						else {
							speed_xyr.y = SPEED_CORR;
							anti = -3;
						}
					}
				break;
				//冲出迷宫的最后一步
				case BLIND_FORWARD:
					if (-40 < info.cvxpos && info.cvxpos < 40 && -YAW_PERMIT < yaw_err && yaw_err < YAW_PERMIT) {speed_xyr.y = SPEED_FORWARD;continous++;}
					else speed_xyr.y = 0;
					speed_xyr.x = KP_X*info.cvxpos;
					speed_xyr.r = KP_YAW*yaw_err + KD_YAW*yaw_diff;
					
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
		if(move_en != 1 || (HAL_GetTick() - newtime > 110))
		{
			speed_xyr.cal_speed = 1;
			speed_xyr.x = 0;
			speed_xyr.y = 0;
			speed_xyr.r = 0;
			stopped = 1;
		}
		else stopped = 0;
		if(speed_xyr.cal_speed)
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
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//timing debug
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
  huart3.Init.BaudRate = 57600;
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

  /*Configure GPIO pins : PC13 PC14 PC15 PC8 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC3 PC4 PC6 
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
  if (GPIO_Pin == ir_left) find_left = 1;
	else if (GPIO_Pin == ir_right) find_right = 1;
	else if (GPIO_Pin == ir_front) find_front = 1;
	else if (GPIO_Pin == ir_back) find_back = 1;
	else if (GPIO_Pin == ir_left_f) find_left_f = 1;
	else if (GPIO_Pin == ir_left_b) find_left_b = 1;
	else if (GPIO_Pin == ir_right_f) find_right_f = 1;
	else if (GPIO_Pin == ir_right_b) find_right_b = 1;
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
