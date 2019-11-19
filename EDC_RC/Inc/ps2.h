#ifndef __PS2_H__
#define __PS2_H__

#define ENBANK GPIOC
#define ENPORT LL_GPIO_PIN_2
#define MT_CTRL common_mt_ctrl
#define MT_MTS motors
#define SPIPORT hspi1
#define SPEED_XYR speed_xyr
#define CAL_SPEED cal_speed
#define BUFFSIZE 9
#define SPIBANK GPIOA
#define SCK LL_GPIO_PIN_5
#define CMD LL_GPIO_PIN_7
#define DAT GPIO_PIN_6
#define JOY_TH 10

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
 
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16
 
//#define WHAMMY_BAR		8
 
//These are stick values
#define PSS_RX 5                //ÓÒÒ¡¸ËXÖáÊý¾Ý
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

#define CHASISS 0  //Âó¿ËÄÉÄ·ÂÖµ×ÅÌ
#define ROBOT_ARM 1  //»úÐµ±Û

void PS2_ReadData(void);
void Button_Servo(uint8_t button1,uint8_t button2,uint16_t *servo,uint8_t rate);
void Ps2_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed);

#endif
