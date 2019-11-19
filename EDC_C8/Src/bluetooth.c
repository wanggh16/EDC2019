#include "main.h"
#include "stm32f1xx_hal.h"
#include "bluetooth.h"

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;
uint8_t cv_cnt = 0;
uint8_t cvRxbuf[10]={0};

int wrong = 0;
extern gameinfo info;

void usart_task()
{
	/*state mechime : 
	0:free ,set dma
	1:wait to recieved start frame 
	2:start frame recieved,set dma for body
	3:waiting for body(dma)
	4:data recieved complete*/
	//cv uart 3
	switch(cv_cnt)
	{
		case 0:
			cvRxbuf[0] = 0;
			HAL_UART_Receive_DMA(&huart3,cvRxbuf,1);
			cv_cnt = 1;
		break;
		case 1:
			break;
		case 2:
			HAL_UART_Receive_DMA(&huart3,&cvRxbuf[1],5);
			cv_cnt = 3;
		break;
		case 3:
		break;
		case 4:
			if (cvRxbuf[1] == 0xfe)
			{
				info.cvstate = cvRxbuf[2];
				info.cvxpos = cvRxbuf[3];
				info.cvangle = cvRxbuf[4];
				info.cvypos = cvRxbuf[5];
			}
			cv_cnt = 0;
			break;
		default:
			cv_cnt = 0;		
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		switch(cv_cnt)
		{
			case 1:
			if(0xff == cvRxbuf[0])
				cv_cnt = 2;
			else
				cv_cnt = 0;
			break;
			case 3:
				cv_cnt = 4;
			break;
		}
	}
}


/* USER CODE BEGIN 1 */
void MY_NVIC_PriorityGroupConfig(uint8_t NVIC_Group)	 
{ 
	uint32_t temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //写入钥匙
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	    	  				   
}
void MY_NVIC_Init(uint8_t NVIC_PreemptionPriority,uint8_t NVIC_SubPriority,uint8_t NVIC_Channel,uint8_t NVIC_Group)	 
{ 
	uint32_t temp;	
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;//取低四位  
	NVIC->ISER[NVIC_Channel/32]|=(1<<NVIC_Channel%32);//使能中断位(要清除的话,相反操作就OK) 
	NVIC->IP[NVIC_Channel]|=temp<<4;//设置响应优先级和抢断优先级   	    	  				   
}
void usart2_init(uint32_t pclk2,uint32_t bound)
{  	 
	float temp;
	uint16_t mantissa;
	uint16_t fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	GPIOA->CRL&=0XFFFF00FF;//IO状态设置
	GPIOA->CRL|=0X00008B00;//IO状态设置
		  
	RCC->APB1RSTR|=1<<17;   //复位串口1
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; //波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
  USART2->CR1|=1<<8;    //PE中断使能
  USART2->CR1|=1<<5;    //接收缓冲区非空中断使能 
	USART2->CR1|=1<<4;    //IDLE中断使能	
  MY_NVIC_Init(0,0,USART2_IRQn,2);//组2 高优先级
}

//串口1中断服务程序      
uint8_t USART_RX_BUF[50];    //接收缓冲,最大100个字节.
uint8_t USART_RX_CNT=0;       //接收到的有效字节数目      

void USART2_IRQHandler(void)
{
    uint8_t clear=clear;
		if(USART2->SR&(1<<4))//IDLE中断
		{
			clear=USART2->SR;
			clear=USART2->DR;
			USART_RX_BUF[USART_RX_CNT]=0;
			uint16_t crc;
			crc = CRC16(USART_RX_BUF, 28);
			if (0 == USART_RX_BUF[24] && 0 == USART_RX_BUF[25] && 0 == USART_RX_BUF[26] && 0 == USART_RX_BUF[27] && crc == (((int16_t)USART_RX_BUF[28])<<8) + (int16_t)USART_RX_BUF[29] && 0x0d == USART_RX_BUF[30] && 0x0a == USART_RX_BUF[31])
			{
				info.mixed0 = USART_RX_BUF[2];
				info.mixed1 = USART_RX_BUF[3];
				info.AXL = USART_RX_BUF[4];
				info.AYL = USART_RX_BUF[5];
				info.BXL = USART_RX_BUF[6];
				info.BYL = USART_RX_BUF[7];
				info.P1XL = USART_RX_BUF[8];
				info.P1YL = USART_RX_BUF[9];
				info.P2XL = USART_RX_BUF[10];
				info.P2YL = USART_RX_BUF[11];
				info.BALLXL = USART_RX_BUF[12];
				info.BALLYL = USART_RX_BUF[13];
				wrong = 0;
			}
			else wrong++;
			if (wrong > 0)
			{
				info.AXL = 0;
				info.AYL = 0;
				info.BXL = 0;
				info.BYL = 0;
			}
			USART_RX_CNT = 0;
		}
		else if(USART2->SR&(1<<5))//接收到数据
    {
			USART_RX_BUF[USART_RX_CNT] = USART2->DR;
			USART_RX_CNT++;			
    }
} 


uint16_t CRC16(uint8_t *puchMsg, uint32_t usDataLen)
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wChar = 0;
  
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(int i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}

void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf)
{
		int i;
		uint8_t tmp[4];
		tmp[0] = 0;
		for(i=0;i< 8;i++)
		{
			if(srcBuf[0]& (1 << i))
				tmp[0]|=1<<(7-i);
		}
		dBuf[0] = tmp[0];
}

void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf)
{
		int i;
		uint16_t tmp[4];
		tmp[0] = 0;
		for(i=0;i< 16;i++)
		{
			if(srcBuf[0]& (1 << i))
				tmp[0]|=1<<(15 - i);
		}
		dBuf[0] = tmp[0];
}

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
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
/* USER CODE END 1 */
