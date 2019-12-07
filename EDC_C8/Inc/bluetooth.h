#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__


typedef struct{
	uint8_t mixed0;
	uint8_t mixed1;
	uint8_t AXL;
	uint8_t AYL;
	uint8_t BXL;
	uint8_t BYL;
	uint8_t P1XL;
	uint8_t P1YL;
	uint8_t P2XL;
	uint8_t P2YL;
	uint8_t BALLXL;
	uint8_t BALLYL;
	
	int16_t yaw;
	
	uint8_t cvxf;
	uint8_t cvxb;
	uint8_t cvxl;
	uint8_t cvxr;
}gameinfo;

unsigned short CRC16(uint8_t *puchMsg, uint32_t usDataLen);
void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf);
void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf);

void usart_task(void);
void MY_NVIC_PriorityGroupConfig(uint8_t NVIC_Group);
void MY_NVIC_Init(uint8_t NVIC_PreemptionPriority,uint8_t NVIC_SubPriority,uint8_t NVIC_Channel,uint8_t NVIC_Group);
void usart2_init(uint32_t pclk2,uint32_t bound);
#endif
