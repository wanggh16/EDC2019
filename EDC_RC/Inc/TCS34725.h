#ifndef __TCS34725_H__
#define __TCS34725_H__

#define HIIC hi2c1
#define TCSADDR 0X29<<1
#define TCS_CMD_BIT 0x80
#define TCS_WORD_MODE 0X20

#define TCS_EN 0X00
#define TCS_ATIME 0X01
#define TCS_CTRL 0X0F
#define TCS_ID 0X12
#define TCS_STATUS 0X13
#define TCS_DAT 0X14

#define TCS_AEN 0X02
#define TCS_PON 0X01
#define TCS_24MS 0XF6
#define TCS_1GAIN 0X00
#define TCS_4GAIN 0X01
#define TCS_16GAIN 0X10
#define TCS_60GAIN 0X11

typedef struct{
	uint16_t cdata;
	uint16_t rdata;
	uint16_t gdata;
	uint16_t bdata;
	uint16_t color_h;
	uint16_t color_v;
	float color_s;
}colortype;

void TCS_task(colortype *clr);
void TCS_Init(void);
void TCS_Getcolor(colortype *clr);


__STATIC_INLINE void tcs_Read_reg(I2C_HandleTypeDef *hi2c,uint8_t reg,uint8_t *dat,uint8_t len)
{
	while(hi2c->State != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Read(hi2c,TCSADDR,reg|TCS_CMD_BIT,I2C_MEMADD_SIZE_8BIT,dat,len,10);
}

__STATIC_INLINE HAL_StatusTypeDef tcs_Write_reg(I2C_HandleTypeDef *hi2c,uint8_t reg,uint8_t cmd)
{
	return HAL_I2C_Mem_Write(hi2c,TCSADDR,reg|TCS_CMD_BIT,I2C_MEMADD_SIZE_8BIT,&cmd,1,10);
}

#endif
