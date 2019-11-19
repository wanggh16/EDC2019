#ifndef __MPU9250_H__
#define __MPU9250_H__

#define HIIC hi2c1
extern I2C_HandleTypeDef HIIC;

// 定义MPU9250内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)


//****************************

#define	GYRO_ADDRESS   0xD0	  //陀螺地址
#define MAG_ADDRESS    0x18   //磁场地址
#define ACCEL_ADDRESS  0xD0 
#define SlaveAddress 0xD0

typedef struct{
	int16_t accel_xout;
	int16_t accel_yout;
	int16_t accel_zout;
	
	int16_t gyro_xout;
	int16_t gyro_yout;
	int16_t gyro_zout;
	
	int16_t mag_xout;
	int16_t mag_yout;
	int16_t mag_zout;
}imutype;

void InitMPU6050(void);
//int16_t GetData(unsigned char REG_Address);
void Read_MPU9250_acc(imutype *imudata);
void Read_MPU9250_gyro(imutype *imudata);
void Read_MPU9250_mag(imutype *imudata);

/** 读数据函数****/

__STATIC_INLINE void I2C_ByteWrite(uint8_t dev_address,uint8_t REG_Address,uint8_t REG_data)
{
    HAL_I2C_Mem_Write(&HIIC,dev_address,REG_Address,I2C_MEMADD_SIZE_8BIT,&REG_data,1,10);
}

/*** 读数据函数 **/

__STATIC_INLINE void I2C_ByteRead(uint8_t dev_address,uint8_t REG_Address,uint8_t *dat,uint8_t len)
{
	while(HIIC.State != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Read(&HIIC,dev_address,REG_Address,I2C_MEMADD_SIZE_8BIT,dat,len,10);
}
  

#endif
