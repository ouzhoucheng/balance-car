#include "mpu6050.h"
#include "iic.h"

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	uint8_t buf[6],res;  
	res=I2C_ReadSomeDataFromSlave(MPU6050_SLAVE_ADDRESS,MPU6050_GYRO_OUT,6,buf);
//	if(res==0)
//	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
//	} 	
	return res;;
}
