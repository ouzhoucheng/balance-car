//i2c.h
#ifndef __IIC_H
#define __IIC_H
#include "stm32f1xx_hal.h"
 
#include <inttypes.h>

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

//void i2c_Start(void);
//void i2c_Stop(void);
//uint8_t i2c_SendByte(uint8_t _ucByte);
//uint8_t i2c_ReadByte(uint8_t ack);
//uint8_t i2c_WaitAck(void);
//void i2c_Ack(void);
//void i2c_NAck(void);
//uint8_t i2c_CheckDevice(uint8_t _Address);
//uint8_t i2c_write(unsigned char slave_addr, unsigned char reg_addr,\
//                  unsigned char length, unsigned char const *data);
//uint8_t i2c_read(unsigned char slave_addr, unsigned char reg_addr,\
//                 unsigned char length, unsigned char *data);
void _I2C_IsBusy(void);

//IIC起始信号
void I2C_Start(void);

//IIC终止信号
void I2C_Stop(void);

void I2C_SetACK(FunctionalState ackState);

FunctionalState I2C_GetACK(void);

//写入数据给从机,并返回ACK/NACK
FunctionalState I2C_WriteByte(uint8_t data);

//读取从机发送的数据，并决定是ACK/NACK
uint8_t I2C_ReadByte(FunctionalState ackState);

//往从机中的寄存器写入一字节的数据,参数:(从机地址，寄存器地址，希望传入的数据)
uint8_t I2C_WriteByteToSlave(uint8_t addr,uint8_t reg,uint8_t data);

//往从机中的寄存器写入多个数据，连续写入
uint8_t I2C_WriteSomeDataToSlave(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

uint8_t I2C_ReadFromSlave(uint8_t addr,uint8_t reg,uint8_t *buf);

uint8_t I2C_ReadSomeDataFromSlave(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

#endif /*__ IIC_H */
