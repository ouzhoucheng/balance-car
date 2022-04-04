#include "iic.h"
#include "main.h"
 
/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define GPIO_PORT_I2C	GPIOB			/* GPIO端口 */
#define I2C_SCL_PIN		SCL_Pin			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		SDA_Pin			/* 连接到SDA数据线的GPIO */

		 //SDA-PB7


/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
#if 0	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
	#define I2C_SCL_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
	
	#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* 读SDA口线状态 */
#elif 0	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#elif 1
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define I2C_SCL_HIGH  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_LOW  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_HIGH  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_LOW  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#endif



/*
*********************************************************************************************************
*	函 数 名: i2c_Delay
*	功能说明: I2C总线位延迟，最快400KHz
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	for (i = 0; i < 10; i++);
}

void _I2C_IsBusy(void)
{
    while(!I2C_SDA_READ)
    {
        //读取SDA线上得电平状态，若为低电平则说明总线被控制,
        I2C_SCL_LOW;
        i2c_Delay();i2c_Delay();i2c_Delay();
        I2C_SCL_HIGH;
        i2c_Delay();i2c_Delay();
			
    }
}

//IIC起始信号
void I2C_Start(void)
{
    _I2C_IsBusy();//检查总线状态可有可无，增加健壮性
//printf("111\n");
     I2C_SCL_LOW;//不能来SCL为高的时候切换电平
     i2c_Delay();

      I2C_SDA_HIGH;
     i2c_Delay();

      I2C_SCL_HIGH;
     i2c_Delay();

      I2C_SDA_LOW;
     i2c_Delay();
			
      I2C_SCL_LOW;     //将SCL拉低，钳住SCL线，准备发送地址数据；
     i2c_Delay();
}

//IIC终止信号
void I2C_Stop(void)
{
       I2C_SCL_LOW;//不能来SCL为高的时候切换电平
       i2c_Delay();

      I2C_SDA_LOW;
      i2c_Delay();

       I2C_SCL_HIGH;
       i2c_Delay();

      I2C_SDA_HIGH;
      i2c_Delay();
}

void I2C_SetACK(FunctionalState ackState)      //发送应答信号
{
     I2C_SCL_LOW;
     i2c_Delay();


     if(ackState == ENABLE)
     {
          I2C_SDA_LOW;
          i2c_Delay();
     }
     else
     {
         I2C_SDA_HIGH;
         i2c_Delay();
     }

     I2C_SCL_HIGH;
     i2c_Delay();
     //释放总线
     I2C_SCL_LOW;
     i2c_Delay();

     I2C_SDA_HIGH;
     i2c_Delay();
}

FunctionalState I2C_GetACK(void)//获取应答信号
{   
    FunctionalState ask;

    I2C_SCL_HIGH;
    i2c_Delay();

    if(I2C_SDA_READ)
    {
        ask = DISABLE;
    }
    else
    {
        ask = ENABLE;
    }

    I2C_SCL_LOW;
    i2c_Delay();

    return ask;
}
//写入数据给从机,并返回ACK/NACK
FunctionalState I2C_WriteByte(uint8_t data)
{
    //MSB 高位先行;
    //LSB 低位先行;
    uint8_t i;
    for(i=0;i<8;i++)
    {
        I2C_SCL_LOW;
        i2c_Delay();

        if(data & 0x80)
        {
            I2C_SDA_HIGH;
            i2c_Delay();
        }
        else
        {
            I2C_SDA_LOW;
            i2c_Delay();
        }

        data <<= 1;//数据左移
        i2c_Delay();

        I2C_SCL_HIGH;
        i2c_Delay();
    }
    //主机释放SDA线,使得总线空闲，以便MPU6050能够发出响应信息，并钳住SCL线
    //释放SDA线就是讲SDA置高
    I2C_SCL_LOW;
    i2c_Delay();

    I2C_SDA_HIGH;
    i2c_Delay();

    return I2C_GetACK();
}

//读取从机发送的数据，并决定是ACK/NACK
uint8_t I2C_ReadByte(FunctionalState ackState)
{
    uint8_t i;
    uint8_t data = 0x00;

    for(i=0;i<8;i++)
    {
        I2C_SCL_HIGH;
        i2c_Delay();

        data <<= 1;
        if(I2C_SDA_READ)
        {
            data|=0x01;
        }

        I2C_SCL_LOW;
        i2c_Delay();
    }

    I2C_SetACK(ackState);
    return data;
}

//往从机中的寄存器写入一字节的数据,参数:(从机地址，寄存器地址，希望传入的数据)
uint8_t I2C_WriteByteToSlave(uint8_t addr,uint8_t reg,uint8_t data)
{
    FunctionalState state;

    I2C_Start();

    state = I2C_WriteByte(addr<<1|0);

    if(state == ENABLE)
    {
        state = I2C_WriteByte(reg);
        if(state)
        {
            state = I2C_WriteByte(data);
            //正常写入，产生停止信号，并返回0
            I2C_Stop();
            return 0;
        }
    }
    //写入异常，产生停止信号，并返回1
    I2C_Stop();
    return 1;
}

//往从机中的寄存器写入多个数据，连续写入
uint8_t I2C_WriteSomeDataToSlave(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    FunctionalState state;  //记录ACK/NACK
    uint8_t i;

    I2C_Start();
    state = I2C_WriteByte(addr<<1|0);
    if(state == ENABLE)
    {
        state = I2C_WriteByte(reg);
        if(state == ENABLE)
        {
            for(i=0;i<len;i++)
            {
                state = I2C_WriteByte(*(buf+i));
                if(state == DISABLE)
                {
                    //从机未应答，产生停止信号结束数据传输
                    I2C_Stop();
                    return 1;
                }
            }
        }
    }
    //写入异常，产生停止信号，并返回1
    I2C_Stop();
    return 0;
}

uint8_t I2C_ReadFromSlave(uint8_t addr,uint8_t reg,uint8_t *buf)
{
    FunctionalState state;
    I2C_Start(); //发送起始信号
    state = I2C_WriteByte(addr<<1|0); //发送从机地址和写方向
    if(state == ENABLE)
    {
        state = I2C_WriteByte(reg);//发送读取数据的寄存器
        if(state == ENABLE)
        {
            I2C_Start(); //发送起始数据
            state = I2C_WriteByte(addr<<1|1); //写从机地址和读方向
            if(state == ENABLE)
                *buf = I2C_ReadByte(DISABLE);//读完一个数据后向从机发NACK

            //数据正常读取，产生停止信号，返回0
            I2C_Stop();
            return 0;
        }
    }
    I2C_Stop();
    return 1;
}

uint8_t I2C_ReadSomeDataFromSlave(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    FunctionalState state;

    I2C_Start(); //发送起始信号
    state = I2C_WriteByte(addr<<1|0); //发送从机地址和写方向

    if(state == ENABLE)
    {
        state = I2C_WriteByte(reg);//发送读取数据的寄存器
        if(state == ENABLE)
        {
            I2C_Start();
            state = I2C_WriteByte(addr<<1|1); //发送从机地址和写方向
            if(state == ENABLE)
            {
                while(len)
                {
                    
                    if(len==1)
                            *buf = I2C_ReadByte(DISABLE);//接收最后一位数据的时候给从机返回一个NACK
                    else
                            *buf = I2C_ReadByte(ENABLE);//接收数据时，给从机返回一个ACK
                    len--;
                    buf++;
                }
                //数据正常读取，产生停止信号，返回0
                I2C_Stop();
                return 0;
            }
        }
    }
    I2C_Stop();
    return 1;
}

