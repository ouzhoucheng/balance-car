## cubeMX+MDK制作蓝牙平衡车
# 前言

有很多单片机初学者都会选择把平衡小车作为第一件作品，但是制作过程中会碰到许多疑问，笔者也在学习过程中完成了一台摇摇晃晃的平衡小车，希望以自己的方式，向读者展示一台平衡小车诞生的全流程。

文末获取工程文件及其他资料，建议结合keil工程阅读本文。

## 效果

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210210232358449.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NjE0MzE1Mg==,size_16,color_FFFFFF,t_70#pic_center)
[video(video-w7TjWQih-1612971289599)(type-bilibili)(url-https://player.bilibili.com/player.html?aid=671699548)(image-https://ss.csdn.net/p?http://i0.hdslb.com/bfs/archive/8bb5fa088fce1ce4bb160b5d374812355e5210e9.jpg)(title-蓝牙平衡车)]


# 组成

![组成框图](https://img-blog.csdnimg.cn/img_convert/16424efc460469f08482a51b5b0c27d4.png){#fig:组成框图}

## 底盘

![小车底盘](https://img-blog.csdnimg.cn/img_convert/561ba43a57734d290cf642512d15eeb2.png){#fig:小车底盘}

如图，有两个带编码器的电机，轮子和支架。

## 主控

![stm32ZET6核心板](https://img-blog.csdnimg.cn/img_convert/110463c2fc644aec4c577b2114e509fc.png){#fig:stm32f103ZET6}

附件提供的程序对应的是stm32f103ZET6，在野火买的核心板，这里有些大材小用，其实stm32c8t6最小系统板是完全足够的。\

![stm32C8T6核心板](https://img-blog.csdnimg.cn/img_convert/a92e59430776a7d9de6dc5d4126a5987.png){#fig:stm32f103C8T6}

## 电机驱动

![电机驱动模块](https://img-blog.csdnimg.cn/img_convert/f12de88b55ee35c09cd75e832b2b52a5.png){#fig:TB6612FNG}

电机模块用的是TB6612FNG。\

## 姿态传感器

![姿态传感器MPU6050](https://img-blog.csdnimg.cn/img_convert/b36b49b8bcdd61e4e7e6978775c004d3.png){#fig:MPU6050}

## 电源
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021021023260379.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NjE0MzE1Mg==,size_16,color_FFFFFF,t_70#pic_center)


一个电池盒，装上两节18650电池可以输出7.4V的电压，接上电源模块MB-102，可以得到5V和3.3V，为整个系统供电。

## 结构等等

M3螺丝螺母铜柱、杜邦线若干。\
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210210232617976.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NjE0MzE1Mg==,size_16,color_FFFFFF,t_70#pic_center)


# 直立

首先把车扶稳，开电源，mpu6050初始化并确定平衡位置，车往哪边倒，mpu6050就给芯片发对应数据，芯片控制电机就往哪边追，这就颤颤巍巍地立起来了。

通过mpu6050，可以获取小车当前的姿态角，通过iic来读取模块内寄存器的数据，可以测到三轴加速度和三周角速度，我们需要的是Pitch角或Roll角（由mpu摆放位置决定），直接读取出来的是六个数据，还需要复杂的数学计算才能得到三个角度，一种便捷的方法是移植官方的DMP库，让mpu自己处理成角度让我们读取。具体怎样移植，在这个教程里有详细介绍。

[DMP库移植](https://www.moore8.com/courses/1385)\
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210210232633565.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NjE0MzE1Mg==,size_16,color_FFFFFF,t_70#pic_center)

那获取到了角度，就需要PID算法，平衡小车的PID分成三部，直立PD环、速度PI环、转向PD环。

## 直立环

要立5秒，需要第一个环，直立环需要mpu的角度数据，需要一个脚接收中断、两个脚iic通信、还要开两个定时器输出PWM控制电机（电机的逻辑控制还需要四个脚，还要电机驱动的总开关STBY）。

stm32的高级定时器有PWM输出模式，我们在cubeMX里打开这个功能。

![定时器配置](https://img-blog.csdnimg.cn/img_convert/690d83c2e1242c76ba7bd255b5489a5d.png){#fig:配置定时器PWM模式}

![定时器配置](https://img-blog.csdnimg.cn/img_convert/7d92266cb819b83b4f8bdd8882bd6c6f.png){#fig:配置定时器PWM模式}

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//直立环
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
    float Bias;
    int balance;
    Bias = Angle-Mechanical_balance;
    balance = balance_UP_KP*Bias+balance_UP_KD*Gyro;
    return balance;
}
```

## 速度环

要立得稳怎么推都不倒，就需要前两个环，而速度环还需要电机的速度数据，那就需要用32两个定时器进行解码。

这里再启用两个定时器的解码模式。

![定时器配置](https://img-blog.csdnimg.cn/img_convert/ceec4c46dcb4a3b862f9a15714ae1ff1.png){#fig:配置定时器解码器模式}

![定时器配置](https://img-blog.csdnimg.cn/img_convert/e3ca68e1f0529ec6c5e03b8fda64edc6.png){#fig:配置定时器解码器模式}

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//速度环
int velocity(int encoder_left,int encoder_right)
{  //定义一些变量
    static float Velocity,Encoder_Least,Encoder,Movement = 0;
    static float Encoder_Integral;  
    //然后就是PID的一些操作
    Encoder_Least =(Encoder_Left+Encoder_Right)-0;
    Encoder *= 0.8;
    Encoder += Encoder_Least*0.2;
    Encoder_Integral +=Encoder;
    Encoder_Integral = Encoder_Integral;
    //接下来限制一下速度，这里是因为我的车用了很多螺丝、
    //长尾夹之类的比较重，冲太快容易倒，限到7000几乎就不会倒了
    if(Encoder_Integral>7000)   
    Encoder_Integral=7000;             //积分限幅
    if(Encoder_Integral<-7000)      
    Encoder_Integral=-7000;            //积分限幅
    //得到速度环的值
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;
    //为了安全，万一车倒了车轮就不要再转了，
    //所以要清空积分，配合下面检测有没有倒的函数
    if(roll<-45 || roll>45)             
    Encoder_Integral=0;         //清空积分
    
    return Velocity;
}
```

这样，在前面HAL、Tim、GPIO、IIC、MPU各种初始化后，mpu开始工作，100Hz的频率采集一次角度（这里采集的是Roll角），并发出中断，stm32就进入了中断回调函数。

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   //获取成功的信号，然后获取Roll角度，和一个角速度
    //（这里全拿了，显然代码不是自己写的。。）
    gyro_data_ready_cb();
    dmp_getdata();
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    //接着Encoder读取了两个计时器的计数值，原本是累加值，
    //但是每读取一次就归零变成求了一次导，这得到的就是跟速度线性相关的值了
    Encoder_Left = (short)TIM2 -> CNT;TIM2 -> CNT = 0;
    Encoder_Right = -((short)TIM4 -> CNT);TIM4 -> CNT = 0;
    //再把获取的速度放进直立环和速度环函数
    Balance_Pwm = balance_UP(roll, Mechanical_angle, gyrox);
    Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);
    //再再运算得出两个电机的PWM占空比
    moto1 = Balance_Pwm - Velocity_Pwm;
    moto2 = Balance_Pwm - Velocity_Pwm;
    //然后限幅、检测车有没有倒，就可以更新PWM值了
    Xianfu_Pwm();
    Turn_Off(roll);
    Set_Pwm(moto1, moto2);
}
//限幅
void Xianfu_Pwm(void)
{
    if(moto1<-7000 ) moto1=-7000 ;
    if(moto1>7000 )  moto1=7000 ;
    if(moto2<-7000 ) moto2=-7000 ;
    if(moto2>7000 )  moto2=7000 ;
}
//摔倒保护
void Turn_Off(float angle)
{
    if(angle<-45||angle>45){
        moto1=0;
        moto2=0;
    }       
}
```

到此，小车就可以自己站起来啦

# 蓝牙

既然能自己站，下一步就要跑一跑了，需要加一个蓝牙模块，这里用了HC-05。

![加上蓝牙的系统框图](https://img-blog.csdnimg.cn/img_convert/d96a050ad1890ffd8a57756c11fd05ae.png){#fig:整体框图}

## 蓝牙控制逻辑

这里启用了串口2+中断，这样，手机这边按下方向键发送一个记号1234，单片机接收到进入串口中断，判断前后左右走，手机这边松开键，就发一个0，单片机接收到再进入串口中断，恢复到直立状态。

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    switch(usart2_Re)
    {
        case 0:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
        HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 1);
        break;
        case 1:
        Flag_forward = 1, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
        HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
        break;
        case 2:
        Flag_forward = 0, Flag_retreat = 1, Flag_left = 0, Flag_right = 0;
        HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
        break;
        case 3:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 1, Flag_right = 0;
        HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
        break;
        case 4:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 1;
        HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
        break;
        default:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
        Yaw_FPV_L = 0, Yaw_FPV_R = 0, Pitch_FPV_U = 0, Pitch_FPV_D = 0;
        break;
    }
    HAL_UART_Receive_IT(&huart2, &usart2_Re, 1);
}
```

## 制作蓝牙app

至于手机的上位机软件，用appinventor可以快速制作，参见教程 [app
inventor教程](https://blog.csdn.net/qq_40515692/article/details/80894253)\

![界面布局](https://img-blog.csdnimg.cn/img_convert/f6313c4acbbe8eff1d4619f5c7337cff.png){#fig:app布局}

![app启动逻辑](https://img-blog.csdnimg.cn/img_convert/fd7efe370af8ce5af17dedc8f0c2ad1e.png){#fig:app启动逻辑}

![蓝牙控制指令逻辑](https://img-blog.csdnimg.cn/img_convert/554c50e6f3506c74d514d9c81bb40c35.png){#fig:app控制逻辑}

这里提供制作好的app（见工程apk文件夹），可以先用蓝牙模块和串口助手尝试。

![手机上打开应用](https://img-blog.csdnimg.cn/img_convert/8bf6dbb6dd365f5007810fddfc8dd066.png){#fig:手机app}

可以在app inventor导入附件中的aia文件，获取这个工程。

控制前后，我们再给两个标志位，还要再前面的速度环加一些东西。

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//修改速度环
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement = 0;
    static float Encoder_Integral;
    if(1==Flag_forward){
        Movement=-180;
    }
    else if(1==Flag_retreat){
        Movement=90;
    }//这里前后的Movement不一样是因为我的车后面更重，后退的猛就容易倒
    else{   
        Movement=0;
    }
    //=============speed PI controller==================//  
    Encoder_Least =(Encoder_Left+Encoder_Right)-0;
    Encoder *= 0.8;
    Encoder += Encoder_Least*0.2;
    Encoder_Integral +=Encoder; 
    Encoder_Integral=Encoder_Integral-Movement; 
```

## 转向环

前进后退就有了，我们再加一个转向PD环，这种算法控制下的平衡车并不能原地转，而是需要一个速度，在已有速度基础上让两个车轮产生速度差，才能实现转向。

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//转向环
int turn(int encoder_left,int encoder_right,float gyro)//×ªÏò¿ØÖÆ
{
    static float Turn_Target,Turn,Encoder_temp,Turn_Convert=10,Turn_Count;
    float Turn_Amplitude=100,Kp=16,Kd=0;        
    if(1==Flag_left)               
    Turn_Target+=Turn_Convert;
    else if(1==Flag_right)       
    Turn_Target-=Turn_Convert; 
    else 
    Turn_Target=0;
    
    if(Turn_Target>Turn_Amplitude)  
    Turn_Target=Turn_Amplitude;
    if(Turn_Target<-Turn_Amplitude) 
    Turn_Target=-Turn_Amplitude;
    
    if(Flag_forward==1||Flag_retreat==1)  
    Kd=4;        
    else 
    Kd=0;
    //=============turing PD controller==================//
    Turn=-Turn_Target*Kp+gyro*Kd;
    return Turn;
}
```

同时原来的中断函数也要修改。

```{.[ANSI]C language="[ANSI]C" title="control.c"}
//中断回调的部分
Balance_Pwm = balance_UP(roll, Mechanical_angle, gyrox);
Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);

if(1==Flag_left||1==Flag_right)    
Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz);
else 
Turn_Pwm=0.5*gyroz;

moto1 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
moto2 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
```

到此，能跑的蓝牙平衡车就做好啦。

但这样做出来的车还是非常臃肿不可靠，如果有能力的话，设计PCB板，把模块都集中在一个板上是最佳的方法，还可以加入旋转编码器、摇杆、小屏幕等方便调节参数。

# 下载链接

```{.[ANSI]C language="[ANSI]C" title="工程文件、安装包和其他资料"}
链接：https://pan.baidu.com/s/1REZpL91b-o_YxCZ-x2xm1Q 
提取码：tEGO 
复制这段内容后打开百度网盘手机App，操作更方便哦
