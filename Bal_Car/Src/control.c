#include "control.h"
#include "mpu_dmp_useapi.h"
#include <stdio.h>
#include "mpu6050.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

float Mechanical_angle=0; 

float balance_UP_KP=700; 	 //700 PD parameters of car upright ring
float balance_UP_KD=-2.3;      //-2.3
float velocity_KP=200;       //200 PI parameters of car speed
float velocity_KI=1;      //1

extern float pitch, roll, yaw;
short gyrox,gyroy,gyroz;
int moto1=0, moto2=0;

int Encoder_Left, Encoder_Right, Direction_Left, Direction_Right;  //编码器计数
int Flag_forward = 0, Flag_retreat = 0;
int Flag_left = 0, Flag_right = 0;

//直立环PD
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
    float Bias;
    int balance;
    Bias=Angle-Mechanical_balance;    				 //find out the middle angle value
    balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //count motor PWM value
    return balance;
}

//速度环PI

int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement = 0;
    static float Encoder_Integral;
    if(1==Flag_forward)				
    {
        Movement=-180;
    }
    else if(1==Flag_retreat)//½ÓÊÕµ½À¶ÑÀAPPÒ£¿ØÊý¾Ý		
    {
        Movement=90;//Éè¶¨ËÙ¶È
    }
    else//Ã»ÓÐ½ÓÊÜµ½ÈÎºÎµÄÊý¾Ý
    {	
        Movement=0;
    }
    //=============speed PI controller=======================//	
    Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //get the newest speed defference
    Encoder *= 0.8;		                                              //1st order low pass filter     
    Encoder += Encoder_Least*0.2;	                                  //1st order low pass filter
    Encoder_Integral +=Encoder;                                       //intergral displacement t=10ms
    Encoder_Integral=Encoder_Integral-Movement;                       //receive remote control data
    if(Encoder_Integral>7000)  	
        Encoder_Integral=7000;             //intergral limit
    if(Encoder_Integral<-7000)		
        Encoder_Integral=-7000;            //intergral limit
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //speed control
    if(roll<-40||roll>40) 			
        Encoder_Integral=0;     		  //clean intergral if motor close
    return Velocity;
}

//绝对值
int myabs(int a)
{ 		   
    int temp;
    if(a<0)  temp=-a;  
    else temp=a;
    return temp;
}

//限幅
void Xianfu_Pwm(void)
{
	 //===PWMÂú·ùÊÇ7200 ÏÞÖÆÔÚ7000
    if(moto1<-7000 ) moto1=-7000 ;
    if(moto1>7000 )  moto1=7000 ;
    if(moto2<-7000 ) moto2=-7000 ;
    if(moto2>7000 )  moto2=7000 ;
}


//幅值PWM
void Set_Pwm(int moto1,int moto2)
{
    if(moto1>0){
        HAL_GPIO_WritePin(MOTOR_L1_GPIO_Port,MOTOR_L1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L2_GPIO_Port,MOTOR_L2_Pin,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(MOTOR_L1_GPIO_Port,MOTOR_L1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L2_GPIO_Port,MOTOR_L2_Pin,GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,myabs(moto1));
//    printf("\r\n%d\t",moto1);
    if(moto2<0){
        HAL_GPIO_WritePin(MOTOR_R1_GPIO_Port,MOTOR_R1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R2_GPIO_Port,MOTOR_R2_Pin,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(MOTOR_R1_GPIO_Port,MOTOR_R1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R2_GPIO_Port,MOTOR_R2_Pin,GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,myabs(moto2));        
//    printf("\t%d\r\n",moto2);
}

//close motor while abnormal
void Turn_Off(float angle)
{
    if(angle<-45||angle>45)	 //close motor while voltage < 11.1V
    {	                                   //close motor while angle > 40																	 
        moto1=0;
        moto2=0;
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
    }		
}

//get speed from encoder
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
    {
        case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
        case 5:  Encoder_TIM= (short)TIM5 -> CNT;  TIM5 -> CNT=0;break;	
        default: Encoder_TIM=0;
    }
    return Encoder_TIM;
}

//usart IT control by BlueTeeth
extern uint8_t usart2_Re;
int Yaw_FPV_L = 0, Yaw_FPV_R = 0, Pitch_FPV_U = 0, Pitch_FPV_D = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    switch(usart2_Re)
    {
        case 0:
            Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
            HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 1);
//            printf("\r\nSTOP\r\n");
        break;
        case 1:
            Flag_forward = 1, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
            HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
//            printf("\r\nFORWARD\r\n");
        break;
        case 2:
            Flag_forward = 0, Flag_retreat = 1, Flag_left = 0, Flag_right = 0;
            HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
//            printf("\r\nRETREAT\r\n");
        break;
        case 3:
            Flag_forward = 0, Flag_retreat = 0, Flag_left = 1, Flag_right = 0;
            HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
//            printf("\r\nLEFT\r\n");
        break;
        case 4:
            Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 1;
            HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, 0);
//            printf("\r\nRIGHT\r\n");
        break;
        
        case 9:
            Yaw_FPV_L = 0, Yaw_FPV_R = 0, Pitch_FPV_U = 0, Pitch_FPV_D = 0;
            HAL_GPIO_WritePin(LED_BT_FPV_GPIO_Port, LED_BT_FPV_Pin, 1);
//            printf("\r\nFPV Stay\r\n");
        break;
        case 5:
            Yaw_FPV_L = 0, Yaw_FPV_R = 0, Pitch_FPV_U = 1, Pitch_FPV_D = 0;
            HAL_GPIO_WritePin(LED_BT_FPV_GPIO_Port, LED_BT_FPV_Pin, 0);
//            printf("\r\nFPV Up\r\n");
        break;
        case 6:
            Yaw_FPV_L = 0, Yaw_FPV_R = 0, Pitch_FPV_U = 0, Pitch_FPV_D = 1;
            HAL_GPIO_WritePin(LED_BT_FPV_GPIO_Port, LED_BT_FPV_Pin, 0);
//            printf("\r\nFPV Down\r\n");
        break;
        case 7:
            Yaw_FPV_L = 1, Yaw_FPV_R = 0, Pitch_FPV_U = 0, Pitch_FPV_D = 0;
            HAL_GPIO_WritePin(LED_BT_FPV_GPIO_Port, LED_BT_FPV_Pin, 0);
//            printf("\r\nFPV Left\r\n");
        break;
        case 8:
            Yaw_FPV_L = 0, Yaw_FPV_R = 1, Pitch_FPV_U = 0, Pitch_FPV_D = 0;
            HAL_GPIO_WritePin(LED_BT_FPV_GPIO_Port, LED_BT_FPV_Pin, 0);
            printf("\r\nFPV Right\r\n");
        break;
        
        default:
            Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
            Yaw_FPV_L = 0, Yaw_FPV_R = 0, Pitch_FPV_U = 0, Pitch_FPV_D = 0;
//            printf("\r\nSTOP\r\n");
        break;
    }
    HAL_UART_Receive_IT(&huart2, &usart2_Re, 1);
}

int turn(int encoder_left,int encoder_right,float gyro)//×ªÏò¿ØÖÆ
{
    static float Turn_Target,Turn,Encoder_temp,Turn_Convert=10,Turn_Count;
    float Turn_Amplitude=100,Kp=16,Kd=0;     
//=============remote for L or R=======================//
//adjust the starting speed according to the speed before rotation to increase adaptability
//    if(1==Flag_left||1==Flag_right)                      
//    {
//        if(++Turn_Count==1)
//        Encoder_temp=myabs(encoder_left+encoder_right);      
//        Turn_Convert=55/Encoder_temp;
//        if(Turn_Convert<0.6)Turn_Convert=0.6;
//        if(Turn_Convert>3)Turn_Convert=3;
//    }	
//    else
//    {
//        Turn_Convert=50;
//        Turn_Count=0;
//        Encoder_temp=0;
//    }			
    if(1==Flag_left)	           
        Turn_Target+=Turn_Convert;
    else if(1==Flag_right)	     
        Turn_Target-=Turn_Convert; 
    else 
        Turn_Target=0;
//limit turn speed
    if(Turn_Target>Turn_Amplitude)  
        Turn_Target=Turn_Amplitude;
    if(Turn_Target<-Turn_Amplitude) 
        Turn_Target=-Turn_Amplitude;
//cancel gyro correction when turning, fuzzy PID
    if(Flag_forward==1||Flag_retreat==1)  
        Kd=4;        
    else 
        Kd=0;
//=============turing PD controller=======================//
    Turn=-Turn_Target*Kp+gyro*Kd;                 //===½áºÏZÖáÍÓÂÝÒÇ½øÐÐPD¿ØÖÆ
    return Turn;
//    float Turn,Kp=20,Bias,Turn_Amplitude=0;
//	if(1==Flag_left) Turn_Amplitude=1100;
//	else if(1==Flag_right) Turn_Amplitude=-1100;
//	else Turn_Amplitude=0;
//	Bias=gyro-0;
//	Turn=-Bias*Kp;
//	Turn+=Turn_Amplitude;
//	return Turn;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    gyro_data_ready_cb();
    dmp_getdata();
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    
    Encoder_Left = (short)TIM2 -> CNT;TIM2 -> CNT = 0;
    Encoder_Right = -((short)TIM4 -> CNT);TIM4 -> CNT = 0;
    
//    printf("\r\n%d\t%d\r\n",Encoder_Left, Encoder_Right);
    
    
    Balance_Pwm = balance_UP(roll, Mechanical_angle, gyrox);
    Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);
    
    if(1==Flag_left||1==Flag_right)    
		Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz);        //===×ªÏò»·PID¿ØÖÆ
    else 
        Turn_Pwm=0.5*gyroz;
    
    moto1 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
    moto2 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
    
    Xianfu_Pwm();
    Turn_Off(roll);
    Set_Pwm(moto1, moto2);
}
