Writed in 2020/8/22
by Zichuan Ou
production cycle: 8/1 - 8/22

Bluetooth Balancing Car "Yifan"
	
MCU: stm32f103ZET6
Communication_module: HC-05
	RXD <-> PA2
	TXD <-> PA3
	STATE <-> PA4
Attitude_sensor: MPU6050
	SCL <-> PB6
	SDA <-> PB7
	INT <-> PA11
Motor_drive: TB6612FNG
	PWMA <-> PA8
	AIN1 <-> PB8
	AIN2 <-> PB9
	STBY <-> PC0
	PWMB <-> PC7
	BIN1 <-> PB10
	BIN2 <-> PB11
Motor: GM25-370*2
	MOTOR_R_A <-> PD12
	MOTOR_R_B <-> PD13
	MOTOR_L_A <-> PA15
	MOTOR_L_B <-> PB3
Power_module: MB-102
Battery: 18650*2
Pan_tilt: sg90*2
	Motor_yaw <-> PA6
	Motor_pitch <-> PA1
	
MCU_Source:
	TIM*6
		TIM1 & TIM8: motor PWM output
		TIM2 & TIM4: motor speed encoder
		TIM3 & TIM5: sg90 angle control
	USART*2
		USART1: communicate and debug with upper computer
		USART2: communicate with HC-05 
	NVIC*1
		PA11: communicate with MPU6050 and update posture
	GPIO*26
		TIM*8
		USART*4
		NVIC*1
		LED*3
		Bluetooth_state*1
		I2C*2
		MOTOR_PWM*4
		MOTOR_EN*1
		RCC*2
	SYSCLK = 72MHz
	

