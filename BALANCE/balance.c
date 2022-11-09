#include "balance.h"

__IO int32_t Time_count = 0; //Time variable //计时变量
__IO static uint16_t Action_num = 0; 
__IO static uint16_t Action_cnt = 0;
__IO static uint16_t Running = 0;

Action dst[5];

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     

/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
uint8_t flag = 0;
uint8_t wait = 60;
uint8_t mode = 0;

void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();

    Action_init(4);
    
    Running = 0;
    while(1)
    {	
        // This task runs at a frequency of 100Hz (10ms control once)
        //此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

        lastWakeTime = getSysTickCnt();

        //Time count is no longer needed after 30 seconds
        //时间计数，30秒后不再需要, 加一次10ms, 当运行时基用
        Time_count++;
        if(Time_count >= 10000000L) // 100分钟为一个运行周期
            Time_count = 0;

        flag = Key();

        if(flag > 0) {
            mode++;
            flag = 0;
        }
        
        cheak_time();

        //获取编码器数据，并转换位国际单位m/s
        Get_Velocity_Form_Encoder();

        Drive_Motor(dst[Action_cnt].x_speed, dst[Action_cnt].y_speed, dst[Action_cnt].z_speed);  

        if(Turn_Off(Voltage) == 0) 
        {
            MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
            MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
            MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
            MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
            Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0); 
        }
        else 
        {
            Set_Pwm(0, 0, 0, 0, 0);
        }

	}  
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude = 3.5; //Wheel target speed limit //车轮目标速度限幅

	    if (Running == 1) 
        {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target = target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target = target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target = target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target = target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
        else
        {
            MOTOR_A.Target = 0;
            MOTOR_B.Target = 0;
            MOTOR_C.Target = 0;
            MOTOR_D.Target = 0;
        }

}

void Action_init(uint8_t num) {
    uint8_t i = 0;
    uint8_t static flag = 0;

    if(flag == 0) //只做一遍
    {
        Action_num = num;

        for (i = 0; i < 5; i++)
        {
            dst[i].enable = 0;
        }

        dst[0].enable = 1;
        dst[0].distance = 0.5;
        dst[0].x_speed = 0.4;
        dst[0].y_speed = 0;
        dst[0].z_speed = 0;
        dst[0].need_time = (int)((dst[0].distance / dst[0].x_speed) * 100);
        
        dst[1].enable = 1;
        dst[1].distance = 1.54;
        dst[1].x_speed = 0;
        dst[1].y_speed = 0;
        dst[1].z_speed = 2;
        dst[1].need_time = (int)((dst[1].distance / dst[1].z_speed) * 100);

        dst[2].enable = 1;
        dst[2].distance = 1.54 * 2;
        dst[2].x_speed = 0;
        dst[2].y_speed = 0;
        dst[2].z_speed = 2;
        dst[2].need_time = (int)((dst[2].distance / dst[2].z_speed) * 100);

        dst[3].enable = 1;
        dst[3].distance = 0.5;
        dst[3].x_speed = 0;
        dst[3].y_speed = 0.4;
        dst[3].z_speed = 0;
        dst[3].need_time = (int)((dst[3].distance / dst[3].y_speed) * 100);
        flag++;
    }
    else
        return;
}

/**************************************************************************
函数功能：检测动作是否完成
入口参数：
返回  值：无
**************************************************************************/
void cheak_time(void) {
    static int32_t time = 0;

    if(mode == 1) 
    {
        if(dst[Action_cnt].enable == 1)
        {
            if(Running == 0)
            {
                if(Time_count - time >= wait) 
                {
                    dst[Action_cnt].start_time = Time_count;;
                    Running = 1;
                }
            }
            else
            {
                if((Time_count - dst[Action_cnt].start_time) >= dst[Action_cnt].need_time)
                {
                    Running = 0;
                    Action_cnt++;  
                    time = Time_count;
                }
            }
        }
        else
        {
            mode = 0;
            Action_cnt = 0;
        }
    }
}
/**************************************************************************
函数功能：左转，基于编码器
入口参数：Turn结构体
返回  值：无
**************************************************************************/
typedef struct {
    short encoder_target;
    short now_encoder;
    short lastError;
    float KP;
    float KD;
} Turn;


float PID_TurnLeft(Turn *turn) 
{
    int Ek = 0;  
    float out = 0;   

    Ek = turn->now_encoder - turn->encoder_target;

    out = Ek * turn->KP - (Ek - turn->lastError) * turn->KD;

    turn->lastError = Ek;

	return out;
}

/**************************************************************************
函数功能：走直线
入口参数：
返回  值：无
**************************************************************************/
// float PID_Straight(float yaw) {
//     float out = 0;
//     float Ek = 0;
//     //static float out = 0;

//     Ek = yaw - straight_control.SetPoint;
          
// 	out = Ek * straight_control.KP - (Ek - straight_control.LastError) * straight_control.KD; //位置式PID

// 	straight_control.LastError = Ek;

//     return out;
// }

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	 

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10 || EN==0)//||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}

/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias = Target - Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+1800*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias, Pwm, Last_bias;
	 Bias = Target - Encoder; //Calculate the deviation //计算偏差
	 Pwm += Velocity_KP * (Bias - Last_bias) + 1600 * Bias;  
	 if(Pwm > 16700) Pwm = 16700;
	 if(Pwm < -16700) Pwm = -16700;
	 Last_bias = Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+1500*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}

/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
uint8_t Key(void)
{	
	u8 tmp = 0;
	tmp=click_N_Double_MPU6050(50); 
    return tmp;
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}

/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

