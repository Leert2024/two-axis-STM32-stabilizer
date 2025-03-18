#include "stm32f10x.h"// Device header
#include "Delay.h"
#include "OLED.h"
#include "leertHardware.h"
#include "servo.h"
#include "MPU6050.h"
#include "math.h"

#define PI 3.1415926535

MPU6050_DataStruct MPU6050_DataAdj;

float roll, pitch;
float rollAdj, pitchAdj;

/**
 * @brief 校准姿态(将当前状态设定为角度零点)
 */
void setBeginStatus(){
	OLED_Clear();
	OLED_ShowString(1,1,"adj will start  ");
	OLED_ShowString(2,1,"in one second   ");
	OLED_ShowString(3,1,"dont move       ");
	Delay_ms(1000);
	OLED_Clear();
	OLED_ShowString(2,1,"ADJUST");
	OLED_ShowString(3,1,"BEGIN");
	digitalWrite('A', 3, 0);//点亮小灯

	MPU6050_DataStruct data;
	int32_t AccXAdj = 0, AccYAdj = 0, AccZAdj = 0;
	for(uint8_t i = 0; i < 100; i++){
		data = MPU6050_GetData();
		AccXAdj -= data.AccX;
		AccYAdj -= data.AccY;
		AccZAdj -= data.AccZ;
	}
	AccXAdj /= 100;
	AccYAdj /= 100;
	AccZAdj /= 100;

	MPU6050_DataAdj.AccX = AccXAdj;
	MPU6050_DataAdj.AccY = AccYAdj;
	MPU6050_DataAdj.AccZ = AccZAdj;
	
	rollAdj = atan((float)MPU6050_DataAdj.AccY/MPU6050_DataAdj.AccZ)*180/PI;
	pitchAdj = atan((float)MPU6050_DataAdj.AccX/MPU6050_DataAdj.AccZ)*180/PI;

	OLED_Clear();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
	OLED_ShowString(1,1,"pose adj done   ");
	digitalWrite('A', 3, 1);//熄灭小灯
	Delay_ms(500);
	OLED_Clear();
}

int main(void){
	//初始化各个外设
	OLED_Init();
	OLED_ShowString(1,1,"press key");
	Servo1_Init();
	Servo2_Init();
	MPU6050_Init();
	pinMode('B', GPIO_Pin_1, GPIO_Mode_IPU);//初始化按键
	pinMode('A', GPIO_Pin_3, GPIO_Mode_Out_PP);//初始化小灯
	digitalWrite('A', 3, 1);//熄灭小灯
	Servo1_SetAngle(90);
	Servo2_SetAngle(90);

	while(!getKey('B', 1, 0)){
		MPU6050_DataStruct data = MPU6050_GetData();
		OLED_ShowSignedNum(2,1,data.AccX, 5);
		OLED_ShowSignedNum(3,1,data.AccY, 5);
		OLED_ShowSignedNum(4,1,data.AccZ, 5);
		OLED_ShowSignedNum(2,8,data.GyroX, 5);
		OLED_ShowSignedNum(3,8,data.GyroY, 5);
		OLED_ShowSignedNum(4,8,data.GyroZ, 5);	
	};//等待按键按下

	setBeginStatus();
	
	while(1){
		MPU6050_DataStruct data = MPU6050_GetData();
		int16_t x = data.AccX;
		int16_t y = data.AccY;
		int16_t z = data.AccZ;

		roll = atan((float)y/z)*180/PI;
		pitch = atan((float)x/z)*180/PI;

		OLED_ShowString(1,1,"AccX");
		OLED_ShowString(2,1,"AccY");
		OLED_ShowString(3,1,"AccZ");
		OLED_ShowSignedNum(1,5, x, 2);
		OLED_ShowSignedNum(2,5, y, 2);
		OLED_ShowSignedNum(3,5, z, 2);
		
		float servo_roll = - roll + rollAdj + 90;
		float servo_pitch = - pitch + pitchAdj + 90;
		
		OLED_ShowSignedNum(4,1, servo_roll, 5);
		OLED_ShowSignedNum(4,8, servo_pitch, 5);
		Servo1_SetAngle(servo_roll);
		Servo2_SetAngle(servo_pitch);
	}
}
