#include "stm32f10x.h"
#include "leertHardware.h"

/**
 * @brief   初始化水平方向舵机引脚及PWM
 * @note    初始化后舵机角度默认为0度
 */
void Servo1_Init(void){
    //水平方向舵机使用PA0引脚，定时器为TIM2，通道为1
    pinMode('A', GPIO_Pin_0, GPIO_Mode_AF_PP);//配置引脚为复用推挽输出
    analogWrite('A', 0, 20000, 500);
}

/**
 * @brief   初始化垂直方向舵机引脚及PWM
 * @note    初始化后舵机角度默认为0度
 */
void Servo2_Init(void){
    //垂直方向舵机使用PA1引脚，定时器为TIM2，通道为2
    pinMode('A', GPIO_Pin_1, GPIO_Mode_AF_PP);//配置引脚为复用推挽输出
    analogWrite('A', 1, 20000, 500);
}

/**
 * @brief   设置水平方向舵机角度
 * @param   angle 角度值，范围0至180
 */
void Servo1_SetAngle(float angle){
	if(angle > 180 || angle < 0)return;
	TIM_SetCompare1(TIM2, (int)(500 + 945.0 / 90.0 * angle));
}

/**
 * @brief   设置垂直方向舵机角度
 * @param   angle 角度值，范围0至180
 */
void Servo2_SetAngle(float angle){
	if(angle > 180 || angle < 0)return;
	TIM_SetCompare2(TIM2, (int)(500 + 945.0 / 90.0 * angle));
}