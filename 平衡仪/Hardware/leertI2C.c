#include "stm32f10x.h"
#include "Delay.h"

GPIO_TypeDef* SCL_PORT;
uint16_t SCL_PIN;
uint16_t SDA_PIN;

/**
 * @brief   初始化I2C通信
 * @param	c 代表GPIO组别的字符
 * @param	SCLpin SCL引脚对应的编号
 * @param	SDApin SDA引脚对应的编号
 */
void setI2C(char c, uint16_t SCLPinNum, uint16_t SDAPinNum){
    if(c == 'A'){
        SCL_PORT = GPIOA;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }else if(c == 'B'){
        SCL_PORT = GPIOB;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }else return;
    SCL_PIN = 0x0001<<SCLPinNum;
    SDA_PIN = 0x0001<<SDAPinNum;
    GPIO_InitTypeDef GPIO_InitStructure;

    //引脚设置为开漏输出
    //在开漏输出模式下，仅输出低电平有效，不输出低电平时，电平由外界决定
    //因此，开漏输出模式既有输出效用，也有输入效用
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;

	GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//输出速度为50MHz
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);
    GPIO_SetBits(SCL_PORT, SCL_PIN | SDA_PIN);//开始时为高电平，即I2C总线处于空闲状态
}

/**
 * @brief   在SCL引脚输出指定电平
 * @param   value 写入的电平值(低电平为0，若不为0则为高电平)
 */
void writeSCL(uint8_t value){
    GPIO_WriteBit(SCL_PORT, SCL_PIN, (BitAction)value);
    Delay_us(10);
}

/**
 * @brief   在SDA引脚输出指定电平
 * @param   value 写入的电平值(低电平为0，若不为0则为高电平)
 */
void writeSDA(uint8_t value){
    GPIO_WriteBit(SCL_PORT, SDA_PIN, (BitAction)value);
    Delay_us(10);
}

uint8_t readSCL(){
    uint8_t value = GPIO_ReadInputDataBit(SCL_PORT, SCL_PIN);
    Delay_us(10);
    return value;
}

uint8_t readSDA(){
    uint8_t value = GPIO_ReadInputDataBit(SCL_PORT, SDA_PIN);
    Delay_us(10);
    return value;
}

void I2CBegin(void){
    writeSDA(1);
    writeSCL(1);
    writeSDA(0);
    writeSCL(0);
}

void I2CStop(void){
    writeSDA(0);
    writeSCL(1);
    writeSDA(1);
}

/**
 * @brief   发送一个字节
 * @param   value 要发送的字节的值
 */
void I2CSendByte(uint8_t value){
    for(uint8_t i = 0; i < 8; i++){
        writeSDA(value & (0x80 >> i));
        writeSCL(1);
        writeSCL(0); 
    }
}

/**
 * @brief   接收一个字节
 * @retval  接收到的字节的值
 */
uint8_t I2CRecvByte(void){
    uint8_t value = 0x00;
    writeSDA(1);
    for(uint8_t i = 0; i < 8; i++){
        writeSCL(1);
        if (readSDA()) value |= (0x80 >> i);
        writeSCL(0);
    }
    return value;
}

/**
 * @brief   发送应答
 * @param   value 应答的值
 */
void I2CSendACK(uint8_t value){
    writeSDA(value);
    writeSCL(1);
    writeSCL(0); 
}

/**
 * @brief   接收应答
 * @retval  应答的值
 */
uint8_t I2CRecvACK(void){
    uint8_t value;
    writeSDA(1);//主机SDA进入浮空状态，此后SDA电平由从机决定
    writeSCL(1);
    value = readSDA();//读取SDA引脚的电平，即从机向SDA输出的电平
    writeSCL(0);
    return value;
}