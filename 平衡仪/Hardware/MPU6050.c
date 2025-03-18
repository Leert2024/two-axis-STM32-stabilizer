#include "stm32f10x.h"                  // Device header
#include "leertI2C.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"

#define MPU6050_ADDR    0xD0
#define MPU6050_PORT    'B'
#define MPU6050_SCLPIN  10
#define MPU6050_SDAPIN  11

/**
 * @brief   向MPU6050模块指定寄存器地址写入一个字节
 * @param   addr MPU6050寄存器地址
 * @param   data 要写入的数据(一个字节)
 */
void MPU6050_WriteReg(uint8_t addr, uint8_t data){
    I2CBegin();
    I2CSendByte(MPU6050_ADDR);
    if(I2CRecvACK())return;//找不到MPU6050模块
    I2CSendByte(addr);
    I2CRecvACK();
    I2CSendByte(data);
    I2CRecvACK();
    I2CStop();
}

/**
 * @brief   从MPU6050模块指定寄存器地址读入一个字节
 * @param   addr MPU6050寄存器地址
 * @retval  读取到的数据(一个字节)
 */
uint8_t MPU6050_ReadReg(uint8_t addr){
    I2CBegin();
    I2CSendByte(MPU6050_ADDR);
    I2CRecvACK();//找不到MPU6050模块
    I2CSendByte(addr);
    I2CRecvACK();

    //读取一个字节
    I2CBegin();
    I2CSendByte(MPU6050_ADDR | 0x01);
    I2CRecvACK();
    uint8_t data = I2CRecvByte();
    I2CSendACK(1);//因为只要读取1个字节，故不给从机应答，读取结束
    I2CStop();

    return data;
}

/**
 * @brief   初始化MPU6050模块
 * @note    默认SCL引脚为PB10，SDA引脚为PB11，若要修改，可在MPU6050.h的宏定义中修改
 */
void MPU6050_Init(){
    setI2C(MPU6050_PORT, MPU6050_SCLPIN, MPU6050_SDAPIN);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

/**
 * @brief   获取各方向的加速度值与角速度值
 * @retval	一个MPU6050_DataStruct对象，其成员包含上述数据
 */
MPU6050_DataStruct MPU6050_GetData(){
	MPU6050_DataStruct data;
	
	uint8_t dataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	uint8_t dataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	data.AccX = dataH << 8 | dataL;
	
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	data.AccY = dataH << 8 | dataL;
	
	dataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	data.AccZ = dataH << 8 | dataL;
	
	dataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	data.GyroX = dataH << 8 | dataL;
	
	dataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	data.GyroY = dataH << 8 | dataL;
	
	dataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	dataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	data.GyroZ = dataH << 8 | dataL;
	
	return data;
}
