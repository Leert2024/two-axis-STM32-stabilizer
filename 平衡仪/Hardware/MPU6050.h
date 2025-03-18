#ifndef MPU6050_H
#define MPU6050_H

typedef struct MPU6050_DataStruct{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
}MPU6050_DataStruct;

void MPU6050_WriteReg(uint8_t addr, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t addr);
void MPU6050_Init(void);
MPU6050_DataStruct MPU6050_GetData(void);

#endif
