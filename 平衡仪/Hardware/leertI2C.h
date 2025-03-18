#ifndef LEERT12C_H
#define LEERT12C_H

void setI2C(char c, uint16_t SCLPinNum, uint16_t SDAPinNum);
void I2CBegin(void);
void I2CStop(void);
void I2CSendByte(uint8_t value);
uint8_t I2CRecvByte(void);
void I2CSendACK(uint8_t value);
uint8_t I2CRecvACK(void);

#endif
