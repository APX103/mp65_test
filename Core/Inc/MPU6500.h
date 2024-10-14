//
// Created by PJLAB\lijialun on 10/11/24.
//

#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.hpp"

#define MPU6500_ADDR 0xD0


#define MPU6500_SMPRT_DIV 0X19
#define MPU6500_WHO_AM_I 0X75
#define MPU6500_CONFIG 0X1A
#define MPU6500_GYRO_CONFIG 0X1B
#define MPU6500_ACCEL_CONFIG 0X1C
#define MPU6500_INT_PIN_CFG 0X37
#define MPU6500_INT_ENABLE 0X38
#define MPU6500_INT_STATUS 0X3A
#define MPU6500_ACCEL_XOUT_H 0X3B
#define MPU6500_ACCEL_XOUT_L 0X3C
#define MPU6500_PWR_MGMT_1 0X6B //most important



#define MPU6500_INT_PORT 	GPIOB
#define MPU6500_INT_PIN 	GPIO_PIN_5


typedef struct _MPU6500{
    short acc_x_raw;
    short acc_y_raw;
    short acc_z_raw;
    short temperature_raw;
    short gyro_x_raw;
    short gyro_y_raw;
    short gyro_z_raw;

    float acc_x;
    float acc_y;
    float acc_z;
    float temperature;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}Struct_MPU6500;

extern Struct_MPU6500 MPU6500;

void MPU6500_WriteByte(uint8_t reg_addr, uint8_t val);
void MPU6500_WriteBytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6500_ReadByte(uint8_t reg_addr, uint8_t* data);
void MPU6500_ReadBytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6500_Initialization(void);
void MPU6500_Get6AxisRawData(Struct_MPU6500* mpu6500);
int MPU6500_DataReady(void);
void MPU6500_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC);
void MPU6500_DataConvert(Struct_MPU6500* mpu6500);
void MPU6500_ProcessData(Struct_MPU6500* mpu6500);

#ifdef __cplusplus
}
#endif

#endif //MPU6500_H
