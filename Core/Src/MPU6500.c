//
// Created by PJLAB\lijialun on 10/11/24.
//
#include <stdio.h>

#include "MPU6500.h"
#include "main.hpp"
#include "i2c.h"

Struct_MPU6500 MPU6500;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

void MPU6500_WriteByte(const uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&hi2c3, MPU6500_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1);
}

void MPU6500_WriteBytes(const uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Write(&hi2c3, MPU6500_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6500_ReadByte(const uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c3, MPU6500_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU6500_ReadBytes(const uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c3, MPU6500_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6500_Initialization(void)
{
	HAL_Delay(50);
	uint8_t who_am_i = 0;
	printf("Checking MPU6500...\r\n");

	MPU6500_ReadByte(MPU6500_WHO_AM_I, &who_am_i);
	if(who_am_i == 0x70)
	{
		printf("MPU6500 who_am_i = 0x%02x...OK\r\n", who_am_i);
	}
	else
	{
		while(1)
		{
			printf("ERROR!\r\n");
			printf("MPU6500 who_am_i : 0x%02x should be 0x68\r\n", who_am_i);
			printf("who am i error. Can not recognize mpu6500\r\n");
			HAL_Delay(1000);
		}
	}

	//Reset the whole module before initialization
	MPU6500_WriteByte(MPU6500_PWR_MGMT_1, 0x1<<7);
	HAL_Delay(100);

	//Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU6500*/
	MPU6500_WriteByte(MPU6500_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	//Sample rate divider
	/*Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV) */
	//	MPU6500_Writebyte(MPU6500_SMPRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz
	MPU6500_WriteByte(MPU6500_SMPRT_DIV, 39); // Sample Rate = 200Hz
	HAL_Delay(50);

	//FSYNC and DLPF setting
	/*DLPF is set to 0*/
	MPU6500_WriteByte(MPU6500_CONFIG, 0x00);
	HAL_Delay(50);

	//GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x0;
	MPU6500_WriteByte(MPU6500_GYRO_CONFIG, FS_SCALE_GYRO<<3);
	HAL_Delay(50);

	//ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x0;
	MPU6500_WriteByte(MPU6500_ACCEL_CONFIG, FS_SCALE_ACC<<3);
	HAL_Delay(50);

	MPU6500_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
	printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\r\n",LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC); // @suppress("Float formatting support")

	//Interrupt PIN setting
	uint8_t INT_LEVEL = 0x0; //0 - active high, 1 - active low
	uint8_t LATCH_INT_EN = 0x0; //0 - INT 50us pulse, 1 - interrupt clear required
	uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation
	MPU6500_WriteByte(MPU6500_INT_PIN_CFG, (INT_LEVEL<<7)|(LATCH_INT_EN<<5)|(INT_RD_CLEAR<<4)); //
	HAL_Delay(50);

	//Interrupt enable setting
	uint8_t DATA_RDY_EN = 0x1; // 1 - enable, 0 - disable
	MPU6500_WriteByte(MPU6500_INT_ENABLE, DATA_RDY_EN);
	HAL_Delay(50);

	printf("MPU6500 setting is finished\r\n");
}

/*Get Raw Data from sensor*/
void MPU6500_Get6AxisRawData(Struct_MPU6500* mpu6500)
{
	uint8_t data[14];
	MPU6500_ReadBytes(MPU6500_ACCEL_XOUT_H, 14, data);

	mpu6500->acc_x_raw = (data[0] << 8) | data[1];
	mpu6500->acc_y_raw = (data[2] << 8) | data[3];
	mpu6500->acc_z_raw = (data[4] << 8) | data[5];

	mpu6500->temperature_raw = (data[6] << 8) | data[7];

	mpu6500->gyro_x_raw = ((data[8] << 8) | data[9]);
	mpu6500->gyro_y_raw = ((data[10] << 8) | data[11]);
	mpu6500->gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU6500_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch(FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

/*Convert Unit. acc_raw -> g, gyro_raw -> degree per second*/
void MPU6500_DataConvert(Struct_MPU6500* mpu6500)
{
	//printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO,LSB_Sensitivity_ACC);
	mpu6500->acc_x = mpu6500->acc_x_raw / LSB_Sensitivity_ACC;
	mpu6500->acc_y = mpu6500->acc_y_raw / LSB_Sensitivity_ACC;
	mpu6500->acc_z = mpu6500->acc_z_raw / LSB_Sensitivity_ACC;

	mpu6500->temperature = (float)(mpu6500->temperature_raw)/340+36.53;

	mpu6500->gyro_x = mpu6500->gyro_x_raw / LSB_Sensitivity_GYRO;
	mpu6500->gyro_y = mpu6500->gyro_y_raw / LSB_Sensitivity_GYRO;
	mpu6500->gyro_z = mpu6500->gyro_z_raw / LSB_Sensitivity_GYRO;
}

int MPU6500_DataReady(void)
{
	return HAL_GPIO_ReadPin(MPU6500_INT_PORT, MPU6500_INT_PIN);
}

void MPU6500_ProcessData(Struct_MPU6500* mpu6500)
{
	MPU6500_Get6AxisRawData(mpu6500);
	MPU6500_DataConvert(mpu6500);
}
