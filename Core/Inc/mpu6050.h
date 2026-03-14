/**
 * @file mpu6050.h
 * @brief MPU6050 6-axis IMU Driver
 */

#ifndef MPU6050_H
#define MPU6050_H

//Includes
#include "stm32f4xx_hal.h"
#include <math.h>

//======================================================
// MPU6050 I2C Address
//======================================================
#define MPU6050_DEVICE_ID  0X68
#define MPU6050_ADDR       (0x68 << 1)

//======================================================
// MPU6050 Register Addresses
//======================================================
#define MPU6050_REG_WHO_AM_I         0x75
#define MPU6050_REG_PWR_MGMT_1       0x6B
#define MPU6050_REG_CONFIG			 0X1A
#define MPU6050_REG_GYRO_CONFIG      0x1B
#define MPU6050_REG_ACCEL_CONFIG	 0X1C
#define MPU6050_REG_ACCEL_XOUT_H     0x3B

//======================================================
// MPU6050 Accelerometer and gyroscope ranges
//======================================================
// Accelerometer Range Settings
#define MPU6050_ACCEL_2G     0x00
#define MPU6050_ACCEL_4G     0x08
#define MPU6050_ACCEL_8G     0x10
#define MPU6050_ACCEL_16G    0x18

// Gyroscope Range Settings
#define MPU6050_GYRO_250     0x00
#define MPU6050_GYRO_500     0x08
#define MPU6050_GYRO_1000    0x10
#define MPU6050_GYRO_2000    0x18

//======================================================
// Struct to store the MPU6050 configs
//======================================================
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t accel_range;
	uint8_t gyro_range;
	float accel_sensitivity; // LSB/g
	float gyro_sensitivity;  // LSB(degree/s)
	float accel_offset[3];
	float gyro_offset[3];
	uint8_t calibrated_flag;
} MPU6050_t;

//======================================================
// Struct to store MPU6050 data
//======================================================
typedef struct {
    int16_t ax, ay, az;           // Raw accelerometer values (signed 16-bit)
    int16_t gx, gy, gz;           // Raw gyroscope values (signed 16-bit)
    int16_t temp_raw;	          // Raw temperature readings
    float ax_g, ay_g, az_g;       // Accelerometer readings in g (gravity)
    float gx_dps, gy_dps, gz_dps; // Gyroscope readings in degrees/sec
    float temp_deg;				  // Temperature in degree

} MPU6050_Data_t;

//======================================================
// Function prototypes
//======================================================
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Config(MPU6050_t *mpu,uint8_t gyro_range,uint8_t accel_range);
HAL_StatusTypeDef MPU6050_SetConfig(MPU6050_t *mpu, uint8_t ext_sync, uint8_t dlpf_level);
HAL_StatusTypeDef MPU6050_ReadData(MPU6050_t *mpu, MPU6050_Data_t *data);
HAL_StatusTypeDef MPU6050_SetOffsets(MPU6050_t *mpu,
		float ax_offset, float ay_offset, float az_offset,
		float gx_offset, float gy_offset, float gz_offset
		);
HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_t *mpu);
float radians_to_degrees(float rad);
void calculate_pitch_roll(float ax, float ay, float az, float* pitch, float* roll);
#endif
