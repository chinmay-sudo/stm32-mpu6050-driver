/**
 * @file mpu6050.c
 * @brief MPU6050 6-axis IMU Driver
 */

#include "mpu6050.h"
#include <math.h>

/**
 * @brief Initialize MPU6050 sensor
 * @param[in,out] mpu MPU6050 Structure
 * @param[in] hi2c I2C Handle
 * @return HAL_OK if successful
 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c) {
	// Check for NULL Pointers
	if(mpu == NULL || hi2c == NULL){
		return HAL_ERROR;
	}

    mpu->hi2c = hi2c;
    HAL_StatusTypeDef ret;
    uint8_t who_am_i = 0;

    // Verify sensor connection by reading WHO_AM_I register
    ret = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &who_am_i, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK || who_am_i != MPU6050_DEVICE_ID) return HAL_ERROR;

    // Power up sensor by clearing sleep bit in PWR_MGMT_1 register
    uint8_t pwr_mgmt = 0x00;
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &pwr_mgmt, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK || who_am_i != MPU6050_DEVICE_ID) return HAL_ERROR;

    return ret;
}

/**
 * @brief Configure measurement ranges for accelerometer and gyroscope
 */
HAL_StatusTypeDef MPU6050_Config(MPU6050_t *mpu, uint8_t gyro_range, uint8_t accel_range) {
    // Check for NULL Pointer
    if(mpu == NULL){
    	return HAL_ERROR;
    }

    // Set measurement ranges by writing to GYRO_CONFIG and ACCEL_CONFIG registers
    HAL_StatusTypeDef ret;
    mpu->gyro_range = gyro_range;
    mpu->accel_range = accel_range;

    ret = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR, GYRO_CONFIG, 1, &gyro_range, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    ret = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR, ACCEL_CONFIG, 1, &accel_range, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // Gyroscope sensitivity
    switch (gyro_range) {
        case MPU6050_GYRO_250:  mpu->gyro_sensitivity = 131.0f; break;
        case MPU6050_GYRO_500:  mpu->gyro_sensitivity = 65.5f; break;
        case MPU6050_GYRO_1000: mpu->gyro_sensitivity = 32.8f; break;
        case MPU6050_GYRO_2000: mpu->gyro_sensitivity = 16.4f; break;
        default: return HAL_ERROR;
    }

    // Accelerometer sensitivity
    switch (accel_range) {
        case MPU6050_ACCEL_2G:  mpu->accel_sensitivity = 16384.0f; break;
        case MPU6050_ACCEL_4G:  mpu->accel_sensitivity = 8192.0f; break;
        case MPU6050_ACCEL_8G:  mpu->accel_sensitivity = 4096.0f; break;
        case MPU6050_ACCEL_16G: mpu->accel_sensitivity = 2048.0f; break;
        default: return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Configure MPU6050 EXT_SYNC and DLPF registers
 */

HAL_StatusTypeDef MPU6050_SetConfig(MPU6050_t *mpu, uint8_t ext_sync, uint8_t dlpf_level) {
	// Check for NULL Pointer
	if(mpu == NULL){
		return HAL_ERROR;
	}
	uint8_t data = (ext_sync << 3) | (dlpf_level & 0x07);
    return HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR, MPU6050_REG_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read sensor data
 */
HAL_StatusTypeDef MPU6050_ReadData(MPU6050_t *mpu, MPU6050_Data_t *data) {
	// Check for NULL Pointer
	if(mpu == NULL || data == NULL){
		return HAL_ERROR;
	}
    uint8_t raw[14];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, raw, 14, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // Raw data
    data->ax = (int16_t)(raw[0] << 8 | raw[1]);
    data->ay = (int16_t)(raw[2] << 8 | raw[3]);
    data->az = (int16_t)(raw[4] << 8 | raw[5]);
    data->temp_raw = (int16_t)(raw[6] << 8 | raw[7]);
    data->gx = (int16_t)(raw[8] << 8 | raw[9]);
    data->gy = (int16_t)(raw[10] << 8 | raw[11]);
    data->gz = (int16_t)(raw[12] << 8 | raw[13]);

    // Scale to physical units (g, deg, dps)
    data->ax_g = data->ax / mpu->accel_sensitivity;
    data->ay_g = data->ay / mpu->accel_sensitivity;
    data->az_g = data->az / mpu->accel_sensitivity;
    data->temp_deg = (data->temp_raw / 340.0f) + 36.53f;
    data->gx_dps = data->gx / mpu->gyro_sensitivity;
    data->gy_dps = data->gy / mpu->gyro_sensitivity;
    data->gz_dps = data->gz / mpu->gyro_sensitivity;

    return HAL_OK;
}

/**
 * @brief Calculate Pitch and Roll
 */
float radians_to_degrees(float rad) {
    return rad * (180.0f / M_PI);
}

/**
 * @brief Calculate pitch and roll from accelerometer
 */
void calculate_pitch_roll(float ax, float ay, float az, float* pitch, float* roll) {
	// Check for NULL Pointer
	if(pitch == NULL || roll == NULL){
		return;
	}

    *roll  = radians_to_degrees(atan2f(ay, az));
    *pitch = radians_to_degrees(atan2f(-ax, sqrtf(ay * ay + az * az)));
}










