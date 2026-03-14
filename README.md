# MPU6050 STM32 Driver

6-axis IMU sensor driver for STM32F4xx microcontrollers.

## Features
- I²C communication (400 kHz)
- Configurable gyroscope & accelerometer ranges
- Raw + scaled data output
- Temperature measurement
- Pitch/roll calculation

## Hardware
- STM32F4 with I²C peripheral
- MPU6050 sensor with 4.7kΩ pull-ups on SDA/SCL
- 3.3V power supply

## Usage
```c
MPU6050_t mpu;
MPU6050_Data_t sensor_data;

MPU6050_Init(&mpu, &hi2c1);
MPU6050_Config(&mpu, MPU6050_GYRO_250, MPU6050_ACCEL_2G);
MPU6050_Calibrate(&mpu);

while (1)
{
    MPU6050_ReadData(&mpu, &sensor_data);

    // Print scaled accelerometer values
    printf("%.2f\t%.2f\t%.2f\n",
           sensor_data.ax_g,
           sensor_data.ay_g,
           sensor_data.az_g);

    // Print scaled gyroscope values
    printf("%.2f\t%.2f\t%.2f\n",
           sensor_data.gx_dps,
           sensor_data.gy_dps,
           sensor_data.gz_dps);

    // Calculate roll and pitch
    float pitch, roll;

    calculate_pitch_roll(sensor_data.ax_g,
                         sensor_data.ay_g,
                         sensor_data.az_g,
                         &pitch,
                         &roll);

    printf("Pitch: %.2f deg\tRoll: %.2f deg\n", pitch, roll);

    HAL_Delay(500);
}

```
