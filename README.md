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
MPU6050_Init(&mpu, &hi2c1);
MPU6050_Config(&mpu, MPU6050_GYRO_250, MPU6050_ACCEL_2G);

MPU6050_Data_t data;
while(1) {
    MPU6050_ReadData(&mpu, &data);
    printf("Accel: %.2f g\n", data.ax_g);
    HAL_Delay(500);
}
```
