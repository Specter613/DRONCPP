/*
 * mpu6500.c
 *
 *  Created on: Jul 9, 2026
 *      Author: luisa
 */
#include <mpu6500.hpp>

static constexpr uint16_t CS_PIN = GPIO_PIN_12;
#define CS_PORT GPIOB


MPU6500::MPU6500(SPI_HandleTypeDef *hspi) : hspi_(hspi) {}

static inline void ChipSelectLow(){
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
}

static inline void ChipSelectHigh(){
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}


void MPU6500::Init()
{
    uint8_t data[2];

    // Reset completo del dispositivo
    data[0] = 0x6B; data[1] = 0x80;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(100);

    // Despierta el sensor, clock interno
    data[0] = 0x6B; data[1] = 0x01;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(100);

    // Filtro pasa bajas del giroscopio, DLPF = 41 Hz
    data[0] = 0x1A; data[1] = 0x03;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(10);

    // Giroscopio ±500 °/s
    data[0] = 0x1B; data[1] = 0x08;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(10);

    // Acelerómetro ±4 g
    data[0] = 0x1C; data[1] = 0x08;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(10);

    // Filtro pasa bajas del acelerómetro, 41 Hz
    data[0] = 0x1D; data[1] = 0x03;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(10);

    // Frecuencia de muestreo, 500 Hz
    data[0] = 0x19; data[1] = 0x01;
    ChipSelectLow();
    HAL_SPI_Transmit(hspi_, data, 2, HAL_MAX_DELAY);
    ChipSelectHigh();
    HAL_Delay(10);
}

void MPU6500::ReadRaw()
{
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};

    // Dirección inicial (ACCEL_XOUT_H) + bit de lectura
    tx[0] = 0x3B | 0x80;

    ChipSelectLow();
    HAL_SPI_TransmitReceive(hspi_, tx, rx, 15, HAL_MAX_DELAY);
    ChipSelectHigh();

    raw_.ax = static_cast<int16_t>((rx[1] << 8) | rx[2]);
    raw_.ay = static_cast<int16_t>((rx[3] << 8) | rx[4]);
    raw_.az = static_cast<int16_t>((rx[5] << 8) | rx[6]);

    raw_.gx = static_cast<int16_t>((rx[9]  << 8) | rx[10]);
    raw_.gy = static_cast<int16_t>((rx[11] << 8) | rx[12]);
    raw_.gz = static_cast<int16_t>((rx[13] << 8) | rx[14]);
}

void MPU6500::Process()
{
    ReadRaw();

    data_.ax = raw_.ax / MPU6500_ACCEL_SCALE;
    data_.ay = raw_.ay / MPU6500_ACCEL_SCALE;
    data_.az = raw_.az / MPU6500_ACCEL_SCALE;
    data_.ax -= accelOffset_[0];
    data_.ay -= accelOffset_[1];
    data_.az -= accelOffset_[2];

    data_.gx = raw_.gx / MPU6500_GYRO_SCALE - gyroOffset_[0];
    data_.gy = raw_.gy / MPU6500_GYRO_SCALE - gyroOffset_[1];
    data_.gz = raw_.gz / MPU6500_GYRO_SCALE - gyroOffset_[2];

    data_.roll  = atan2f(data_.ay, data_.az) * 180.0f / static_cast<float>(M_PI);
    data_.pitch = atan2f(-data_.ax, sqrtf(data_.ay * data_.ay + data_.az * data_.az))
                  * 180.0f / static_cast<float>(M_PI);
}

void MPU6500::CalibrateGyro()
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    const uint16_t samples = 1000;

    for(uint16_t i = 0; i < samples; i++)
    {
        ReadRaw();
        sumX += raw_.gx;
        sumY += raw_.gy;
        sumZ += raw_.gz;
        HAL_Delay(2);
    }

    gyroOffset_[0] = (sumX / static_cast<float>(samples)) / MPU6500_GYRO_SCALE;
    gyroOffset_[1] = (sumY / static_cast<float>(samples)) / MPU6500_GYRO_SCALE;
    gyroOffset_[2] = (sumZ / static_cast<float>(samples)) / MPU6500_GYRO_SCALE;
}

void MPU6500::CalibrateAccel()
{
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    const uint16_t samples = 1000;

    for(uint16_t i = 0; i < samples; i++)
    {
        Process();
        sumX += data_.ax;
        sumY += data_.ay;
        sumZ += data_.az;
        HAL_Delay(2);
    }

    accelOffset_[0] = sumX / samples;
    accelOffset_[1] = sumY / samples;
    // El eje Z debe medir +1g cuando está horizontal
    accelOffset_[2] = (sumZ / samples) - 1.0f;
}
