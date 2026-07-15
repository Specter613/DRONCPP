/*
 * mpu6500.h
 *
 *  Created on: Jul 9, 2026
 *      Author: luisa
 */

#ifndef INC_MPU6500_HPP_
#define INC_MPU6500_HPP_

#include <cstdint>
#include <cmath>
#include "main.h"

//Escalas
#define MPU6500_ACCEL_SCALE   8192.0f
#define MPU6500_GYRO_SCALE    65.5f

//Estructuras
//Datos crudos
struct MPU6500_Raw{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

};
//Datos Procesados
struct MPU6500_Data
{
    float ax, ay, az;
    float gx, gy, gz;
    float pitch, roll;
};

class MPU6500{
public:
	explicit MPU6500(SPI_HandleTypeDef *hspi);
	void Init();
	void CalibrateGyro();
	void CalibrateAccel();
	void ReadRaw();
	void Process();

	const MPU6500_Data& GetData() const { return data_; }
	const MPU6500_Raw& GetRaw() const { return raw_; }

private:
	SPI_HandleTypeDef *hspi_;
	MPU6500_Raw raw_ = {};
	MPU6500_Data data_ = {};

    float gyroOffset_[3]  = {0.0f, 0.0f, 0.0f};
    float accelOffset_[3] = {0.0f, 0.0f, 0.0f};
};

#endif
