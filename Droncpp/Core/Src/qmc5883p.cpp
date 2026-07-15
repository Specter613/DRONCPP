/*
 * qmc5883p.cpp
 *
 *  Created on: 14 jul 2026
 *      Author: specter0163
 */

#include "qmc5883p.hpp"

QMC5883::QMC5883(I2C_HandleTypeDef *hi2c) : hi2c_(hi2c) {}

void QMC5883::Init()
{
    uint8_t data;

    data = 0x00; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x00; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x00; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x02; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x37, 1, &data, 1, HAL_MAX_DELAY);

    data = 0x06; HAL_I2C_Mem_Write(hi2c_, QMC_ADDR, 0x29, 1, &data, 1, HAL_MAX_DELAY); // axis sign
    data = 0x08; HAL_I2C_Mem_Write(hi2c_, QMC_ADDR, 0x0B, 1, &data, 1, HAL_MAX_DELAY); // range 8G + set/reset
    data = 0xC9; HAL_I2C_Mem_Write(hi2c_, QMC_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY); // continuous, 200Hz
}

bool QMC5883::ReadRaw()
{
    uint8_t reg = 0x01;
    uint8_t buffer[6];

    HAL_I2C_Master_Transmit(hi2c_, QMC_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c_, QMC_ADDR, buffer, 6, HAL_MAX_DELAY);

    raw_.mx = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    raw_.my = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    raw_.mz = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    return true;
}

bool QMC5883::Process()
{
    if(!ReadRaw()) return false;

    float mx = raw_.mx / SCALE;
    float my = raw_.my / SCALE;
    float mz = raw_.mz / SCALE;

    // Hard iron
    mx -= offsetX_;
    my -= offsetY_;
    mz -= offsetZ_;

    // Soft iron
    mx *= scaleX_;
    my *= scaleY_;
    mz *= scaleZ_;

    data_.mx = mx;
    data_.my = my;
    data_.mz = mz;

    return true;
}

void QMC5883::ComputeHeading(float roll_deg, float pitch_deg)
{
    float r = roll_deg  * static_cast<float>(M_PI) / 180.0f;
    float p = pitch_deg * static_cast<float>(M_PI) / 180.0f;

    float Xh = data_.mx * cosf(p) + data_.mz * sinf(p);
    float Yh = data_.mx * sinf(r) * sinf(p)
             + data_.my * cosf(r)
             - data_.mz * sinf(r) * cosf(p);

    float heading = atan2f(Yh, Xh) * 180.0f / static_cast<float>(M_PI);
    if(heading < 0) heading += 360.0f;

    heading += MOUNTING_OFFSET_DEG;
    if(heading >= 360.0f) heading -= 360.0f;

    data_.heading = heading;
}

void QMC5883::SetOffsets(float ox, float oy, float oz)
{
    offsetX_ = ox;
    offsetY_ = oy;
    offsetZ_ = oz;
}

void QMC5883::GetOffsets(float &ox, float &oy, float &oz) const
{
    ox = offsetX_;
    oy = offsetY_;
    oz = offsetZ_;
}

void QMC5883::Calibration(int16_t &min_x, int16_t &max_x,
                                 int16_t &min_y, int16_t &max_y,
                                 int16_t &min_z, int16_t &max_z)
{
    if(!ReadRaw()) return;

    if(raw_.mx < min_x) min_x = raw_.mx;
    if(raw_.mx > max_x) max_x = raw_.mx;
    if(raw_.my < min_y) min_y = raw_.my;
    if(raw_.my > max_y) max_y = raw_.my;
    if(raw_.mz < min_z) min_z = raw_.mz;
    if(raw_.mz > max_z) max_z = raw_.mz;
}


