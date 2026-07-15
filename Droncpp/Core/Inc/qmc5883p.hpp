/*
 * qmc5883p.hpp
 *
 *  Created on: 14 jul 2026
 *      Author: specter0163
 */

#ifndef INC_QMC5883P_HPP_
#define INC_QMC5883P_HPP_


#include <cstdint>
#include <cmath>
#include "main.h"
#include "mpu6500.hpp"   // necesita MPU6500_Data para tilt-compensation

struct QMC5883Raw
{
    int16_t mx = 0, my = 0, mz = 0;
};

struct QMC5883Data
{
    float mx = 0, my = 0, mz = 0;
    float heading = 0;
};

class QMC5883
{
public:
    explicit QMC5883(I2C_HandleTypeDef *hi2c);

    void Init();
    bool ReadRaw();                              // true si la lectura fue válida
    bool Process();        // lee + calibra + tilt-compensa + heading

    // Calcula heading con tilt-compensation, a partir de roll/pitch externos.
    // Se llama DESPUÉS de Update(), usando los datos ya leídos.
    void ComputeHeading(float roll_deg, float pitch_deg);

    const QMC5883Data& GetData() const { return data_; }
    const QMC5883Raw&  GetRaw()  const { return raw_; }

    // Calibración: solo la matemática, sin I/O de USB ni flash
    void SetOffsets(float ox, float oy, float oz);
    void GetOffsets(float &ox, float &oy, float &oz) const;

    // Muestreo crudo para calibración (min/max), usado por una capa externa
    // que decide cuándo mostrar progreso / guardar en flash
    void Calibration(int16_t &min_x, int16_t &max_x,
                            int16_t &min_y, int16_t &max_y,
                            int16_t &min_z, int16_t &max_z);

private:
    I2C_HandleTypeDef *hi2c_;
    QMC5883Raw  raw_  = {};
    QMC5883Data data_ = {};

    float offsetX_ = 0.0f, offsetY_ = 0.0f, offsetZ_ = 0.0f;
    float scaleX_ = 1.0f, scaleY_ = 1.0f, scaleZ_ = 1.0f;   // soft-iron, reservado a futuro

    static constexpr float SCALE = 1000.0f;   // QMC5883P ±8 Gauss aprox
    static constexpr uint16_t QMC_ADDR  = (0x2C << 1);
    static constexpr uint16_t MPU_ADDR  = (0x68 << 1);   // registros compartidos I2C del breakout
    static constexpr float MOUNTING_OFFSET_DEG = 275.0f;  // offset de montaje, calibrado empíricamente
};

#endif /* INC_QMC5883P_CPP_ */
