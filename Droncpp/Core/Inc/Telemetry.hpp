/*
 * Telemetry.hpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#ifndef INC_TELEMETRY_HPP_
#define INC_TELEMETRY_HPP_

#include <cstdint>
#include <cstring>
#include "mpu6500.hpp"
#include "qmc5883p.hpp"
#include "gnss.hpp"
#include "mtf02.hpp"
#include "crsf.hpp"

enum class TelemetryMode : uint8_t
{
    Off = 0,
    Gyro,
    Mag,
    Gps,
    Flow,
    Crsf,
    Ekf,
};

class Telemetry
{
public:
    Telemetry(MPU6500 &imu, QMC5883 &mag, Gnss &gps, MTF01 &flow, CRSF &radio);

    // Procesa un comando de texto (MAG, GYRO, GPS, FLOW, CALMAG, CALSTATUS, ELRS, EKF, x)
    void HandleCommand(const char *cmd);

    // Llamar en el loop principal — imprime según el modo activo
    void Update();
    void ProcessPendingCalibration();

private:
    void PrintGyro();
    void PrintMag();
    void PrintGps();
    void PrintFlow();
    void PrintCrsf();
    void PrintEkf();
    void RunMagCalibration();

    MPU6500 &imu_;
    QMC5883 &mag_;
    Gnss &gps_;
    MTF01 &flow_;
    CRSF &radio_;

    TelemetryMode mode_ = TelemetryMode::Off;
    volatile bool calMagRequested_ = false;
};

#endif
