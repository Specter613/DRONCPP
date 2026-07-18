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
#include "mtf02.hpp"
#include "gnss.hpp"
#include "crsf.hpp"
#include "ekf.hpp"


enum class TelemetryMode : uint8_t{
	Off= 0,
	Gyro,
	Mag,
	Flow,
	Gps,
	Elrs,
	Ekf,
	_Count
};

struct Muestreo{
	uint32_t tick = 0;
	uint32_t deltaMs = 0;
};

class Telemetry{
public:
	Telemetry(MPU6500 &imu, QMC5883 &mag,MTF01 &flow, Gnss &gps, CRSF &elrs, Ekf &ekf);
	void HandleCommand(const char *cmd);
	void Update();
	void ProcessPendingCalibration();

	void GetMuestreo(TelemetryMode mode, uint32_t deltaMs);

private:
	void PrintGyro();
	void PrintMag();
	void PrintFlow();
	void PrintGps();
	void PrintElrs();
	void PrintEkf();
	void RunMagCalibration();

	MPU6500 &imu_;
	QMC5883 &mag_;
	MTF01 &flow_;
	Gnss &gps_;
	CRSF &elrs_;
	Ekf &ekf_;
	TelemetryMode mode_ = TelemetryMode::Off;

	volatile bool calMagRequested_ = false;
	Muestreo muestreo_[static_cast<uint8_t>(TelemetryMode::_Count)]; // Automaticamente crea un array con el total de enum class TelemetryMode
};

#endif
/*
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

struct SampleInfo{
	uint32_t tick = 0;      // HAL_GetTick() en el momento exacto del muestreo
	uint32_t deltaMs = 0;   // ms transcurridos desde el muestreo anterior de ESE sensor
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

    // Llamado por los trampolines del scheduler cada vez que un sensor
    // se muestrea, para registrar cuándo ocurrió exactamente.
    void MarkSample(TelemetryMode mode, uint32_t deltaMs);

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

    // Índice por TelemetryMode (Off=0 no se usa, así que basta con
    // reservar hasta Ekf).
    SampleInfo sampleInfo_[7];
};

#endif*/
