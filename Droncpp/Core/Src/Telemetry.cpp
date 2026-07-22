/*
 * Telemetry.cpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#include "Telemetry.hpp"
#include "usbd_cdc_if.h"
#include <cstdio>
#include "CompassStorage.hpp"

Telemetry::Telemetry(MPU6500 &imu, QMC5883 &mag, MTF01 &flow, Gnss &gps, CRSF &elrs, Ekf &ekf)
		: imu_(imu), mag_(mag), flow_(flow), gps_(gps), elrs_(elrs), ekf_(ekf){}

void Telemetry::HandleCommand(const char *cmd){
	if(strcmp(cmd,"x") == 0){
		mode_ = TelemetryMode::Off;
		static const char msg[] = "Lectura detenida \r\n"
								  "Escriba GYRO-MAG-GPS-FLOW-CRSF-CALMAG-CALSTATUS-EKF\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
	else if(strcmp(cmd,"GYRO") == 0) mode_ = TelemetryMode::Gyro;
	else if(strcmp(cmd, "MAG") == 0)    mode_ = TelemetryMode::Mag;
	else if(strcmp(cmd, "FLOW") == 0)    mode_ = TelemetryMode::Flow;
	else if(strcmp(cmd,"GPS") == 0) mode_ = TelemetryMode::Gps;
	else if(strcmp(cmd,"ELRS") == 0) mode_ =TelemetryMode::Elrs;
	else if(strcmp(cmd,"EKF") == 0) mode_ = TelemetryMode::Ekf;
    else if(strcmp(cmd, "CALMAG") == 0)
    {
        calMagRequested_ = true;
    }
    else if(strcmp(cmd, "CALSTATUS") == 0)
    {
        float ox, oy, oz;
        static char msg[100];
        int len;
        if(MagCalibrationStorage::Load(ox, oy, oz))
            len = snprintf(msg, sizeof(msg), "Calibracion presente: X=%.2f Y=%.2f Z=%.2f\r\n", ox, oy, oz);
        else
            len = snprintf(msg, sizeof(msg), "Sin calibracion guardada\r\n");
        CDC_Transmit_FS((uint8_t*)msg, len);
    }
}

void Telemetry::Update(){
	switch(mode_){
	case TelemetryMode::Gyro: PrintGyro(); break;
	case TelemetryMode::Mag:  PrintMag();  break;
	case TelemetryMode::Flow:  PrintFlow();  break;
	case TelemetryMode::Gps: PrintGps(); break;
	case TelemetryMode::Elrs: PrintElrs(); break;
	case TelemetryMode::Ekf:  PrintEkf();  break;
	case TelemetryMode::Off:  default: break;
	}
}

void Telemetry::GetMuestreo(TelemetryMode mode, uint32_t deltaMs){
	uint8_t idx = static_cast<uint8_t>(mode);
    muestreo_[idx].tick = HAL_GetTick();
    muestreo_[idx].deltaMs = deltaMs;
}


void Telemetry::PrintGyro()
{
    const MPU6500_Data &d = imu_.GetData();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(TelemetryMode::Gyro)];

    static char msg[200];
    int len = snprintf(msg, sizeof(msg),
        "GYRO\r\nAX:%.2f AY:%.2f AZ:%.2f\r\nGX:%.2f GY:%.2f GZ:%.2f\r\nRoll:%.2f Pitch:%.2f\r\n"
        "tick:%lu dt:%lums\r\n",
        d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.roll, d.pitch,
        s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintMag()
{
    const QMC5883Data &d = mag_.GetData();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(TelemetryMode::Mag)];

    static char msg[150];
    int len = snprintf(msg, sizeof(msg),
        "MAG\r\nMX:%.2f MY:%.2f MZ:%.2f Heading:%.2f\r\ntick:%lu dt:%lums\r\n",
        d.mx, d.my, d.mz, d.heading, s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintFlow()
{
    const MTF01_Data &d = flow_.GetData();
    // FLOW se actualiza por interrupción, de forma asíncrona — no pasa
    // por el scheduler, así que aquí "tick" es el momento de la CONSULTA,
    // no necesariamente el momento exacto del último byte recibido.
    static char msg[180];
    int len = snprintf(msg, sizeof(msg),
        "FLOW\r\nDistance: %.2f mm\r\nFlowX: %.2f FlowY: %.2f\r\nQuality: %d Status: %d\r\n"
        "consultado en tick:%lu\r\n",
        d.distance, d.flowX, d.flowY, d.quality, d.status, HAL_GetTick());
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintGps()
{
    const GnssData &d = gps_.GetData();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(TelemetryMode::Gps)];

    static char msg[350];
    int len = snprintf(msg, sizeof(msg),
        "GPS\r\nDay: %d-%d-%d\r\nTime: %d:%d:%d\r\n"
        "Status of fix: %d\r\nSat used: %d Sat count: %d\r\n"
        "Lat: %f Lon: %f\r\nHeight ellip.(m): %.2f Height MSL(m): %.2f\r\n"
        "Ground Speed(2D): %ld\r\ntick:%lu dt:%lums\r\n",
        d.day, d.month, d.year, d.hour, d.min, d.sec,
        d.fixType, d.numSV, d.satCount, d.fLat, d.fLon,
        static_cast<float>(d.height)/1000.0f, static_cast<float>(d.hMSL)/1000.0f,
        d.gSpeed, s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintElrs()
{
    const CRSF_Radio *d = elrs_.GetDevice();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(TelemetryMode::Elrs)];

    static char msg[220];
    int len = snprintf(msg, sizeof(msg),
        "ELRS\r\nStatus: %d ActiveChannels: %d\r\n"
        "CH1:%d CH2:%d CH3:%d CH4:%d CH5:%d\r\ntick:%lu dt:%lums\r\n",
        d->status, d->activeChannels, d->ch[0], d->ch[1], d->ch[2], d->ch[3], d->ch[4],
        s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintEkf()
{
    float roll, pitch, yaw;
    ekf_.GetEuler(roll, pitch, yaw);
    const Muestreo &s = muestreo_[static_cast<uint8_t>(TelemetryMode::Ekf)];

    static char msg[150];
    int len = snprintf(msg, sizeof(msg),
        "EKF\r\nRoll:%.2f Pitch:%.2f Yaw:%.2f\r\ntick:%lu dt:%lums\r\n",
        roll, pitch, yaw, s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::ProcessPendingCalibration()
{
    if(calMagRequested_)
    {
        calMagRequested_ = false;
        RunMagCalibration();
    }
}

void Telemetry::RunMagCalibration()
{
    int16_t minX = 32767, maxX = -32768;
    int16_t minY = 32767, maxY = -32768;
    int16_t minZ = 32767, maxZ = -32768;

    const uint16_t samples = 2000;
    uint16_t count = 0;

    static char msg[100];
    int len = snprintf(msg, sizeof(msg), "CALIBRACION MAG: gira la placa ahora (%d muestras)\r\n", samples);
    CDC_Transmit_FS((uint8_t*)msg, len);

    while(count < samples)
    {
        mag_.Calibration(minX, maxX, minY, maxY, minZ, maxZ);
        count++;

        if(count % (samples / 10) == 0)
        {
            len = snprintf(msg, sizeof(msg), "Progreso: %d%%\r\n", (count * 100) / samples);
            CDC_Transmit_FS((uint8_t*)msg, len);
        }
        HAL_Delay(5);
    }

    float ox = (maxX + minX) / 2.0f / 1000.0f;
    float oy = (maxY + minY) / 2.0f / 1000.0f;
    float oz = (maxZ + minZ) / 2.0f / 1000.0f;

    mag_.SetOffsets(ox, oy, oz);
    MagCalibrationStorage::Save(ox, oy, oz);

    len = snprintf(msg, sizeof(msg), "CALIBRACION completa. Offsets: X=%.2f Y=%.2f Z=%.2f\r\n", ox, oy, oz);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

/*
#include "Telemetry.hpp"
#include "usbd_cdc_if.h"
#include <cstdio>
#include "CompassStorage.hpp"

Telemetry::Telemetry(MPU6500 &imu, QMC5883 &mag, Gnss &gps, MTF01 &flow, CRSF &radio)
    : imu_(imu), mag_(mag), gps_(gps), flow_(flow), radio_(radio) {}

void Telemetry::HandleCommand(const char *cmd)
{
    if(strcmp(cmd, "x") == 0)
    {
        mode_ = TelemetryMode::Off;
        static const char msg[] = "Lectura detenida\r\nEscriba: GYRO MAG GPS FLOW ELRS EKF\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }
    else if(strcmp(cmd, "GYRO") == 0)   mode_ = TelemetryMode::Gyro;
    else if(strcmp(cmd, "MAG") == 0)    mode_ = TelemetryMode::Mag;
    else if(strcmp(cmd, "GPS") == 0)    mode_ = TelemetryMode::Gps;
    else if(strcmp(cmd, "FLOW") == 0)   mode_ = TelemetryMode::Flow;
    else if(strcmp(cmd, "ELRS") == 0)   mode_ = TelemetryMode::Crsf;
    else if(strcmp(cmd, "EKF") == 0)    mode_ = TelemetryMode::Ekf;
    else if(strcmp(cmd, "CALMAG") == 0)
    {
        calMagRequested_ = true;
    }
    else if(strcmp(cmd, "CALSTATUS") == 0)
    {
        float ox, oy, oz;
        static char msg[100];
        int len;
        if(MagCalibrationStorage::Load(ox, oy, oz))
            len = snprintf(msg, sizeof(msg), "Calibracion presente: X=%.2f Y=%.2f Z=%.2f\r\n", ox, oy, oz);
        else
            len = snprintf(msg, sizeof(msg), "Sin calibracion guardada\r\n");
        CDC_Transmit_FS((uint8_t*)msg, len);
    }
}

void Telemetry::Update()
{
    switch(mode_)
    {
        case TelemetryMode::Gyro: PrintGyro(); break;
        case TelemetryMode::Mag:  PrintMag();  break;
        case TelemetryMode::Gps:  PrintGps();  break;
        case TelemetryMode::Flow: PrintFlow(); break;
        case TelemetryMode::Crsf: PrintCrsf(); break;
        case TelemetryMode::Ekf:  PrintEkf();  break;
        case TelemetryMode::Off:  default: break;
    }
}

void Telemetry::MarkSample(TelemetryMode mode, uint32_t deltaMs)
{
    uint8_t idx = static_cast<uint8_t>(mode);
    sampleInfo_[idx].tick = HAL_GetTick();
    sampleInfo_[idx].deltaMs = deltaMs;
}

void Telemetry::PrintGyro()
{
    const MPU6500_Data &d = imu_.GetData();
    const SampleInfo &s = sampleInfo_[static_cast<uint8_t>(TelemetryMode::Gyro)];

    static char msg[200];
    int len = snprintf(msg, sizeof(msg),
        "GYRO\r\nAX:%.2f AY:%.2f AZ:%.2f\r\nGX:%.2f GY:%.2f GZ:%.2f\r\nRoll:%.2f Pitch:%.2f\r\n"
        "tick:%lu dt:%lums\r\n",
        d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.roll, d.pitch,
        s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintMag()
{
    const QMC5883Data &d = mag_.GetData();
    const SampleInfo &s = sampleInfo_[static_cast<uint8_t>(TelemetryMode::Mag)];

    static char msg[150];
    int len = snprintf(msg, sizeof(msg),
        "MAG\r\nMX:%.2f MY:%.2f MZ:%.2f Heading:%.2f\r\ntick:%lu dt:%lums\r\n",
        d.mx, d.my, d.mz, d.heading, s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintGps()
{
    const GnssData &d = gps_.GetData();
    const SampleInfo &s = sampleInfo_[static_cast<uint8_t>(TelemetryMode::Gps)];

    static char msg[350];
    int len = snprintf(msg, sizeof(msg),
        "GPS\r\nDay: %d-%d-%d\r\nTime: %d:%d:%d\r\n"
        "Status of fix: %d\r\nSat used: %d Sat count: %d\r\n"
        "Lat: %f Lon: %f\r\nHeight ellip.(m): %.2f Height MSL(m): %.2f\r\n"
        "Ground Speed(2D): %ld\r\ntick:%lu dt:%lums\r\n",
        d.day, d.month, d.year, d.hour, d.min, d.sec,
        d.fixType, d.numSV, d.satCount, d.fLat, d.fLon,
        static_cast<float>(d.height)/1000.0f, static_cast<float>(d.hMSL)/1000.0f,
        d.gSpeed, s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintFlow()
{
    const MTF01_Data &d = flow_.GetData();
    // FLOW se actualiza por interrupción, de forma asíncrona — no pasa
    // por el scheduler, así que aquí "tick" es el momento de la CONSULTA,
    // no necesariamente el momento exacto del último byte recibido.
    static char msg[180];
    int len = snprintf(msg, sizeof(msg),
        "FLOW\r\nDistance: %.2f mm\r\nFlowX: %.2f FlowY: %.2f\r\nQuality: %d Status: %d\r\n"
        "consultado en tick:%lu\r\n",
        d.distance, d.flowX, d.flowY, d.quality, d.status, HAL_GetTick());
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintCrsf()
{
    const CRSF_Radio *d = radio_.GetDevice();
    const SampleInfo &s = sampleInfo_[static_cast<uint8_t>(TelemetryMode::Crsf)];

    static char msg[220];
    int len = snprintf(msg, sizeof(msg),
        "ELRS\r\nStatus: %d ActiveChannels: %d\r\n"
        "CH1:%d CH2:%d CH3:%d CH4:%d\r\ntick:%lu dt:%lums\r\n",
        d->status, d->activeChannels, d->ch[0], d->ch[1], d->ch[2], d->ch[3],
        s.tick, s.deltaMs);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetry::PrintEkf()
{
    // Placeholder hasta que EkfFeed esté terminado
    static char msg[] = "EKF\r\n(pendiente de integrar)\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void Telemetry::ProcessPendingCalibration()
{
    if(calMagRequested_)
    {
        calMagRequested_ = false;
        RunMagCalibration();
    }
}

void Telemetry::RunMagCalibration()
{
    int16_t minX = 32767, maxX = -32768;
    int16_t minY = 32767, maxY = -32768;
    int16_t minZ = 32767, maxZ = -32768;

    const uint16_t samples = 500;
    uint16_t count = 0;

    static char msg[100];
    int len = snprintf(msg, sizeof(msg), "CALIBRACION MAG: gira la placa ahora (%d muestras)\r\n", samples);
    CDC_Transmit_FS((uint8_t*)msg, len);

    while(count < samples)
    {
        mag_.Calibration(minX, maxX, minY, maxY, minZ, maxZ);
        count++;

        if(count % (samples / 10) == 0)
        {
            len = snprintf(msg, sizeof(msg), "Progreso: %d%%\r\n", (count * 100) / samples);
            CDC_Transmit_FS((uint8_t*)msg, len);
        }
        HAL_Delay(5);
    }

    float ox = (maxX + minX) / 2.0f / 1000.0f;
    float oy = (maxY + minY) / 2.0f / 1000.0f;
    float oz = (maxZ + minZ) / 2.0f / 1000.0f;

    mag_.SetOffsets(ox, oy, oz);
    MagCalibrationStorage::Save(ox, oy, oz);

    len = snprintf(msg, sizeof(msg), "CALIBRACION completa. Offsets: X=%.2f Y=%.2f Z=%.2f\r\n", ox, oy, oz);
    CDC_Transmit_FS((uint8_t*)msg, len);
}*/

