/*
 * puente.c
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */
#include "puente.h"
#include "main.h"
#include "Telemetry.hpp"
#include "scheduler.hpp"
#include "mpu6500.hpp"
#include "qmc5883p.hpp"
#include "mtf02.hpp"
#include "gnss.hpp"
#include <cstring>
#include "CompassStorage.hpp"

extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

static MPU6500 imu(&hspi2);
static QMC5883 mag(&hi2c1);
static MTF01 flow(&huart6);
static Gnss gps(&huart4);
static CRSF_Radio radioData;
static CRSF elrs(&huart2);



static Telemetry telemetry(imu, mag, flow, gps, elrs);
static Scheduler scheduler;

static void GyroTask(void *ctx, uint32_t deltaMs)
{
    imu.Process();
    telemetry.GetMuestreo(TelemetryMode::Gyro, deltaMs);
}

static void MagTask(void *ctx, uint32_t deltaMs)
{
    mag.Process();
    mag.ComputeHeading(imu.GetData().roll, imu.GetData().pitch);
    telemetry.GetMuestreo(TelemetryMode::Mag, deltaMs);
}

static void GpsTask(void *ctx, uint32_t deltaMs)
{
    gps.Update();
    telemetry.GetMuestreo(TelemetryMode::Gps, deltaMs);
}

static void ElrsTask(void *ctx, uint32_t deltaMs)
{
    elrs.Process();
    telemetry.GetMuestreo(TelemetryMode::Elrs, deltaMs);
}

extern "C" void App_Init(void)
{
    imu.Init();
    imu.CalibrateGyro();
    imu.CalibrateAccel();
    mag.Init();
    float ox, oy, oz;
    if(MagCalibrationStorage::Load(ox, oy, oz))
    {
        mag.SetOffsets(ox, oy, oz);
    }
    flow.Init();
    gps.Init();
    gps.SetMode(GnssMode::Stationary);
    HAL_Delay(250);
    elrs.Init(&radioData);


    // Registro de tareas periódicas — mismos periodos que ya usabas
    scheduler.AddTask(GyroTask, nullptr, 5);
    scheduler.AddTask(MagTask,  nullptr, 5);
    scheduler.AddTask(GpsTask, nullptr, 200);
    scheduler.AddTask(ElrsTask, nullptr, 2);
}

extern "C" void App_Loop(void)
{
    scheduler.Run();
    telemetry.ProcessPendingCalibration();
    telemetry.Update();
}

extern "C" void App_HandleCommand(char *cmd)
{
    cmd[strcspn(cmd, "\r\n")] = 0;
    telemetry.HandleCommand(cmd);
}
/*
#include "puente.h"
#include "mpu6500.hpp"
#include "qmc5883p.hpp"
#include "gnss.hpp"
#include "crsf.hpp"
#include "mtf02.hpp"
#include "main.h"
#include "Telemetry.hpp"
#include "scheduler.hpp"
#include <cstring>
#include "CompassStorage.hpp"

extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

static MPU6500 imu(&hspi2);
static QMC5883 mag(&hi2c1);
static Gnss gps(&huart4);
static MTF01 flow(&huart6);
static CRSF_Radio radioData;
static CRSF radio(&huart2);

static Telemetry telemetry(imu, mag, gps, flow, radio);
static Scheduler scheduler;

// --- Trampolines: cada uno hace el trabajo real del sensor y marca
// el momento de muestreo en Telemetry ---

static void GyroTask(void *ctx, uint32_t deltaMs)
{
    imu.Process();
    telemetry.MarkSample(TelemetryMode::Gyro, deltaMs);
}

static void MagTask(void *ctx, uint32_t deltaMs)
{
    mag.Process();
    mag.ComputeHeading(imu.GetData().roll, imu.GetData().pitch);
    telemetry.MarkSample(TelemetryMode::Mag, deltaMs);
}

static void GpsTask(void *ctx, uint32_t deltaMs)
{
    gps.Update();
    telemetry.MarkSample(TelemetryMode::Gps, deltaMs);
}

static void CrsfTask(void *ctx, uint32_t deltaMs)
{
    radio.Process();
    telemetry.MarkSample(TelemetryMode::Crsf, deltaMs);
}

extern "C" void App_Init(void)
{
    imu.Init();
    imu.CalibrateGyro();
    imu.CalibrateAccel();
    mag.Init();

    float ox, oy, oz;
    if(MagCalibrationStorage::Load(ox, oy, oz))
    {
        mag.SetOffsets(ox, oy, oz);
    }

    gps.Init();
    gps.SetMode(GnssMode::Stationary);
    HAL_Delay(250);
    flow.Init();
    radio.Init(&radioData);

    // Registro de tareas periódicas — mismos periodos que ya usabas
    scheduler.AddTask(GyroTask, nullptr, 5);
    scheduler.AddTask(MagTask,  nullptr, 5);
    scheduler.AddTask(GpsTask,  nullptr, 200);
    scheduler.AddTask(CrsfTask, nullptr, 4);
}

extern "C" void App_Loop(void)
{
    scheduler.Run();

    telemetry.ProcessPendingCalibration();
    telemetry.Update();
}

extern "C" void App_HandleCommand(char *cmd)
{
    cmd[strcspn(cmd, "\r\n")] = 0;
    telemetry.HandleCommand(cmd);
}
*/
