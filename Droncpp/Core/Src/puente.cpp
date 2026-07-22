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
#include "ekf.hpp"
#include <cmath>
#include "mavlink_link.hpp"
#include "motor_controller.hpp"
#include "usbd_cdc_if.h"//quitar

extern "C" {
#include "Mavlink/common/mavlink.h"
}

extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static MotorController motors(&htim3, &htim4);
static MPU6500 imu(&hspi2);
static QMC5883 mag(&hi2c1);
static MTF01 flow(&huart6);
static Gnss gps(&huart4);
static CRSF_Radio radioData;
static CRSF elrs(&huart2);
static Ekf ekf;

static MavlinkLink mavlink;




static Telemetry telemetry(imu, mag, flow, gps, elrs, ekf);
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

static void EkfTask(void *ctx, uint32_t deltaMs)
{
    constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
    float dt = deltaMs / 1000.0f;

    const MPU6500_Data &imuData = imu.GetData();

    ekf.Predict(imuData.gx * DEG2RAD, imuData.gy * DEG2RAD, imuData.gz * DEG2RAD, dt);
    ekf.UpdateAccel(imuData.ax, imuData.ay, imuData.az);
    ekf.UpdateYaw(mag.GetData().heading);

    telemetry.GetMuestreo(TelemetryMode::Ekf, deltaMs);
}

static void MavlinkHeartbeatTask(void *ctx, uint32_t deltaMs)
{
    mavlink.SendHeartbeat();
}

static void MavlinkAttitudeTask(void *ctx, uint32_t deltaMs)
{
    float roll, pitch, yaw;
    ekf.GetEuler(roll, pitch, yaw);

    const MPU6500_Data &imuData = imu.GetData();

    constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
    mavlink.SendAttitude(
        roll * DEG2RAD, pitch * DEG2RAD, yaw * DEG2RAD,
        imuData.gx * DEG2RAD,   // rollspeed, rad/s
        imuData.gy * DEG2RAD,   // pitchspeed, rad/s
        imuData.gz * DEG2RAD);  // yawspeed, rad/s
}

static void MavlinkPositionTask(void *ctx, uint32_t deltaMs){
    const GnssData &d = gps.GetData();
    mavlink.SendGlobalPosition(d.fLat, d.fLon,
        static_cast<float>(d.hMSL) / 1000.0f,
        static_cast<float>(d.height) / 1000.0f,
        0, 0, 0,   // velocidad NED — pendiente hasta que tengas eso calculado
        0);        // heading — pendiente de conectar mag.GetData().heading
}

static void MavlinkFlowTask(void *ctx, uint32_t deltaMs){
    const MTF01_Data &d = flow.GetData();
    mavlink.SendOpticalFlow(d.distance, d.flowX, d.flowY, d.quality);
}

static void MotorTask(void *ctx, uint32_t deltaMs)
{
    const CRSF_Radio *r =elrs.GetDevice();

    bool signalLost = (r->status == CRSF_SIGNAL_LOST);

    // Canales principales: 1=throttle, 2=roll, 3=pitch, 4=yaw
    float throttle = (r->ch[0] - 172) / (float)(1811 - 172);
    float roll     = (r->ch[1] - 992) / 820.0f;
    float pitch    = (r->ch[2] - 992) / 820.0f;
    float yaw      = (r->ch[3] - 992) / 820.0f;

    // Canal de armado dedicado — CANAL_ARMADO define cuál usar (5 a 16)
    // ch[] es 0-indexado, así que canal 5 físico = índice 4
    constexpr uint8_t CANAL_ARMADO = 4;   // <-- AJUSTA aquí: 4=canal5, 5=canal6, ..., 15=canal16
    bool armed = (r->ch[CANAL_ARMADO] > 1500);

    motors.Update(armed, signalLost, throttle, roll, pitch, yaw);
}

extern "C" void App_Init(void)
{
    // PRUEBA AISLADA DEL LED — antes de cualquier otra cosa
    // PRUEBA DIRECTA: PWM escrito manualmente, sin pasar por MotorController


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
    scheduler.AddTask(EkfTask,  nullptr, 5);

    //scheduler.AddTask(MavlinkHeartbeatTask, nullptr, 1000);   // 1Hz, estándar MAVLink
    //scheduler.AddTask(MavlinkAttitudeTask,  nullptr, 100);    // 10Hz
    //scheduler.AddTask(MavlinkPositionTask,  nullptr, 200);    // 5Hz, alineado con GPS
    // en App_Init()
    //scheduler.AddTask(MavlinkFlowTask, nullptr, 50);   // igual que tu Telemetry, 20Hz
    //scheduler.AddTask(MotorTask, nullptr, 4);

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
