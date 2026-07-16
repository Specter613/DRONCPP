/*
 * puente.c
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */
#include "puente.h"
#include "mpu6500.hpp"
#include "qmc5883p.hpp"
#include "gnss.hpp"
#include "crsf.hpp"
#include "mtf02.hpp"
#include "main.h"
#include "Telemetry.hpp"
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
}

extern "C" void App_Loop(void)
{
    imu.Process();
    mag.Process();
    mag.ComputeHeading(imu.GetData().roll, imu.GetData().pitch);
    radio.Process();

    static uint32_t lastGpsUpdate = 0;
    if(HAL_GetTick() - lastGpsUpdate >= 200)
    {
        lastGpsUpdate = HAL_GetTick();
        gps.Update();
    }

    telemetry.ProcessPendingCalibration();   // <-- agregar
    telemetry.Update();
}

extern "C" void App_HandleCommand(char *cmd)
{
    cmd[strcspn(cmd, "\r\n")] = 0;
    telemetry.HandleCommand(cmd);
}

