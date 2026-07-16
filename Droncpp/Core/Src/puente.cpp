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

extern "C" void App_Init(void)
{
    imu.Init();
    imu.CalibrateGyro();
    imu.CalibrateAccel();
    mag.Init();
    gps.Init();
    gps.SetMode(GnssMode::Stationary);
    HAL_Delay(250);
    flow.Init();
    radio.Init(&radioData);
}

extern "C" void App_Loop(void)
{
	  /*imu.Process();
    const MPU6500_Data &d = imu.GetData();

    static char msg[150];
    int len = snprintf(msg, sizeof(msg),"AX:%.2f AY:%.2f AZ:%.2f\r\n"
										  "GX:%.2f GY:%.2f GZ:%.2f \r\n"
  		  	  	  	  	  	  	  	  "Roll:%.2f Pitch:%.2f\r\n",
										  d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.roll, d.pitch);

    CDC_Transmit_FS((uint8_t*)msg, len);
    HAL_Delay(100);
    mag.Process();         // usa el roll/pitch fresco del gyro
    mag.ComputeHeading(imu.GetData().roll, imu.GetData().pitch);   // calcula heading aparte

    const QMC5883Data &d = mag.GetData();

    static char msg[100];
    int len = snprintf(msg, sizeof(msg),
        "MX:%.2f MY:%.2f MZ:%.2f Heading:%.2f\r\n",
        d.mx, d.my, d.mz, d.heading);

    CDC_Transmit_FS((uint8_t*)msg, len);
    HAL_Delay(100);
    static uint32_t lastGpsUpdate = 0;
        if(HAL_GetTick() - lastGpsUpdate >= 200)
        {
            lastGpsUpdate = HAL_GetTick();
            gps.Update();

            const GnssData &d = gps.GetData();
            static char msg[300];
            int len = snprintf(msg, sizeof(msg),
                "GPS\r\n"
                "Day: %d-%d-%d\r\n"
                "Time: %d:%d:%d\r\n"
                "Status of fix: %d\r\n"
                "Sat used: %d  Sat count: %d\r\n"
                "Lat: %f  Lon: %f\r\n"
                "Height ellip.(m): %f  Height MSL(m): %f\r\n"
                "Ground Speed(2D): %ld\r\n",
                d.day, d.month, d.year,
                d.hour, d.min, d.sec,
                d.fixType,
                d.numSV, d.satCount,
                d.fLat, d.fLon,
                static_cast<float>(d.height) / 1000.0f,
                static_cast<float>(d.hMSL) / 1000.0f,
                d.gSpeed);
            CDC_Transmit_FS((uint8_t*)msg, len);
        }
    const MTF01_Data &d = flow.GetData();

    static uint32_t lastFlowUpdate = 0;
    if(HAL_GetTick() - lastFlowUpdate >= 100)
    {
        lastFlowUpdate = HAL_GetTick();

        static char msg[150];
        int len = snprintf(msg, sizeof(msg),
            "FLOW\r\n"
            "Distance: %.2f mm\r\n"
            "FlowX: %.2f  FlowY: %.2f\r\n"
            "Quality: %d  Status: %d\r\n",
            d.distance, d.flowX, d.flowY,
            d.quality, d.status);

        CDC_Transmit_FS((uint8_t*)msg, len);
    }
    radio.Process();

    if(radio.NewFrameAvailable() || true)  // o simplemente lee cuando quieras
    {
        static char msg[200];
        int len = snprintf(msg, sizeof(msg),
            "CRSF status=%d activeChannels=%d\r\n"
            "CH1:%d CH2:%d CH3:%d CH4:%d\r\n",
            radioData.status, radioData.activeChannels,
            radioData.ch[0], radioData.ch[1], radioData.ch[2], radioData.ch[3]);
        CDC_Transmit_FS((uint8_t*)msg, len);
    }*/
}



