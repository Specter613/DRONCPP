/*
 * gnss.hpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#ifndef INC_GNSS_HPP_
#define INC_GNSS_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"

enum class GnssMode : int16_t
{
    Portable = 0, Stationary = 1, Pedestrian = 2, Automotive = 3,
    Airborne1G = 5, Airborne2G = 6, Airborne4G = 7, Wrist = 8, Bike = 9
};

struct GnssData
{
    uint8_t uniqueID[4] = {};

    uint16_t year = 0;
    uint8_t month = 0, day = 0;
    uint8_t hour = 0, min = 0, sec = 0;
    uint8_t fixType = 0;
    uint8_t numSV = 0;
    uint8_t satCount = 0;

    int32_t lon = 0, lat = 0;
    float fLon = 0.0f, fLat = 0.0f;

    int32_t height = 0, hMSL = 0;
    uint32_t hAcc = 0, vAcc = 0;

    int32_t gSpeed = 0, headMot = 0;
};

class Gnss
{
public:
    explicit Gnss(UART_HandleTypeDef *huart);

    void Init();
    void SetMode(GnssMode mode);

    // Alterna internamente entre pedir PVT y NAV-SAT en cada llamada
    void Update();

    void ParseBuffer();

    const GnssData& GetData() const { return data_; }
    uint8_t GetLastTxStatus() const { return lastTxStatus_; }
    uint8_t GetLastRxStatus() const { return lastRxStatus_; }

private:
    void LoadConfig();
    void RequestPVT();
    void RequestNAVSAT();
    void RequestUniqID();
    void RequestNavigatorData();
    void RequestPOSLLH();

    void ParseUniqID();
    void ParsePVT();
    void ParseNavigatorData();
    void ParsePOSLLH();
    void ParseNAVSAT();

    static void ComputeChecksum(const uint8_t *buffer, uint16_t start, uint16_t len,
                                 uint8_t &ckA, uint8_t &ckB);

    UART_HandleTypeDef *huart_;
    uint8_t uartBuffer_[210] = {};
    GnssData data_;

    uint8_t lastTxStatus_ = 0;
    uint8_t lastRxStatus_ = 0;
    bool requestNavSat_ = false;   // alterna PVT/NAV-SAT
};

#endif
