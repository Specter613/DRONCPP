/*
 * mtf02.hpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#ifndef INC_MTF02_HPP_
#define INC_MTF02_HPP_

#include <cstdint>
#include <cstring>
#include "stm32f4xx_hal.h"
#include "uart_dispatch.hpp"

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             (MICOLINK_MAX_PAYLOAD_LEN + 7)

#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51

typedef struct
{
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;

} MICOLINK_MSG_t;

#pragma pack(push,1)
typedef struct
{
    uint32_t time_ms;
    uint32_t distance;
    uint8_t strength;
    uint8_t precision;
    uint8_t dis_status;
    uint8_t reserved1;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved2;

} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)

typedef struct
{
    float distance;
    float flowX;
    float flowY;
    uint8_t quality;
    uint8_t status;

} MTF01_Data;

class MTF01
{
public:
    explicit MTF01(UART_HandleTypeDef *huart) : huart_(huart) {}

    void Init();   // ya no recibe huart, lo toma de huart_

    bool micolink_check_sum(MICOLINK_MSG_t *msg);
    bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);
    void MTF01_ProcessByte(uint8_t byte);
    void MTF01_GetData(MTF01_Data *data);

    const MTF01_Data& GetData() const { return mtf; }
private:
    static void MTF01_RxHandler(UART_HandleTypeDef *huart);

    UART_HandleTypeDef *huart_;
    uint8_t mtf01_rx_byte = 0;
    MICOLINK_MSG_t msg = {};
    MTF01_Data mtf = {};

    static MTF01 *activeInstance;
};

#endif
