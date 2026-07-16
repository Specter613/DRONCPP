/*
 * mtf03.cpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#include "mtf02.hpp"

MTF01 *MTF01::activeInstance = nullptr;

void MTF01::Init()
{
    activeInstance = this;

    UartDispatch::Register(huart_, MTF01_RxHandler);
    HAL_UART_Receive_IT(huart_, &mtf01_rx_byte, 1);
}

void MTF01::MTF01_RxHandler(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr)
    {
        activeInstance->MTF01_ProcessByte(activeInstance->mtf01_rx_byte);
        HAL_UART_Receive_IT(huart, &activeInstance->mtf01_rx_byte, 1);
    }
}

bool MTF01::micolink_check_sum(MICOLINK_MSG_t *msg)
{
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;
    uint8_t length = msg->len + 6;

    memcpy(temp, msg, length);

    for(uint8_t i = 0; i < length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
        return true;

    return false;
}

bool MTF01::micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data)
{
    switch(msg->status)
    {
        case 0:

            if(data == MICOLINK_MSG_HEAD)
            {
                msg->head = data;
                msg->status = 1;
            }

            break;

        case 1:

            msg->dev_id = data;
            msg->status++;
            break;

        case 2:

            msg->sys_id = data;
            msg->status++;
            break;

        case 3:

            msg->msg_id = data;
            msg->status++;
            break;

        case 4:

            msg->seq = data;
            msg->status++;
            break;

        case 5:

            msg->len = data;

            if(msg->len == 0)
                msg->status += 2;

            else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            {
                msg->status = 0;
                msg->payload_cnt = 0;
            }

            else
                msg->status++;

            break;

        case 6:

            msg->payload[msg->payload_cnt++] = data;

            if(msg->payload_cnt == msg->len)
            {
                msg->payload_cnt = 0;
                msg->status++;
            }

            break;

        case 7:

            msg->checksum = data;
            msg->status = 0;

            if(micolink_check_sum(msg))
                return true;

            break;

        default:

            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }

    return false;
}

void MTF01::MTF01_ProcessByte(uint8_t byte)
{
    if(micolink_parse_char(&msg, byte))
    {
        if(msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR)
        {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;

            memcpy(&payload, msg.payload, sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t));

            mtf.distance = payload.distance;
            mtf.flowX = payload.flow_vel_x;
            mtf.flowY = payload.flow_vel_y;
            mtf.quality = payload.flow_quality;
            mtf.status = payload.dis_status;
        }
    }
}

void MTF01::MTF01_GetData(MTF01_Data *data)
{
    memcpy(data, &mtf, sizeof(MTF01_Data));
}


