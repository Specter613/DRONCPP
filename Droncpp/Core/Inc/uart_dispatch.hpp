/*
 * uart_dispatch.hpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#ifndef INC_UART_DISPATCH_HPP_
#define INC_UART_DISPATCH_HPP_

#include "stm32f4xx_hal.h"

typedef void (*UART_RxHandler)(UART_HandleTypeDef *huart);

class UartDispatch
{
public:
    static void Register(UART_HandleTypeDef *huart, UART_RxHandler handler);
    static void RegisterHalf(UART_HandleTypeDef *huart, UART_RxHandler handler);

    // Los callbacks HAL reales llaman a estas — deben tener linkage C
    // porque stm32f4xx_hal_uart.c (compilado como C) los invoca por nombre.
    static void HandleRxComplete(UART_HandleTypeDef *huart);
    static void HandleRxHalfComplete(UART_HandleTypeDef *huart);

private:
    struct Registration
    {
        UART_HandleTypeDef *huart = nullptr;
        UART_RxHandler handler = nullptr;
    };

    static constexpr uint8_t kMaxHandlers = 4;

    static Registration registrations_[kMaxHandlers];
    static uint8_t registrationCount_;

    static Registration halfRegistrations_[kMaxHandlers];
    static uint8_t halfRegistrationCount_;
};

#endif
