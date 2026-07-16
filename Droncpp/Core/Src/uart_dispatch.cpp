/*
 * uart_dispatch.cpp
 *
 *  Created on: 15 jul 2026
 *      Author: specter0163
 */

#include "uart_dispatch.hpp"

UartDispatch::Registration UartDispatch::registrations_[UartDispatch::kMaxHandlers];
uint8_t UartDispatch::registrationCount_ = 0;

UartDispatch::Registration UartDispatch::halfRegistrations_[UartDispatch::kMaxHandlers];
uint8_t UartDispatch::halfRegistrationCount_ = 0;

void UartDispatch::Register(UART_HandleTypeDef *huart, UART_RxHandler handler)
{
    if(registrationCount_ < kMaxHandlers)
    {
        registrations_[registrationCount_].huart = huart;
        registrations_[registrationCount_].handler = handler;
        registrationCount_++;
    }
}

void UartDispatch::RegisterHalf(UART_HandleTypeDef *huart, UART_RxHandler handler)
{
    if(halfRegistrationCount_ < kMaxHandlers)
    {
        halfRegistrations_[halfRegistrationCount_].huart = huart;
        halfRegistrations_[halfRegistrationCount_].handler = handler;
        halfRegistrationCount_++;
    }
}

void UartDispatch::HandleRxComplete(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0; i < registrationCount_; i++)
    {
        if(registrations_[i].huart == huart)
        {
            registrations_[i].handler(huart);
            return;
        }
    }
}

void UartDispatch::HandleRxHalfComplete(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0; i < halfRegistrationCount_; i++)
    {
        if(halfRegistrations_[i].huart == huart)
        {
            halfRegistrations_[i].handler(huart);
            return;
        }
    }
}

// El HAL (compilado como C, en stm32f4xx_hal_uart.c) llama a estas
// funciones por su nombre exacto — deben existir con linkage C.
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UartDispatch::HandleRxComplete(huart);
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    UartDispatch::HandleRxHalfComplete(huart);
}


