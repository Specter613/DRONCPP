/*
 * motor_controller.cpp
 *
 *  Created on: 21 jul 2026
 *      Author: specter0163
 */
#include "motor_controller.hpp"

MotorController::MotorController(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
    : htim3_(htim3), htim4_(htim4) {}

void MotorController::Init()
{
    initStatus_[0] = HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_1);
    initStatus_[1] = HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_2);
    initStatus_[2] = HAL_TIM_PWM_Start(htim4_, TIM_CHANNEL_1);
    initStatus_[3] = HAL_TIM_PWM_Start(htim4_, TIM_CHANNEL_2);

    Disarm();
}

void MotorController::SetPwm(TIM_HandleTypeDef *htim, uint8_t channel, uint16_t pulse_us)
{
    if(pulse_us < kPwmMin) pulse_us = kPwmMin;
    if(pulse_us > kPwmMax) pulse_us = kPwmMax;

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

void MotorController::Disarm()
{
    armed_ = false;
    pwm_[0] = pwm_[1] = pwm_[2] = pwm_[3] = kPwmDisarmed;

    SetPwm(htim3_, TIM_CHANNEL_1, kPwmDisarmed);   // PB4
    SetPwm(htim3_, TIM_CHANNEL_2, kPwmDisarmed);   // PB5
    SetPwm(htim4_, TIM_CHANNEL_1, kPwmDisarmed);   // PB6
    SetPwm(htim4_, TIM_CHANNEL_2, kPwmDisarmed);   // PB7
}

void MotorController::ApplyMix(float throttle, float roll, float pitch, float yaw)
{
    float m1 = throttle - roll + pitch + yaw;
    float m2 = throttle + roll + pitch - yaw;
    float m3 = throttle + roll - pitch + yaw;
    float m4 = throttle - roll - pitch - yaw;

    pwm_[0] = static_cast<uint16_t>(kPwmMin + m1 * (kPwmMax - kPwmMin));
    pwm_[1] = static_cast<uint16_t>(kPwmMin + m2 * (kPwmMax - kPwmMin));
    pwm_[2] = static_cast<uint16_t>(kPwmMin + m3 * (kPwmMax - kPwmMin));
    pwm_[3] = static_cast<uint16_t>(kPwmMin + m4 * (kPwmMax - kPwmMin));
}

void MotorController::Update(bool armed, bool signalLost,
                              float throttle, float roll, float pitch, float yaw)
{
    if(signalLost) { Disarm(); return; }
    if(!armed)     { Disarm(); return; }

    armed_ = true;

    if(throttle < 0.0f) throttle = 0.0f;
    if(throttle > 1.0f) throttle = 1.0f;
    if(roll  < -1.0f) roll  = -1.0f; if(roll  > 1.0f) roll  = 1.0f;
    if(pitch < -1.0f) pitch = -1.0f; if(pitch > 1.0f) pitch = 1.0f;
    if(yaw   < -1.0f) yaw   = -1.0f; if(yaw   > 1.0f) yaw   = 1.0f;

    ApplyMix(throttle, roll, pitch, yaw);

    SetPwm(htim3_, TIM_CHANNEL_1, pwm_[0]);   // PB4
    SetPwm(htim3_, TIM_CHANNEL_2, pwm_[1]);   // PB5
    SetPwm(htim4_, TIM_CHANNEL_1, pwm_[2]);   // PB6
    SetPwm(htim4_, TIM_CHANNEL_2, pwm_[3]);   // PB7
}



