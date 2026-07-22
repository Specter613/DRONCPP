/*
 * motor_controller.hpp
 *
 *  Created on: 21 jul 2026
 *      Author: specter0163
 */

#ifndef INC_MOTOR_CONTROLLER_HPP_
#define INC_MOTOR_CONTROLLER_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"

class MotorController{
public:
	MotorController(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);

    void Init();

    // Actualiza los 4 motores a partir de throttle/roll/pitch/yaw normalizados
    // (-1.0 a 1.0 para roll/pitch/yaw, 0.0 a 1.0 para throttle) y el estado
    // de armado/failsafe. Esta es la ÚNICA función que debe tocar el mixer.
    void Update(bool armed, bool signalLost,
                    float throttle, float roll, float pitch, float yaw);

     // Fuerza todos los motores a mínimo (1000us) — usado en emergencias
     void Disarm();

     uint16_t GetPwm(uint8_t motorIndex) const { return pwm_[motorIndex]; }
     const uint8_t* GetInitStatus() const { return initStatus_; }   // <-- AGREGAR

 private:
     void SetPwm(TIM_HandleTypeDef *htim, uint8_t channel, uint16_t pulse_us);
     void ApplyMix(float throttle, float roll, float pitch, float yaw);

     static constexpr uint16_t kPwmMin = 1100;
     static constexpr uint16_t kPwmMax = 1900;
     static constexpr uint16_t kPwmDisarmed = 1100;

     TIM_HandleTypeDef *htim3_;
     TIM_HandleTypeDef *htim4_;
     uint16_t pwm_[4] = { kPwmDisarmed, kPwmDisarmed, kPwmDisarmed, kPwmDisarmed };
     bool armed_ = false;
     uint8_t initStatus_[4] = {0, 0, 0, 0};   // <-- AGREGAR
};


#endif /* INC_MOTOR_CONTROLLER_HPP_ */
