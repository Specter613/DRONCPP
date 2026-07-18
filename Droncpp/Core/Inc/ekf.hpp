/*
 * ekf.hpp
 *
 *  Created on: 17 jul 2026
 *      Author: specter0163
 */

#ifndef INC_EKF_HPP_
#define INC_EKF_HPP_

#include <cstdint>

class Ekf{
public:
	struct Parametros{
	        float gyroNoise;    // ruido de proceso del gyro (rad/s)^2 por segundo
	        float biasNoise;    // random walk del bias (rad/s)^2 por segundo
	        float accelNoise;   // ruido de medicion del accel (g^2)
	        float yawNoise;     // ruido de medicion del yaw (rad^2)
	};

	Ekf();
	void SetParams(const Parametros &parametros);

    // gx,gy,gz en rad/s (SIN restar bias, el filtro lo hace solo). dt en segundos.
    void Predict(float gx, float gy, float gz, float dt);

    // ax,ay,az en g (no hace falta normalizar, el filtro lo hace).
    void UpdateAccel(float ax, float ay, float az);

    // heading_deg: 0-360
    void UpdateYaw(float heading_deg);

    // Angulos de Euler en grados
    void GetEuler(float &roll_deg, float &pitch_deg, float &yaw_deg) const;

private:
    static constexpr int kN = 7;

    // ---- utilidades de matrices (row-major, genericas) ----
    static void MatMult(const float *A, int ar, int ac,
                         const float *B, int br, int bc, float *C);
    static void MatTranspose(const float *A, int r, int c, float *At);
    static uint8_t Mat3Inverse(const float *A, float *Ainv);
    static void QuatNormalize(float *q);
    static float QuatToYaw(const float *q);
    static float WrapPi(float a);

    float q_[4];       // q0(w), q1(x), q2(y), q3(z)
    float bias_[3];     // bias del gyro, rad/s
    float P_[kN][kN];    // covarianza del estado

    Parametros parametros_;
};



#endif /* INC_EKF_HPP_ */
