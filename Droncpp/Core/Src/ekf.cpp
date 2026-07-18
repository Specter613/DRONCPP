/*
 * ekf.cpp
 *
 *  Created on: 17 jul 2026
 *      Author: specter0163
 */

#include "ekf.hpp"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Ekf::Ekf(){
	//Confianza en predicción del gyro
    parametros_.gyroNoise  = 2e-4f; //1e-4f
    //Sube = Menos confía en predicción, corrige más agresivo con sensores
    //Baja = Más confía en gyro entre correcciones

    //Qué tan rápido puede moverse el bias estimado
    parametros_.biasNoise  = 2e-7f; //1e-7f
    //Sube = Bias se re-estima más rápido
    //Baja = Bias casi fijo tras calibración

    //Confianza en el acelerómetro para roll/pitch
    parametros_.accelNoise = 0.05f; //0.05f //0.01
    //Sube = Menos sensible a vibración, pero corrige más lento
    //Baja = Corrige roll/pitch más rápido, pero más sensible a vibración

    //Confianza en el magnetómetro para yaw
    parametros_.yawNoise   = 0.05f; //0.05f //0.01
    //Sube = Menos sensible a distorsión magnética, pero corrige más lento
    //Baja = Corrige yaw más rápido, pero más sensible a distorsión magnética


    memset(q_, 0, sizeof(q_));
    memset(bias_, 0, sizeof(bias_));
    memset(P_, 0, sizeof(P_));

    q_[0] = 1.0f; // cuaternion identidad (sin rotacion)

    for(int i = 0; i < kN; i++)
    {
        P_[i][i] = (i < 4) ? 0.1f : 1e-4f;
    }
}

void Ekf::SetParams(const Parametros &parametros){
    parametros_ = parametros;
}

void Ekf::MatMult(const float *A, int ar, int ac, const float *B, int br, int bc, float *C){
    (void)br;
    for(int i = 0; i < ar; i++)
    {
        for(int j = 0; j < bc; j++)
        {
            float sum = 0.0f;
            for(int k = 0; k < ac; k++)
            {
                sum += A[i*ac + k] * B[k*bc + j];
            }
            C[i*bc + j] = sum;
        }
    }
}

void Ekf::MatTranspose(const float *A, int r, int c, float *At){
    for(int i = 0; i < r; i++)
        for(int j = 0; j < c; j++)
            At[j*r + i] = A[i*c + j];
}


uint8_t Ekf::Mat3Inverse(const float *A, float *Ainv){
    float a=A[0], b=A[1], c=A[2];
    float d=A[3], e=A[4], f=A[5];
    float g=A[6], h=A[7], i=A[8];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if(fabsf(det) < 1e-9f) return 0;
    float invdet = 1.0f / det;

    Ainv[0] =  (e*i - f*h) * invdet;
    Ainv[1] = -(b*i - c*h) * invdet;
    Ainv[2] =  (b*f - c*e) * invdet;
    Ainv[3] = -(d*i - f*g) * invdet;
    Ainv[4] =  (a*i - c*g) * invdet;
    Ainv[5] = -(a*f - c*d) * invdet;
    Ainv[6] =  (d*h - e*g) * invdet;
    Ainv[7] = -(a*h - b*g) * invdet;
    Ainv[8] =  (a*e - b*d) * invdet;
    return 1;
}

void Ekf::QuatNormalize(float *q){
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm < 1e-9f) { q[0]=1; q[1]=q[2]=q[3]=0; return; }
    q[0]/=norm; q[1]/=norm; q[2]/=norm; q[3]/=norm;
}

float Ekf::QuatToYaw(const float *q){
    return atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]),
                  1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
}

float Ekf::WrapPi(float a){
    while(a >  static_cast<float>(M_PI)) a -= 2.0f*static_cast<float>(M_PI);
    while(a < -static_cast<float>(M_PI)) a += 2.0f*static_cast<float>(M_PI);
    return a;
}


void Ekf::Predict(float gx, float gy, float gz, float dt){
    float *q = q_;
    float wx = gx - bias_[0];
    float wy = gy - bias_[1];
    float wz = gz - bias_[2];

    float qDot0 = 0.5f * (-q[1]*wx - q[2]*wy - q[3]*wz);
    float qDot1 = 0.5f * ( q[0]*wx + q[2]*wz - q[3]*wy);
    float qDot2 = 0.5f * ( q[0]*wy - q[1]*wz + q[3]*wx);
    float qDot3 = 0.5f * ( q[0]*wz + q[1]*wy - q[2]*wx);

    q[0] += qDot0 * dt;
    q[1] += qDot1 * dt;
    q[2] += qDot2 * dt;
    q[3] += qDot3 * dt;
    QuatNormalize(q);

    float F[kN][kN];
    memset(F, 0, sizeof(F));

    float Om[4][4] = {
        {  0,  -wx, -wy, -wz },
        {  wx,  0,   wz, -wy },
        {  wy, -wz,  0,   wx },
        {  wz,  wy, -wx,  0  }
    };
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            F[i][j] = (i==j ? 1.0f : 0.0f) + 0.5f*dt*Om[i][j];

    float Xi[4][3] = {
        { -q[1], -q[2], -q[3] },
        {  q[0], -q[3],  q[2] },
        {  q[3],  q[0], -q[1] },
        { -q[2],  q[1],  q[0] }
    };
    for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            F[i][4+j] = -0.5f*dt*Xi[i][j];

    for(int i=4;i<7;i++) F[i][i] = 1.0f;

    float Ft[kN][kN], FP[kN][kN], FPFt[kN][kN];
    MatTranspose((float*)F, kN, kN, (float*)Ft);
    MatMult((float*)F, kN, kN, (float*)P_, kN, kN, (float*)FP);
    MatMult((float*)FP, kN, kN, (float*)Ft, kN, kN, (float*)FPFt);

    for(int i=0;i<kN;i++)
    {
        for(int j=0;j<kN;j++) P_[i][j] = FPFt[i][j];

        float qn = (i < 4) ? parametros_.gyroNoise : parametros_.biasNoise;
        P_[i][i] += qn * dt;
    }
}

void Ekf::UpdateAccel(float ax, float ay, float az){
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 1e-6f) return;
    ax/=norm; ay/=norm; az/=norm;

    float *q = q_;
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];

    float h[3] = {
        2*(q1*q3 - q0*q2),
        2*(q2*q3 + q0*q1),
        q0*q0 - q1*q1 - q2*q2 + q3*q3
    };

    float y[3] = { ax - h[0], ay - h[1], az - h[2] };

    float H[3][kN];
    memset(H, 0, sizeof(H));
    H[0][0]=-2*q2; H[0][1]=2*q3;  H[0][2]=-2*q0; H[0][3]=2*q1;
    H[1][0]=2*q1;  H[1][1]=2*q0;  H[1][2]=2*q3;  H[1][3]=2*q2;
    H[2][0]=2*q0;  H[2][1]=-2*q1; H[2][2]=-2*q2; H[2][3]=2*q3;

    float Ht[kN][3], PHt[kN][3], HPHt[3][3], S[3][3];
    MatTranspose((float*)H, 3, kN, (float*)Ht);
    MatMult((float*)P_, kN, kN, (float*)Ht, kN, 3, (float*)PHt);
    MatMult((float*)H, 3, kN, (float*)PHt, kN, 3, (float*)HPHt);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            S[i][j] = HPHt[i][j] + ((i==j) ? parametros_.accelNoise : 0.0f);

    float Sinv[9];
    if(!Mat3Inverse((float*)S, Sinv)) return;

    float K[kN][3];
    MatMult((float*)PHt, kN, 3, Sinv, 3, 3, (float*)K);

    float dx[kN];
    MatMult((float*)K, kN, 3, y, 3, 1, dx);

    q[0]+=dx[0]; q[1]+=dx[1]; q[2]+=dx[2]; q[3]+=dx[3];
    QuatNormalize(q);
    bias_[0]+=dx[4]; bias_[1]+=dx[5]; bias_[2]+=dx[6];

    float KH[kN][kN], IKH[kN][kN], newP[kN][kN];
    MatMult((float*)K, kN, 3, (float*)H, 3, kN, (float*)KH);
    for(int i=0;i<kN;i++)
        for(int j=0;j<kN;j++)
            IKH[i][j] = ((i==j)?1.0f:0.0f) - KH[i][j];
    MatMult((float*)IKH, kN, kN, (float*)P_, kN, kN, (float*)newP);
    memcpy(P_, newP, sizeof(newP));
}

void Ekf::UpdateYaw(float heading_deg){
    float measured = heading_deg * static_cast<float>(M_PI) / 180.0f;
    float predicted = QuatToYaw(q_);
    float y = WrapPi(measured - predicted);

    float H[kN];
    float eps = 1e-4f;
    float base = predicted;
    float qTmp[4];

    for(int i=0;i<kN;i++)
    {
        if(i<4)
        {
            memcpy(qTmp, q_, sizeof(qTmp));
            qTmp[i] += eps;
            QuatNormalize(qTmp);
            H[i] = WrapPi(QuatToYaw(qTmp) - base) / eps;
        }
        else
        {
            H[i] = 0.0f;
        }
    }

    float PHt[kN];
    for(int i=0;i<kN;i++)
    {
        float sum=0;
        for(int j=0;j<kN;j++) sum += P_[i][j]*H[j];
        PHt[i]=sum;
    }
    float S=0;
    for(int i=0;i<kN;i++) S += H[i]*PHt[i];
    S += parametros_.yawNoise;
    if(fabsf(S) < 1e-9f) return;

    float K[kN];
    for(int i=0;i<kN;i++) K[i] = PHt[i]/S;

    for(int i=0;i<kN;i++)
    {
        if(i<4) q_[i] += K[i]*y;
        else    bias_[i-4] += K[i]*y;
    }
    QuatNormalize(q_);

    float newP[kN][kN];
    for(int i=0;i<kN;i++)
    {
        for(int j=0;j<kN;j++)
        {
            float sum=0;
            for(int k=0;k<kN;k++)
            {
                float IKH_ik = ((i==k)?1.0f:0.0f) - K[i]*H[k];
                sum += IKH_ik * P_[k][j];
            }
            newP[i][j]=sum;
        }
    }
    memcpy(P_, newP, sizeof(newP));
}

void Ekf::GetEuler(float &roll_deg, float &pitch_deg, float &yaw_deg) const {
    const float *q = q_;
    float roll  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
    float pitch = asinf(2*(q[0]*q[2]-q[3]*q[1]));
    float yaw   = QuatToYaw(q);
    if(yaw < 0) yaw += 2.0f*static_cast<float>(M_PI);

    roll_deg  = roll  * 180.0f / static_cast<float>(M_PI);
    pitch_deg = pitch * 180.0f / static_cast<float>(M_PI);
    yaw_deg   = yaw   * 180.0f / static_cast<float>(M_PI);
}
