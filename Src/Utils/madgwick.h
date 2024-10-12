
#ifndef MADGWICK_H_
#define MADGWICK_H_

#include <stdbool.h>

void  madgwickSetBaseZAcc(float zacc);
void  madgwickSetZCorrection(float zrot);
void  madgwickUpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void  madgwickGetQuaternion(float* qx, float* qy, float* qz, float* qw);
void  madgwickGetEulerRPY(float* roll, float* pitch, float* yaw);
float madgwickGetAccZWithoutGravity(const float ax, const float ay, const float az);
float madgwickGetAccZ(float ax, float ay, float az);

#endif /* MADGWICK_H_ */
