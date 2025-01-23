#ifndef UTILS_HPP
#define UTILS_HPP

#include <curand_kernel.h>
#include <cmath>
#include <cuda_runtime.h>

// __device__ void normalize(double* vector) {
//     double norm = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
//     if (norm > 0) {
//         vector[0] /= norm;
//         vector[1] /= norm;
//         vector[2] /= norm;
//     }
// }

__device__ void generate_rotation_matrix(double* RotMat, const double angle, const double3& rot_vector) {
    const double cos_angle = cosf(angle);
    const double sin_angle = sinf(angle);
    const double one_minus_cos = 1.0 - cos_angle;
    const double rx_one_minus_cos = rot_vector.x * one_minus_cos;
    const double ry_one_minus_cos = rot_vector.y * one_minus_cos;
    const double rz_one_minus_cos = rot_vector.z * one_minus_cos;
    const double rx_sin = rot_vector.x * sin_angle;
    const double ry_sin = rot_vector.y * sin_angle;
    const double rz_sin = rot_vector.z * sin_angle;

    RotMat[0] = cos_angle + rot_vector.x * rx_one_minus_cos;
    RotMat[1] = rot_vector.x * ry_one_minus_cos - rz_sin;
    RotMat[2] = rot_vector.x * rz_one_minus_cos + ry_sin;
    RotMat[3] = rot_vector.y * rx_one_minus_cos + rz_sin;
    RotMat[4] = cos_angle + rot_vector.y * ry_one_minus_cos;
    RotMat[5] = rot_vector.y * rz_one_minus_cos - rx_sin;
    RotMat[6] = rot_vector.z * rx_one_minus_cos - ry_sin;
    RotMat[7] = rot_vector.z * ry_one_minus_cos + rx_sin;
    RotMat[8] = cos_angle + rot_vector.z * rz_one_minus_cos;
}

#endif // UTILS_HPP