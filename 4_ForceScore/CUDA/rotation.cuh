#ifndef ROTATION_CUH
#define ROTATION_CUH

#include <cuda_runtime.h>
#include <Eigen/Dense>

__global__ void random_rotation_kernel(Eigen::Vector3f* vectors, int count, double angle_max, Eigen::Vector3f ref_vector);

void generate_random_rotations(Eigen::Vector3f* vectors, int count, double angle_max, Eigen::Vector3f ref_vector);

#endif // ROTATION_CUH