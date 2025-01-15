#include <cuda_runtime.h>  // 包含 CUDA 运行时 API


__global__ void rotateVectors(const double* A, const double* R, double* result, int n, int m) {
    // 计算当前线程处理的向量索引
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;

    if (i < n && j < m) {
        // 获取当前 3x1 向量
        double vec[3] = {A[3 * i + 0 * n + j], A[3 * i + 1 * n + j], A[3 * i + 2 * n + j]};

        // 获取当前 3x3 旋转矩阵
        double rot[3][3] = {
            {R[3 * i + 0 * 3 * n + 3 * j], R[3 * i + 0 * 3 * n + 3 * j + 1], R[3 * i + 0 * 3 * n + 3 * j + 2]},
            {R[3 * i + 1 * 3 * n + 3 * j], R[3 * i + 1 * 3 * n + 3 * j + 1], R[3 * i + 1 * 3 * n + 3 * j + 2]},
            {R[3 * i + 2 * 3 * n + 3 * j], R[3 * i + 2 * 3 * n + 3 * j + 1], R[3 * i + 2 * 3 * n + 3 * j + 2]}
        };

        // 计算旋转后的向量
        double rotated_vec[3] = {0, 0, 0};
        for (int k = 0; k < 3; ++k) {
            for (int l = 0; l < 3; ++l) {
                rotated_vec[k] += rot[k][l] * vec[l];
            }
        }

        // 存储结果
        result[3 * i + 0 * n + j] = rotated_vec[0];
        result[3 * i + 1 * n + j] = rotated_vec[1];
        result[3 * i + 2 * n + j] = rotated_vec[2];
    }
}

extern "C" void launchRotateVectors(const double* A, const double* R, double* result, int n, int m, dim3 gridSize, dim3 blockSize) {
    rotateVectors<<<gridSize, blockSize>>>(A, R, result, n, m);
}