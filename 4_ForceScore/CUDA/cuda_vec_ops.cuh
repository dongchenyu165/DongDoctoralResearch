#include <cassert>
#include <cmath>
#include <cstddef>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <vector_types.h>





__device__ __host__ inline float3 make_vector(const float InX, const float InY, const float InZ)
{
	return make_float3(InX, InY, InZ);
}
__device__ __host__ inline double3 make_vector(const double InX, const double InY, const double InZ)
{
	return make_double3(InX, InY, InZ);
}


template <typename VecType>
__device__ __host__ inline VecType add(const VecType& a, const VecType& b)
{
	return make_vector(a.x + b.x, a.y + b.y, a.z + b.z);
}
template <typename Scalar>
__device__ __host__ inline void add(const Scalar* a, const Scalar* b, Scalar* result, const int size)
{
	for ( int i = 0; i < size; i++ )
	{
		result[i] = a[i] + b[i];
	}
}
template <typename Scalar>
__device__ __host__ inline void add(const Scalar* a, const Scalar b, Scalar* result, const int size)
{
	for ( int i = 0; i < size; i++ )
	{
		result[i] = a[i] + b;
	}
}
// // Vector addition (float3)
// __device__ __host__ inline float3 add(const float3& a, const float3& b)
// {
// 	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
// }

// // Vector addition (double3)
// __device__ __host__ inline double3 add(const double3& a, const double3& b)
// {
// 	return make_double3(a.x + b.x, a.y + b.y, a.z + b.z);
// }





// Vector subtraction (float3)
__device__ __host__ inline float3 subtract(const float3& a, const float3& b)
{
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Vector subtraction (float3)
__device__ __host__ inline float3 subtract(const float3& a, const float& scalar)
{
	return make_float3(a.x - scalar, a.y - scalar, a.z - scalar);
}

// Vector subtraction (double3)
__device__ __host__ inline double3 subtract(const double3& a, const double3& b)
{
	return make_double3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Vector subtraction (double3)
__device__ __host__ inline double3 subtract(const double3& a, const double& scalar)
{
	return make_double3(a.x - scalar, a.y - scalar, a.z - scalar);
}

// nD float vector subtraction
__device__ __host__ inline void subtract(const float* a, const float* b, float* result, const int size)
{
	for ( int i = 0; i < size; i++ )
	{
		result[i] = a[i] - b[i];
	}
}
// nD vector subtraction
__device__ __host__ inline void subtract(const double* a, const double* b, double* result, const int size)
{
	for ( int i = 0; i < size; i++ )
	{
		result[i] = a[i] - b[i];
	}
}

// Vector multiplication (component-wise, float3)
__device__ __host__ inline float3 multiply(const float3& a, const float3& b)
{
	return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

// Vector multiplication (component-wise, double3)
__device__ __host__ inline double3 multiply(const double3& a, const double3& b)
{
	return make_double3(a.x * b.x, a.y * b.y, a.z * b.z);
}

// Scalar multiplication (float3)
__device__ __host__ inline float3 multiply(const float3& a, float scalar)
{
	return make_float3(a.x * scalar, a.y * scalar, a.z * scalar);
}

// Scalar multiplication (double3)
__device__ __host__ inline double3 multiply(const double3& a, double scalar)
{
	return make_double3(a.x * scalar, a.y * scalar, a.z * scalar);
}
// nD  Float Vector scalar multiplication
__device__ __host__ inline void multiply(const float* a, const float scalar, float* result, const int size)
{
	for ( int i = 0; i < size; i++ )
	{
		result[i] = a[i] * scalar;
	}
}
// nD Double Vector scalar multiplication
__device__ __host__ inline void multiply(const double* a, const double scalar, double* result, const int size)
{
	for ( int i = 0; i < size; i++ )
	{
		result[i] = a[i] * scalar;
	}
}

// Scalar division (float3)
__device__ __host__ inline float3 divide(const float3& a, float scalar)
{
	return make_float3(a.x / scalar, a.y / scalar, a.z / scalar);
}

// Scalar division (double3)
__device__ __host__ inline double3 divide(const double3& a, double scalar)
{
	return make_double3(a.x / scalar, a.y / scalar, a.z / scalar);
}

// Dot product (float3)
__device__ __host__ inline float dot(const float3& a, const float3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Dot product (double3)
__device__ __host__ inline double dot(const double3& a, const double3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Cross product (float3)
__device__ __host__ inline float3 cross(const float3& a, const float3& b)
{
	return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Cross product (double3)
__device__ __host__ inline double3 cross(const double3& a, const double3& b)
{
	return make_double3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Vector magnitude (length, float3)
__device__ __host__ inline float length(const float3& v) { return sqrtf(dot(v, v)); }

// Vector magnitude (length, double3)
__device__ __host__ inline double length(const double3& v) { return sqrt(dot(v, v)); }

// Normalize a vector (float3)
__device__ __host__ inline float3 normalize(const float3& v)
{
	float len = length(v);
	return (len > 0.0f) ? divide(v, len) : make_float3(0.0f, 0.0f, 0.0f);
}

// Normalize a vector (double3)
__device__ __host__ inline double3 normalize(const double3& v)
{
	double len = length(v);
	return (len > 0.0) ? divide(v, len) : make_double3(0.0, 0.0, 0.0);
}

// Normalize a vector in place (float3)
__device__ __host__ inline void normalize_inplace(float3& v)
{
	float len = length(v);
	if ( len > 0.0f )
	{
		v.x /= len;
		v.y /= len;
		v.z /= len;
	}
	else
	{
		v.x = 0.0f;
		v.y = 0.0f;
		v.z = 0.0f;
	}
}

// Normalize a vector in place (double3)
__device__ __host__ inline void normalize_inplace(double3& v)
{
	double len = length(v);
	if ( len > 0.0 )
	{
		v.x /= len;
		v.y /= len;
		v.z /= len;
	}
	else
	{
		v.x = 0.0;
		v.y = 0.0;
		v.z = 0.0;
	}
}

// Linear interpolation (lerp, float3)
__device__ __host__ inline float3 lerp(const float3& a, const float3& b, float t)
{
	return make_float3(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z));
}

// Linear interpolation (lerp, double3)
__device__ __host__ inline double3 lerp(const double3& a, const double3& b, double t)
{
	return make_double3(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z));
}

__device__ double3 generate_random_vector_uniform(curandState* state) {
    double3 vec;
    vec.x = curand_uniform_double(state) * 2.0 - 1.0;
    vec.y = curand_uniform_double(state) * 2.0 - 1.0;
    vec.z = curand_uniform_double(state) * 2.0 - 1.0;
    return vec;
}

__device__ bool is_zero(const double* vec, const int length, double tolerance = 1e-6)
{
	for ( int i = 0; i < length; i++ )
	{
		if ( vec[i] > tolerance || vec[i] < -tolerance )
		{
			return false;
		}
	}
	return true;
}

__device__ __host__ inline void MatVecMultiply_3D(const double* InMatPtr, const double3* InVecPtr, double3* OutResult)
{
	OutResult->x = InMatPtr[0] * InVecPtr->x + InMatPtr[1] * InVecPtr->y + InMatPtr[2] * InVecPtr->z;
	OutResult->y = InMatPtr[3] * InVecPtr->x + InMatPtr[4] * InVecPtr->y + InMatPtr[5] * InVecPtr->z;
	OutResult->z = InMatPtr[6] * InVecPtr->x + InMatPtr[7] * InVecPtr->y + InMatPtr[8] * InVecPtr->z;
}

__device__ __host__ inline void MatVecMultiply_3D_Inplace(const double* InMatPtr, double3* InOutVecPtr)
{
	double3 TempVec;
	TempVec.x    = InMatPtr[0] * InOutVecPtr->x + InMatPtr[1] * InOutVecPtr->y + InMatPtr[2] * InOutVecPtr->z;
	TempVec.y    = InMatPtr[3] * InOutVecPtr->x + InMatPtr[4] * InOutVecPtr->y + InMatPtr[5] * InOutVecPtr->z;
	TempVec.z    = InMatPtr[6] * InOutVecPtr->x + InMatPtr[7] * InOutVecPtr->y + InMatPtr[8] * InOutVecPtr->z;
	*InOutVecPtr = TempVec;
}

__device__ __host__ inline void MatVecMultiply_nmD(const double* InMatPtr,
	const double* InVecPtr,
	double* OutResult,
	const int MatRowSize,
	const int MatColSize)
{
	for ( int i = 0; i < MatRowSize; i++ )
	{
		double sum = 0.0;
		for ( int j = 0; j < MatColSize; j++ )
		{
			sum += InMatPtr[i * MatColSize + j] * InVecPtr[j];
		}
		OutResult[i] = sum;
	}
}
