#include <curand_kernel.h>


struct FlowParams
{
	int GenCount;
	int RetryCount;
	int FingerCount;
};
struct ForceGenParams
{
	double FinalAngleMax;
	double ForceMin;
	double ForceMax;
	double GenAngleMax;
};

constexpr int CUDA_MAX_FINGER_CNT = 10;
struct BalanceParams
{
	// double ExternalForce[6];
	// double GMat[6 * CUDA_MAX_FINGER_CNT * 3];  // 6, CUDA_MAX_FINGER_CNT * 3
	// double GMatInv[CUDA_MAX_FINGER_CNT * 3 * 6];
	double* ExternalForce;
	double* GMat;
	double* GMatInv;
};

// Using bit fields
union RetFlagsType {
	struct {
		unsigned int IsBalance : 1;
		unsigned int IsAngleInLimit : 1;
		unsigned int IsForceInLimit : 1;
	};
	unsigned char Flags = 0xFF;
};

// __global__ void Calc(
// 	const FlowParams* InFlowParams,
// 	const ForceGenParams* InForceGenParams,
// 	const BalanceParams* InBalanceParams,
// 	curandState* InRandState,
// 	double* InNormalVectorList,
// 	bool* OutIsBalanced,
// 	double* OutForceScoreList,
// 	double* OutGeneratedForceList);
extern "C" void ForceGen(const int InGenCount,
    const int InRetryCount,
    const int InFingerCount,
    const double InFinalAngleMax,
    const double InForceMin,
    const double InForceMax,
    const double InGenAngleMax,
    const double* InNormalVectorList,
    double* InExternalForce,
    double* InGMat,
    double* InGMatInv,
    RetFlagsType* OutIsBalanced,
    double* OutForceScoreList,
    double* OutGeneratedForceList);
