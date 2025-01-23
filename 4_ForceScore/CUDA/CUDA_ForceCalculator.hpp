#ifndef E0EF931B_55D2_4783_B34E_593C248EDDAA
#define E0EF931B_55D2_4783_B34E_593C248EDDAA


#include <curand_kernel.h>
#include "cuda_force_calc_kernal.cuh"

// void ForceScoreCalc(int NormalPair, int RandomPara, int GMat, int GMatInv, /* Score calc: */ int KnifePose)
// {
// 	//// Random force gen
// 	// Gen random vec
// 	// Make it balance

// 	//// Calc force score
// }

__host__ void ForceGen(const int InGenCount,
	const int InRetryCount,
	const int InFingerCount,
	const double InFinalAngleMax,
	const double InForceMin,
	const double InForceMax,
	const double InGenAngleMax,
	double* InNormalVectorList,
	double* InExternalForce,
	double* InGMat,
	double* InGMatInv,
	bool* OutIsBalanced,
	double* OutForceScoreList,
	double* OutGeneratedForceList)
{
	constexpr int THREADS_PER_BLOCK = 1024;
	constexpr int BLOCKS = 8;
	constexpr int VEC_SIZE = 3;

	// Params
	FlowParams FlowParamsObj = { InGenCount, InRetryCount, InFingerCount };
	ForceGenParams ForceGenParamsObj = { InFinalAngleMax, InForceMin, InForceMax, InGenAngleMax };
	BalanceParams BalanceParamsObj = { InExternalForce, InGMat, InGMatInv };
	FlowParams* FlowParamsPtr;
	ForceGenParams* ForceGenParamsPtr;
	BalanceParams* BalanceParamsPtr;
	cudaMalloc(&FlowParamsPtr, sizeof(FlowParams));
	cudaMalloc(&ForceGenParamsPtr, sizeof(ForceGenParams));
	cudaMalloc(&BalanceParamsPtr, sizeof(BalanceParams));
	cudaMemcpy(FlowParamsPtr, &FlowParamsObj, sizeof(FlowParams), cudaMemcpyHostToDevice);
	cudaMemcpy(ForceGenParamsPtr, &ForceGenParamsObj, sizeof(ForceGenParams), cudaMemcpyHostToDevice);
	cudaMemcpy(BalanceParamsPtr, &BalanceParamsObj, sizeof(BalanceParams), cudaMemcpyHostToDevice);

	// Datas
	double* D_NormalVectorList;
	cudaMalloc(&D_NormalVectorList, InFingerCount * VEC_SIZE * sizeof(double));
	cudaMemcpy(D_NormalVectorList, InNormalVectorList, InFingerCount * VEC_SIZE * sizeof(double), cudaMemcpyHostToDevice);

	// Allocate states
    curandState* D_States;
    cudaMalloc(&D_States, THREADS_PER_BLOCK * BLOCKS * sizeof(curandState));

	// Output data
	bool* D_OutIsBalanced;
	double* D_OutForceScoreList;
	double* D_OutGeneratedForceList;
	cudaMalloc(&D_OutIsBalanced, InGenCount * sizeof(bool));
	cudaMalloc(&D_OutForceScoreList, InGenCount * sizeof(double));
	cudaMalloc(&D_OutGeneratedForceList, InGenCount * InFingerCount * VEC_SIZE * sizeof(double));


	// Calc(
	// 	FlowParamsPtr,
	// 	ForceGenParamsPtr,
	// 	BalanceParamsPtr,
	// 	D_States,
	// 	InNormalVectorList,
	// 	D_OutIsBalanced,
	// 	D_OutForceScoreList,
	// 	D_OutGeneratedForceList);


	cudaMemcpy(OutIsBalanced, D_OutIsBalanced, InGenCount * sizeof(bool), cudaMemcpyDeviceToHost);
	cudaMemcpy(OutForceScoreList, D_OutForceScoreList, InGenCount * sizeof(double), cudaMemcpyDeviceToHost);
	cudaMemcpy(OutGeneratedForceList, D_OutGeneratedForceList, InGenCount * InFingerCount * VEC_SIZE * sizeof(double), cudaMemcpyDeviceToHost);


	cudaFree(FlowParamsPtr);
	cudaFree(ForceGenParamsPtr);
	cudaFree(BalanceParamsPtr);

	cudaFree(D_NormalVectorList);

	cudaFree(D_States);

	cudaFree(D_OutIsBalanced);
	cudaFree(D_OutForceScoreList);
	cudaFree(D_OutGeneratedForceList);

}

#endif /* E0EF931B_55D2_4783_B34E_593C248EDDAA */
