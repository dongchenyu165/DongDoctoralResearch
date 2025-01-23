#include "utils.cuh"
#include "cuda_vec_ops.cuh"
#include "cuda_force_calc_kernal.cuh"
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <vector_functions.h>


__global__ void init_rand_states(curandState* InStatesListPtr, unsigned long long InBaseSeed) {
	int Idx = blockIdx.x * blockDim.x + threadIdx.x;
	InBaseSeed ^= Idx;
	curand_init(InBaseSeed, Idx, 0, &InStatesListPtr[Idx]);
}

__global__ void Calc(
	const FlowParams* InFlowParams,
	const ForceGenParams* InForceGenParams,
	const BalanceParams* InBalanceParams,
	curandState* InRandState,
	const double* InNormalVectorList,
	RetFlagsType* OutReturnFlagsList,
	double* OutForceScoreList,
	double* OutGeneratedForceList)
{
	// constexpr bool DEBUG_PRINT = false;
	constexpr int FORCE_TORQUE_VEC_LEN = 6;
	// constexpr int MAX_FINGER_COUNT = 10;
	constexpr int VEC_LEN = 3;
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= InFlowParams->GenCount)
	{
		return;
	}

	const int PAIR_STEP = idx * InFlowParams->FingerCount * VEC_LEN;

	// Print input parameters
	// if (idx == 23)
	// {
	// 	printf("InFlowParams->GenCount: %d\n", InFlowParams->GenCount);
	// 	printf("InFlowParams->RetryCount: %d\n", InFlowParams->RetryCount);
	// 	printf("InFlowParams->FingerCount: %d\n", InFlowParams->FingerCount);
	// 	printf("InForceGenParams->FinalAngleMax: %f\n", InForceGenParams->FinalAngleMax);
	// 	printf("InForceGenParams->ForceMin: %f\n", InForceGenParams->ForceMin);
	// 	printf("InForceGenParams->ForceMax: %f\n", InForceGenParams->ForceMax);
	// 	printf("InForceGenParams->GenAngleMax: %f\n", InForceGenParams->GenAngleMax);
	// 	printf("InBalanceParams->ExternalForce: %f %f %f %f %f %f\n", 
	// 		InBalanceParams->ExternalForce[0], InBalanceParams->ExternalForce[1], InBalanceParams->ExternalForce[2],
	// 		InBalanceParams->ExternalForce[3], InBalanceParams->ExternalForce[4], InBalanceParams->ExternalForce[5]);
		
	// 	for (int i = 0; i < InFlowParams->FingerCount; i++)
	// 	{
	// 		printf("InNormalVectorList[%d]: %f %f %f\n", i, InNormalVectorList[i * VEC_LEN], InNormalVectorList[i * VEC_LEN + 1], InNormalVectorList[i * VEC_LEN + 2]);
	// 	}
	// }

	int RetryCount = InFlowParams->RetryCount;

	do
	{

		// Generate [FingerCount] random force vectors
		for (int i = 0; i < InFlowParams->FingerCount; i++)
		{
			double3* RefNormalVec = (double3*)(InNormalVectorList + i * VEC_LEN);
			double3 random_rot_vector = generate_random_vector_uniform(InRandState + idx);

			// if (idx > 23 && idx < 67)
			// {
			// 	printf("random_rot_vector: %f %f %f\n", random_rot_vector.x, random_rot_vector.y, random_rot_vector.z);
			// }

			double dot_product = dot(*RefNormalVec, random_rot_vector);
			double3 _vec = multiply(*RefNormalVec, dot_product);
			random_rot_vector = subtract(random_rot_vector, _vec);
			normalize_inplace(random_rot_vector);

			double RotMat[9];
			double angle = (curand_uniform_double(InRandState + idx)) * InForceGenParams->GenAngleMax;
			generate_rotation_matrix(RotMat, angle, random_rot_vector);

			double* ForceVecPtr = OutGeneratedForceList + PAIR_STEP + i * VEC_LEN;
			double3* OutVecPtr = (double3*)(ForceVecPtr);
			MatVecMultiply_3D(RotMat, RefNormalVec, OutVecPtr);

			double ForceScale = InForceGenParams->ForceMin + 
				(InForceGenParams->ForceMax - InForceGenParams->ForceMin) * curand_uniform_double(InRandState + idx);

			double* OutVecPtr2 = OutGeneratedForceList + PAIR_STEP + i * VEC_LEN;
			multiply(OutVecPtr2, ForceScale, OutVecPtr2, VEC_LEN);
		}

		double* ForceVecs = (double*)(OutGeneratedForceList + PAIR_STEP);
		// if (DEBUG_PRINT && idx == 23) {
		// 	printf("ForceVecs: \n");
		// 	for (int i = 0; i < InFlowParams->FingerCount * VEC_LEN; i++) {
		// 		printf("%f ", ForceVecs[i]);
		// 	}
		// 	printf("\n");
		// }
		
		// BalancedForceError = <<GMat * InOutInitForce>> - ExternalForce;
		double InitComposedForce[FORCE_TORQUE_VEC_LEN];
		MatVecMultiply_nmD(InBalanceParams->GMat, ForceVecs, InitComposedForce, 
			FORCE_TORQUE_VEC_LEN, InFlowParams->FingerCount * VEC_LEN);
		// if (DEBUG_PRINT && idx == 23) {
		// 	printf("InitComposedForce: \n");
		// 	for (int i = 0; i < FORCE_TORQUE_VEC_LEN; i++) {
		// 		printf("%f ", InitComposedForce[i]);
		// 	}
		// 	printf("\n");
		// }

		// BalancedForceError = <<GMat * InOutInitForce - ExternalForce>>;
		double MinusedBalancedForceError[FORCE_TORQUE_VEC_LEN];
		subtract(InBalanceParams->ExternalForce, InitComposedForce, MinusedBalancedForceError, FORCE_TORQUE_VEC_LEN);
		// if constexpr (DEBUG_PRINT && idx == 23) {
		// 	printf("BalancedForceError: \n");
		// 	for (int i = 0; i < FORCE_TORQUE_VEC_LEN; i++) {
		// 		printf("%f ", MinusedBalancedForceError[i]);
		// 	}
		// 	printf("\n");
		// 	printf("ExternalForce: \n");
		// 	for (int i = 0; i < FORCE_TORQUE_VEC_LEN; i++) {
		// 		printf("%f ", InBalanceParams->ExternalForce[i]);
		// 	}
		// 	printf("\n");
		// }

		// InOutInitForce = InOutInitForce + GMatInv * <<(-BalancedForceError)>>;
		// multiply(MinusedBalancedForceError, -1.0, MinusedBalancedForceError, FORCE_TORQUE_VEC_LEN);
		// InOutInitForce = InOutInitForce + <<GMatInv * (-BalancedForceError)>>;
		double TempForceVecs[10 * VEC_LEN];
		MatVecMultiply_nmD(InBalanceParams->GMatInv, MinusedBalancedForceError, TempForceVecs, 
			InFlowParams->FingerCount * VEC_LEN, FORCE_TORQUE_VEC_LEN);
		// InOutInitForce = <<InOutInitForce + GMatInv * (-BalancedForceError)>>;
		add(ForceVecs, TempForceVecs, ForceVecs, InFlowParams->FingerCount * VEC_LEN);
		// if (DEBUG_PRINT && idx == 23) {
		// 	printf("minused BalancedForceError: \n");
		// 	for (int i = 0; i < FORCE_TORQUE_VEC_LEN; i++) {
		// 		printf("%f ", MinusedBalancedForceError[i]);
		// 	}
		// 	printf("\n");
		// 	printf("ForceVecs after correction: \n");
		// 	for (int i = 0; i < InFlowParams->FingerCount * VEC_LEN; i++) {
		// 		printf("%f ", ForceVecs[i]);
		// 	}
		// 	printf("\n");
		// }

		double Checking_InitComposedForce[FORCE_TORQUE_VEC_LEN];
		MatVecMultiply_nmD(InBalanceParams->GMat, ForceVecs, Checking_InitComposedForce, 
			FORCE_TORQUE_VEC_LEN, InFlowParams->FingerCount * VEC_LEN);
		// if (DEBUG_PRINT && idx == 23) {
		// 	printf("Checking_InitComposedForce: \n");
		// 	for (int i = 0; i < FORCE_TORQUE_VEC_LEN; i++) {
		// 		printf("%f ", Checking_InitComposedForce[i]);
		// 	}
		// 	printf("\n");
		// }

		double Checking_BalancedForceError[FORCE_TORQUE_VEC_LEN];
		subtract(Checking_InitComposedForce, InBalanceParams->ExternalForce, Checking_BalancedForceError, FORCE_TORQUE_VEC_LEN);
		// if (DEBUG_PRINT && idx == 23) {
		// 	printf("Checking_BalancedForceError: \n");
		// 	for (int i = 0; i < FORCE_TORQUE_VEC_LEN; i++) {
		// 		printf("%f ", Checking_BalancedForceError[i]);
		// 	}
		// 	printf("\n");
		// }

		// OutIsBalanced[idx] = is_zero(Checking_BalancedForceError, FORCE_TORQUE_VEC_LEN);
		OutReturnFlagsList[idx].IsBalance = is_zero(Checking_BalancedForceError, FORCE_TORQUE_VEC_LEN) && (!is_zero(ForceVecs, FORCE_TORQUE_VEC_LEN));

		// DEBUG print output
		// if (DEBUG_PRINT && idx == 23)
		// {
		// 	// Print InBalanceParams->GMat Size is 6 * InFlowParams->FingerCount * VEC_LEN
		// 	printf("InBalanceParams->GMat:\n");
		// 	for (int i = 0; i < 6; i++)
		// 	{
		// 		for (int j = 0; j < InFlowParams->FingerCount * VEC_LEN; j++)
		// 		{
		// 			printf("%+7.6f ", InBalanceParams->GMat[i * InFlowParams->FingerCount * VEC_LEN + j]);
		// 		}
		// 		printf("\n");
		// 	}

		// 	printf("\n\nInBalanceParams->GMatInv:\n");
		// 	for (int i = 0; i < InFlowParams->FingerCount * VEC_LEN; i++)
		// 	{
		// 		for (int j = 0; j < 6; j++)
		// 		{
		// 			printf("%+7.6f ", InBalanceParams->GMatInv[i * 6 + j]);
		// 		}
		// 		printf("\n");
		// 	}
		// }

		double3* OutVecPtr = (double3*)(OutGeneratedForceList + PAIR_STEP);
		OutForceScoreList[idx] = length(*OutVecPtr);

		// Check if the angle between the generated force's direction and normal is within the limit.
		for (int i = 0; i < InFlowParams->FingerCount; i++)
		{
			double3* RefNormalVec = (double3*)(InNormalVectorList + i * VEC_LEN);
			double3* ForceVec = (double3*)(OutGeneratedForceList + PAIR_STEP + i * VEC_LEN);
			const double CheckingAngleDot = dot(*RefNormalVec, *ForceVec);
			const double LimitAngleDot = cos(InForceGenParams->FinalAngleMax);
			OutReturnFlagsList[idx].IsAngleInLimit = (CheckingAngleDot >= LimitAngleDot);
			if (!OutReturnFlagsList[idx].IsAngleInLimit)
			{
				break;
			}
		}

		// Check if the generated force is within the limit.
		for (int i = 0; i < InFlowParams->FingerCount; i++)
		{
			double3* ForceVec = (double3*)(OutGeneratedForceList + PAIR_STEP + i * VEC_LEN);
			const double CheckingForceLen = length(*ForceVec);
			OutReturnFlagsList[idx].IsForceInLimit = CheckingForceLen <= InForceGenParams->ForceMax;
			if (!OutReturnFlagsList[idx].IsForceInLimit)
			{
				break;
			}
		}
	} while (OutReturnFlagsList[idx].Flags != 0x07 && RetryCount-- > 0);

}





void ForceGen(const int InGenCount,
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
	double* OutGeneratedForceList)
{
	constexpr int THREADS_PER_BLOCK = 512;
	// const int BLOCKS = InGenCount / THREADS_PER_BLOCK;
	const int BLOCKS = (InGenCount + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
	constexpr int VEC_SIZE = 3;
	
    // Get current time as base seed
	auto NowTime = std::chrono::high_resolution_clock::now();
    unsigned long long BaseSeed = static_cast<unsigned long long>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(NowTime.time_since_epoch()).count());


	// Params
	FlowParams FlowParamsObj = { InGenCount, InRetryCount, InFingerCount };
	ForceGenParams ForceGenParamsObj = { InFinalAngleMax, InForceMin, InForceMax, InGenAngleMax };
	FlowParams* D_FlowParamsPtr;
	ForceGenParams* D_ForceGenParamsPtr;
	cudaMalloc(&D_FlowParamsPtr, sizeof(FlowParams));
	cudaMalloc(&D_ForceGenParamsPtr, sizeof(ForceGenParams));
	cudaMemcpy(D_FlowParamsPtr, &FlowParamsObj, sizeof(FlowParams), cudaMemcpyHostToDevice);
	cudaMemcpy(D_ForceGenParamsPtr, &ForceGenParamsObj, sizeof(ForceGenParams), cudaMemcpyHostToDevice);
	// Balance Params:
	BalanceParams* D_BalanceParamsPtr;  // [D_BalanceParamsPtr] No permission to assign or free memory.
	double* D_ExternalForcePtr;
	double* D_GMatPtr;
	double* D_GMatInvPtr;

	{
		cudaMalloc(&D_BalanceParamsPtr, sizeof(BalanceParams));
		cudaMalloc(&D_ExternalForcePtr, 6 * sizeof(double));
		cudaMalloc(&D_GMatPtr, (InFingerCount * 3) * (6) * sizeof(double));
		cudaMalloc(&D_GMatInvPtr, (6) * (InFingerCount * 3) * sizeof(double));
		// Update device-side BalanceParams structure
		BalanceParams HostBalanceParams;
		HostBalanceParams.ExternalForce = D_ExternalForcePtr;
		HostBalanceParams.GMat = D_GMatPtr;
		HostBalanceParams.GMatInv = D_GMatInvPtr;

		cudaMemcpy(D_BalanceParamsPtr, &HostBalanceParams, sizeof(BalanceParams), cudaMemcpyHostToDevice);
		
		cudaMemcpy(D_ExternalForcePtr, InExternalForce, 6 * sizeof(double), cudaMemcpyHostToDevice);
		cudaMemcpy(D_GMatPtr, InGMat, (InFingerCount * 3) * (6) * sizeof(double), cudaMemcpyHostToDevice);
		cudaMemcpy(D_GMatInvPtr, InGMatInv, (6) * (InFingerCount * 3) * sizeof(double), cudaMemcpyHostToDevice);
	}

	// Datas
	double* D_NormalVectorList;
	cudaMalloc(&D_NormalVectorList, InFingerCount * VEC_SIZE * sizeof(double));
	cudaMemcpy(D_NormalVectorList, InNormalVectorList, InFingerCount * VEC_SIZE * sizeof(double), cudaMemcpyHostToDevice);

	// Allocate states
    curandState* D_States;
    cudaMalloc(&D_States, THREADS_PER_BLOCK * BLOCKS * sizeof(curandState));

	// Output data
	RetFlagsType* D_OutIsBalanced;
	double* D_OutForceScoreList;
	double* D_OutGeneratedForceList;
	cudaMalloc(&D_OutIsBalanced, InGenCount * sizeof(RetFlagsType));
	cudaMalloc(&D_OutForceScoreList, InGenCount * sizeof(double));
	cudaMalloc(&D_OutGeneratedForceList, InGenCount * InFingerCount * VEC_SIZE * sizeof(double));


	init_rand_states<<<BLOCKS, THREADS_PER_BLOCK>>>(D_States, 0);
	Calc<<<BLOCKS, THREADS_PER_BLOCK>>>(
		D_FlowParamsPtr,
		D_ForceGenParamsPtr,
		D_BalanceParamsPtr,
		D_States,
		D_NormalVectorList,
		D_OutIsBalanced,
		D_OutForceScoreList,
		D_OutGeneratedForceList);
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA error: %s\n", cudaGetErrorString(err));
    }
    // cudaDeviceSynchronize(); // Ensure all printf output is flushed
	err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        printf("CUDA 【cudaDeviceSynchronize】中文测试 error: %s\n", cudaGetErrorString(err));
    }

	cudaMemcpy(OutIsBalanced, D_OutIsBalanced, InGenCount * sizeof(RetFlagsType), cudaMemcpyDeviceToHost);
	cudaMemcpy(OutForceScoreList, D_OutForceScoreList, InGenCount * sizeof(double), cudaMemcpyDeviceToHost);
	cudaMemcpy(OutGeneratedForceList, D_OutGeneratedForceList, InGenCount * InFingerCount * VEC_SIZE * sizeof(double), cudaMemcpyDeviceToHost);


	cudaFree(D_FlowParamsPtr);
	cudaFree(D_ForceGenParamsPtr);

	cudaFree(D_ExternalForcePtr);
	cudaFree(D_GMatPtr);
	cudaFree(D_GMatInvPtr);
	cudaFree(D_BalanceParamsPtr);

	cudaFree(D_NormalVectorList);

	cudaFree(D_States);

	cudaFree(D_OutIsBalanced);
	cudaFree(D_OutForceScoreList);
	cudaFree(D_OutGeneratedForceList);


	// size_t totalMemory = sizeof(FlowParams) +
    //                  sizeof(ForceGenParams) +
    //                  sizeof(BalanceParams) +
    //                  6 * sizeof(double) +
    //                  (InFingerCount * 3) * 6 * sizeof(double) +
    //                  6 * InFingerCount * 3 * sizeof(double) +
    //                  InFingerCount * VEC_SIZE * sizeof(double) +
    //                  THREADS_PER_BLOCK * BLOCKS * sizeof(curandState) +
    //                  InGenCount * sizeof(bool) +
    //                  InGenCount * sizeof(double) +
    //                  InGenCount * InFingerCount * VEC_SIZE * sizeof(double);
	// printf("Total memory: %lu Byte, %luMb\n", totalMemory, totalMemory / 1024 / 1024 * 48);
}