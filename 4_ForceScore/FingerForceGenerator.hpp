#ifndef A879F1EF_340C_44BA_908D_C2BC726C884F
#define A879F1EF_340C_44BA_908D_C2BC726C884F

#include "DataTypes/PointSetData.hpp"
#include "GlobalTypes.hpp"
#include "GlobalBaseTypes.hpp"
#include "4_ForceScore/FingerForceBalancer.hpp"
#include "GlobalVars.hpp"
#include "spdlog/logger.h"
#include <cassert>
#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <random>
#include <memory>

#include <cuda_runtime.h>
#include "4_ForceScore/CUDA/cuda_force_calc_kernal.cuh"

/**
 * @brief A class for generating finger forces that balance with the input [InKnifeForce (also called ExternalForce)].
 * Output these finger forces in a std::vector for further force score calculation
 *
 */
template<typename Scalar, int ForceCount = 2>
class TFingerForceGenerator : public std::enable_shared_from_this<TFingerForceGenerator<Scalar, ForceCount>>
{
protected:
	using json = nlohmann::json;

	using GeneratorPointSetData = TSearchSpaceElement<ForceCount>;
	using GeneratorPointSetDataPtr = std::shared_ptr<GeneratorPointSetData>;
	using GeneratorPointSetDataConstPtr = std::shared_ptr<const GeneratorPointSetData>;

public:
	using ForcePairType      = Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor>;
	using ForcePairAllocType = Eigen::aligned_allocator<ForcePairType>;
	using ForcePairListType  = std::vector<ForcePairType, ForcePairAllocType>;
protected:
	using BalancerType       = TFingerForceBalancer<Scalar, ForceCount>;
	// using NormalType         = decltype(Types::CalcPointSetData::NormalPair);
	using NormalType         = Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor>;


public:
	constexpr static int FORCE_COUNT = ForceCount;
	friend class FingerForceGeneratorTest;
	friend class FingerForceGeneratorTest_ParamSetTest_Test;
	friend class FingerForceGeneratorTest_GenerateRandomVectorTest_Test;

	TFingerForceGenerator() = delete;

	TFingerForceGenerator(Types::ForceTorqueType InKnifeForce,
		Types::Vec3 InCoM,
		GeneratorPointSetDataConstPtr InPointSetDataPtr,
		size_t InFingerForceGenCount,
		const json& InJson,
		std::shared_ptr<spdlog::logger> InLogger = nullptr)
		: KnifeForce(InKnifeForce), CenterOfMass(InCoM), PointSetDataPtr(InPointSetDataPtr),
		  FingerForceGenCount(InFingerForceGenCount), Logger(InLogger)
	{
		GeneratingRetryTimes = InJson["CalForceScore"]["ForceGenBasicParam"]["GeneratingRetryTimes"];
		AngleLimit           = InJson["CalForceScore"]["ForceGenBasicParam"]["AngleLimit"].get<double>() * M_PI / 180.0;

		ForcePairList.reserve(FingerForceGenCount);
#ifndef DONG_ENABLE_CUDA
		ForcePairList.assign(FingerForceGenCount, ForcePairType::Zero());
#endif
		assert(PointSetDataPtr != nullptr);
		FingerForceBalancer.MakeGMat(PointSetDataPtr->PositionPair);

		SetRandomGeneratorSeed(); // Set seed as a random seed.
	}

	// Set seed for random number generator
	void SetRandomGeneratorSeed(std::random_device::result_type InSeed = 0, bool bInUseRandomSeed = true)
	{
		if ( bInUseRandomSeed )
		{
			bUseFixedSeed = false;
			RandGenerator.seed(RandomDev());
		}
		else
		{
			bUseFixedSeed = true;
			RandGenerator.seed(InSeed);
		}
	}

	ForcePairListType& GetGeneratedFingerForceList() { return ForcePairList; }

	virtual size_t GenerateFingerForceList()
	{
		size_t SuccessCount = 0;

		size_t ConditionFailedCount = 0;
		size_t BalanceFailedCount = 0;
		size_t ForceLengthOutOfRangeCount = 0;
		size_t AngleExceedsLimitCount = 0;

		// #pragma omp parallel for reduction(+:SuccessCount, ConditionFailedCount, BalanceFailedCount, ForceLengthOutOfRangeCount, AngleExceedsLimitCount)
		for ( size_t i = 0; i < FingerForceGenCount; i++ )
		{
			int RetryTimes = GeneratingRetryTimes;
			do
			{
				RetryTimes--;

				ForcePairType& ForcePair = ForcePairList[i];
				GenereateSingleForcePair(ForcePair);  // This function is implemented in the sub-class.

				Eigen::Map<typename BalancerType::ForceListType> FingerForceList(ForcePair.data());
				bool bGenSuccess = FingerForceBalancer.MakeForceBalanced(FingerForceList);

				int FailureReason = 0;
				// Check if the generated finger force is valid
				if ( !IsValidFingerForce(ForcePair, PointSetDataPtr->NormalPair, FailureReason) )
				{
					ConditionFailedCount++;
					if ( FailureReason == 1 )
					{
						ForceLengthOutOfRangeCount++;
					}
					else if ( FailureReason == 2 )
					{
						AngleExceedsLimitCount++;
					}
					// If invalid, retry generating.
					continue;
				}
				if ( bGenSuccess )
				{
					// ONLY count the valid and balanced finger force.
					SuccessCount++;
					break;
				}
				else
				{
					BalanceFailedCount++;
				}
			}
			while ( RetryTimes );

			if ( RetryTimes == 0 )
			{
				// If the retry times is exhausted, set the force pair to infinity.
				ForcePairList[i].setConstant(std::numeric_limits<Scalar>::infinity());
			}
		}
		
		// Remove the invalid (force are Eigen::Infinity) force pairs.
		ForcePairList.erase(
			std::remove_if(ForcePairList.begin(), ForcePairList.end(),
						   [](const ForcePairType& ForcePair) {
							const bool&& bIsInf = ForcePair.array().isInf().any();
							   return bIsInf;
						   }),
			ForcePairList.end());

		if ( Logger )
		{
			Logger->trace("===---===---===: RetSize: {}, ConditionFailed: {}, BalanceFailed: {}  ;;   LengthOut: {}, AngleOut: {}",
						 ForcePairList.size(), ConditionFailedCount, BalanceFailedCount, ForceLengthOutOfRangeCount, AngleExceedsLimitCount);
		}

		return ForcePairList.size();
	}

	// Getters
	size_t GetFingerForceGenCount() const { return FingerForceGenCount; }
	int GetGeneratingRetryTimes() const { return GeneratingRetryTimes; }

protected:
	virtual bool IsValidFingerForce(const ForcePairType& InFingerForce, const NormalType& InPointSetNormal, int& OutFailureReason) const
	{
		enum FailureReason
		{
			None = 0,
			ForceLengthOutOfRange,
			AngleExceedsLimit
		};

		for ( int i = 0; i < FORCE_COUNT; i++ )
		{
			const auto& InForce = InFingerForce.row(i);
			const auto& Normal  = InPointSetNormal.row(i);

			const double ForceLength = InForce.norm();

			// Skip zero force vectors
			if ( ForceLength < 1e-6 )
			{
				continue;
			}

			// Check if the force vector is within the specified range
			if ( ForceLength > ForceMax * 2 )
			{
				OutFailureReason = ForceLengthOutOfRange;
				return false;
			}

			// Calculate angle between force and normal
			const auto ForceDir = InForce / ForceLength;
			const double Angle = std::acos(ForceDir.dot(Normal));

			// Check if angle exceeds limit
			if ( Angle > AngleLimit )
			{
				OutFailureReason = AngleExceedsLimit;
				return false;
			}
		}
		OutFailureReason = None;
		return true;
	}

	virtual void GenereateSingleForcePair(ForcePairType& InOutForcePair) = 0;

protected:

	Types::ForceTorqueType KnifeForce;
	Types::Vec3 CenterOfMass;
	GeneratorPointSetDataConstPtr PointSetDataPtr;
	ForcePairListType ForcePairList;
	size_t FingerForceGenCount;
	int GeneratingRetryTimes;

	double AngleLimit;
	bool bUseFixedSeed = false;
	/// The length of the generated force vector is limited to [ForceMin, ForceMax] default is [0, INFINITY]
	/// you can override this value in the sub-class.
	double ForceMin = 0;
	/// The length of the generated force vector is limited to [ForceMin, ForceMax] default is [0, INFINITY], 
	/// you can override this value in the sub-class.
	double ForceMax = INFINITY;

	// Random seed generator
	std::random_device RandomDev;
	// Random number generator
	std::mt19937 RandGenerator;

	BalancerType FingerForceBalancer{ CenterOfMass, KnifeForce };

	std::shared_ptr<spdlog::logger> Logger;
};

template<typename Scalar = double, int ForceCount = 2>
class TFingerForceGeneratorWithinCone : public TFingerForceGenerator<Scalar, ForceCount>
{
	DEF_MATRIX_TYPES_ALIASES(Scalar);
public:
	using Super = TFingerForceGenerator<Scalar, ForceCount>;

	friend class FingerForceGeneratorTest;
	friend class FingerForceGeneratorTest_ParamSetTest_Test;
	friend class FingerForceGeneratorTest_GenerateRandomVectorTest_Test;

	TFingerForceGeneratorWithinCone() = delete;

	TFingerForceGeneratorWithinCone(Types::ForceTorqueType InKnifeForce,
		Types::Vec3 InCoM,
		typename Super::GeneratorPointSetDataConstPtr InPointSetDataPtr,
		size_t InFingerForceGenCount,
		const typename Super::json& InJson,
		std::shared_ptr<spdlog::logger> InLogger = nullptr)
		: Super(InKnifeForce, InCoM, InPointSetDataPtr, InFingerForceGenCount, InJson, InLogger)
	{
		// Do nothing
		const typename Super::json& GeneratorJsonObj = InJson["CalForceScore"]["FingerForceGeneratorWithinCone_Param"];
		const double ForceRangeMin   = GeneratorJsonObj["ForceRangeMin"];
		const double ForceRangeMax   = GeneratorJsonObj["ForceRangeMax"];
		const double RandomHalfAngle = (GeneratorJsonObj["RandomHalfAngle"].template get<double>()) * M_PI / 180.0;
		this->AngleLimit = std::min(this->AngleLimit, RandomHalfAngle);

		this->ForceMin = ForceRangeMin;
		this->ForceMax = ForceRangeMax;

		this->LengthDist  = std::uniform_real_distribution<>{ ForceRangeMin, ForceRangeMax };
		this->AngleDist   = std::uniform_real_distribution<>{ 0, RandomHalfAngle };
		this->AzimuthDist = std::uniform_real_distribution<>{ 0, 2 * M_PI };

		// for (int i = 0; i < Types::CalcPointSetData::FORCE_COUNT; i++) {
		// 	const auto NormalVec = this->PointSetDataPtr->NormalPair.row(i);
		// 	CachedQuaternions[i] = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), NormalVec.normalized());
		// }
		CachedRotationMatrices.assign(ForceCount, Eigen::Matrix<Scalar, 3, 3>::Identity());
		for ( int i = 0; i < ForceCount; i++ )
		{
			auto& NormalVec      = this->PointSetDataPtr->NormalPair.row(i);
			Vec3 zAxis     = Vec3::UnitZ();
			Vec3 axis      = zAxis.cross(NormalVec.normalized());
			double angle              = acos(zAxis.dot(NormalVec));
			CachedRotationMatrices[i] = Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
		}
	}

protected:
	virtual size_t GenerateFingerForceList() override
	{
		constexpr int GEN_COUNT = 65536 * 2;
		constexpr int RETRY_COUNT = 5;

		RetFlagsType ReturnedFlagsList[GEN_COUNT];
		Eigen::MatrixXd ForceScoreList(GEN_COUNT, 1);
		Eigen::Matrix<double, -1, -1, Eigen::RowMajor> GeneratedForceList(GEN_COUNT, 3 * ForceCount);

		double ExtForceLength = this->FingerForceBalancer.GetExternalForce().template block<3, 1>(0, 0).norm();
		ForceGen( GEN_COUNT, RETRY_COUNT, FINGER_NUMBER, 
			this->AngleLimit, this->ForceMin, ExtForceLength, this->AngleDist.max(), 
			this->PointSetDataPtr->NormalPair.data(), 
			this->FingerForceBalancer.GetExternalForce().data(), 
			this->FingerForceBalancer.GMat.data(), 
			this->FingerForceBalancer.GMatInv.data(), 
			ReturnedFlagsList, ForceScoreList.data(), GeneratedForceList.data());

		size_t SuccessCount = 0;

		size_t ConditionFailedCount = 0;
		size_t BalanceFailedCount = 0;
		size_t ForceLengthOutOfRangeCount = 0;
		size_t AngleExceedsLimitCount = 0;
		int FailureReason = 0;
		
		#pragma omp parallel for reduction(+:SuccessCount, ConditionFailedCount, BalanceFailedCount, ForceLengthOutOfRangeCount, AngleExceedsLimitCount)
		for ( int i = 0; i < GeneratedForceList.rows(); i++ )
		{
			const Eigen::Map<typename Super::ForcePairType> ForcePairMap(GeneratedForceList.data() + i * 3 * ForceCount);
			if ( !this->IsValidFingerForce(ForcePairMap, this->PointSetDataPtr->NormalPair, FailureReason) )
			{
				ConditionFailedCount++;
				if ( FailureReason == 1 )
				{
					ForceLengthOutOfRangeCount++;
				}
				else if ( FailureReason == 2 )
				{
					AngleExceedsLimitCount++;
				}
				// If invalid, retry generating.
				continue;
			}
			if ( ReturnedFlagsList[i].IsBalance )
			{
				// ONLY count the valid and balanced finger force.
				SuccessCount++;
			}
			else
			{
				BalanceFailedCount++;
			}
		}

		this->ForcePairList.reserve(SuccessCount);
		#pragma omp parallel for
		for ( int i = 0; i < SuccessCount; i++ )
		{
			const Eigen::Map<typename Super::ForcePairType> ForcePairMap(GeneratedForceList.data() + i * 3 * ForceCount);
			this->ForcePairList.push_back(ForcePairMap);
		}
		std::cout << "------------- ForcePairList.size(): " << this->ForcePairList.size() << std::endl;
		// this->Logger->warn("===---===---===: RetSize: {}, ConditionFailed: {}, BalanceFailed: {}  ;;   LengthOut: {}, AngleOut: {}",
		// 				   SuccessCount, ConditionFailedCount, BalanceFailedCount, ForceLengthOutOfRangeCount, AngleExceedsLimitCount);

		return SuccessCount;
	}

	virtual void GenereateSingleForcePair(typename Super::ForcePairType& InOutForcePair) override
	{
		for ( int i = 0; i < Types::CalcPointSetData::FORCE_COUNT; i++ )
		{
			const auto NormalVec = this->PointSetDataPtr->NormalPair.row(i);

			// Generate random length
			const double Length = this->LengthDist(this->RandGenerator);

			// Generate random direction within cone
			const double Theta = this->AngleDist(this->RandGenerator);
			const double Phi   = this->AzimuthDist(this->RandGenerator);
			
			InOutForcePair.row(i) << sin(Theta) * cos(Phi), sin(Theta) * sin(Phi), cos(Theta);
			
			const Vec3 RotatedDir = CachedRotationMatrices[i] * InOutForcePair.row(i).transpose();
			InOutForcePair.row(i) = Length * RotatedDir;
		}
	}


protected:
	std::uniform_real_distribution<> LengthDist;
	std::uniform_real_distribution<> AngleDist;
	std::uniform_real_distribution<> AzimuthDist;
	// std::vector<Eigen::Quaterniond> CachedQuaternions;
	std::vector<Eigen::Matrix<Scalar, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<Scalar, 3, 3>>> CachedRotationMatrices;
};

template<typename Scalar = double, int ForceCount = 2>
class TFingerForceGeneratorByRandomK : public TFingerForceGenerator<Scalar, ForceCount>
{
public:
	using Super = TFingerForceGenerator<Scalar, ForceCount>;

	friend class FingerForceGeneratorTest;
	friend class FingerForceGeneratorTest_ParamSetTest_Test;
	friend class FingerForceGeneratorTest_GenerateRandomVectorTest_Test;

	TFingerForceGeneratorByRandomK() = delete;

	TFingerForceGeneratorByRandomK(Types::ForceTorqueType InKnifeForce,
		Types::Vec3 InCoM,
		typename Super::GeneratorPointSetDataConstPtr InPointSetDataPtr,
		size_t InFingerForceGenCount,
		const typename Super::json& InJson,
		std::shared_ptr<spdlog::logger> InLogger = nullptr)
		: TFingerForceGenerator<Scalar, ForceCount>(InKnifeForce, InCoM, InPointSetDataPtr, InFingerForceGenCount, InJson, InLogger)
	{
		const typename Super::json& GeneratorJsonObj = InJson["CalForceScore"]["FingerForceGeneratorByRandomK_Param"];
		const double ForceRangeMin   = GeneratorJsonObj["ForceRangeMin"];
		const double ForceRangeMax   = GeneratorJsonObj["ForceRangeMax"];

		this->ForceMin = ForceRangeMin;
		this->ForceMax = ForceRangeMax;

		this->LengthDist = std::uniform_real_distribution<>{ ForceRangeMin, ForceRangeMax };
	}

	virtual void GenereateSingleForcePair(typename Super::ForcePairType& InOutForcePair) override
	{
		typename Super::BalancerType::ForceListType RandForce = typename Super::BalancerType::ForceListType::Zero();
		if ( this->bUseFixedSeed )
		{
			// Insert fixed seed random number into [InOutForcePair]
			for ( int i = 0; i < InOutForcePair.size(); i++ )
			{
				RandForce(i) = this->LengthDist(this->RandGenerator);
			}
		}
		else
		{
			RandForce = typename Super::BalancerType::ForceListType::Random();
		}

		// Normalize the random force vector with each row.
		// And rescale the force vector with a length.
		for ( int i = 0; i < InOutForcePair.rows(); i++ )
		{
			auto ForceVec = RandForce.block(i * 3, 0, (i + 1) * 3, 1);
			ForceVec.normalized();
			ForceVec *= this->LengthDist(this->RandGenerator);
		}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		typename Super::BalancerType::ForceListType Force = this->FingerForceBalancer.CalculateFingerForeceByK(RandForce);
#pragma GCC diagnostic pop
#pragma clang diagnostic pop

		InOutForcePair = Eigen::Map<typename Super::ForcePairType>(Force.data());
	}

protected:
	std::uniform_real_distribution<> LengthDist;
};

#endif /* A879F1EF_340C_44BA_908D_C2BC726C884F */
