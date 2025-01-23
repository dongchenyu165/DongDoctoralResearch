#ifndef C369AAB0_0C1A_4490_A104_5A051204BB76
#define C369AAB0_0C1A_4490_A104_5A051204BB76

// CalcPointSetDataPtr

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <map>
#include "GlobalTypes.hpp"
#include "GlobalBaseTypes.hpp"
#include "4_ForceScore/ForceScoreCalculatorConfig.hpp"
#include "ScoreCalculator/ScoreCalculatorBase.hpp"
#include "Utilities/EigenHelper/EigenComparator.hpp"

namespace __ForceScore_INTERNAL
{
// Define the type aliases inside namespace
template<typename Scalar, int ForceCount>
using ForcePairType = Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor>;

template<typename Scalar, int ForceCount>
using MatrixLessType = Utilities::MatrixLess<Scalar, ForceCount, 3, Eigen::RowMajor>;

template<typename Scalar, int ForceCount>
using AllocatorType = Eigen::aligned_allocator<ForcePairType<Scalar, ForceCount>>;
} // namespace __ForceScore_INTERNAL

template<typename Scalar, int ForceCount>
class TForceScoreCalculator : public TScoreCalculatorBase<__ForceScore_INTERNAL::ForcePairType<Scalar, ForceCount>, 
	ForceScoreCalcConfig, 
	__ForceScore_INTERNAL::MatrixLessType<Scalar, ForceCount>, 
	__ForceScore_INTERNAL::AllocatorType<Scalar, ForceCount>>
{
public:
	using ForcePairType = __ForceScore_INTERNAL::ForcePairType<Scalar, ForceCount>;
protected:
	using MatrixLessType = __ForceScore_INTERNAL::MatrixLessType<Scalar, ForceCount>;
	using AllocatorType = __ForceScore_INTERNAL::AllocatorType<Scalar, ForceCount>;
private:
	using Super = TScoreCalculatorBase<ForcePairType, ForceScoreCalcConfig, MatrixLessType, AllocatorType>;
	using Self  = TForceScoreCalculator;
// using PointSetData         = ;
// using PointSetDataPtr      = std::shared_ptr<TSearchSpaceElement<ForceCount>>;
protected:
	POINT_SET_DATA_TYPE_ALIAS(ForceCount);
	DEF_MATRIX_TYPES_ALIASES(Scalar);

	using ForcePairAllocType = Eigen::aligned_allocator<ForcePairType>;
	using ForcePairListType  = std::vector<ForcePairType, ForcePairAllocType>;

public:
	TForceScoreCalculator(const ForceScoreCalcConfig& InConfigJsonObj, const size_t InDataSize, PointSetDataPtr InPointSetDataPtr, EvaluationStaticData& InStaticData, SPDLog::LoggerType InLogger = nullptr)
		: OperatingPointSetDataPtr(InPointSetDataPtr), Super(InConfigJsonObj, InDataSize, InStaticData, InLogger)
	{
	}

	virtual void CalcRawScore(const ForcePairType& InDataPtr, int InCurrentDataIdx) override
	{
		// Paper 5.4 Section: Force Evaluation function.
		// 1. Calculate [Force] norm score.
		// The [-] sign makes the score meaning to be the smaller the better.
		const Scalar ForceIntensityScore = -InDataPtr.rowwise().norm().sum();

		// 2. Calculate [Force] direction score.
		Eigen::Matrix<Scalar, ForceCount, 3> ForceDir = InDataPtr;
		ForceDir = ForceDir.rowwise().normalized();  // Use normalized() instead of normalize()
		// const auto& /* Vec3 */ ForceDir = InDataPtr.rowwise().normalize();
		const auto& /* Vec3 */ KnifePlaneNormalDir = this->StaticData.KnifePose.template block<3, 1>(0, 2);
		const Scalar ForceDirScore = (ForceDir * KnifePlaneNormalDir).sum();
		/**
		 * @brief Calculates the scalar product between force direction and normal vectors
		 * 
		 * Matrix Operation Breakdown:
		 * 1. ForceDir [nx3] array and NormalPair [nx3] array undergo element-wise multiplication
		 * 2. rowwise().sum() reduces each row to a single value (dot product per row)
		 * 3. final sum() aggregates all row results into a single scalar
		 * 
		 * Example for single row (can be use for multiple rows):
		 * ForceDir    = [fx, fy, fz]
		 * NormalPair  = [nx, ny, nz]
		 * Result      = fx*nx + fy*ny + fz*nz
		 *
		 */
		// Step-by-step calculation for debugging
		// const auto ForceDirArray = ForceDir.array();
		// const auto NormalPairArray = OperatingPointSetDataPtr->NormalPair.array();
		// const Eigen::MatrixX3d ElementWiseProduct = ForceDirArray * NormalPairArray;
		// const Eigen::VectorXd RowwiseSum = ElementWiseProduct.rowwise().sum();
		// const Scalar ForceDirWithNormalScore = RowwiseSum.sum();
		const Scalar ForceDirWithNormalScore = (ForceDir.array() * OperatingPointSetDataPtr->NormalPair.array()).rowwise().sum().sum();

		// 3. Calculate [Force] length variance score.
		// Calculate the variance of the force lengths
		const Eigen::Matrix<Scalar, ForceCount, 1> ForceLengths = InDataPtr.rowwise().norm();
		const Scalar ForceMean = ForceLengths.mean();
		const Scalar ForceVarianceScore = (ForceLengths.array() - ForceMean).square().sum() / (ForceCount - 1);

		this->ScoreRawData.row (InCurrentDataIdx) << ForceIntensityScore, ForceDirScore, ForceVarianceScore;
	}

protected:



protected:
	PointSetDataPtr OperatingPointSetDataPtr;
};

#endif /* C369AAB0_0C1A_4490_A104_5A051204BB76 */
