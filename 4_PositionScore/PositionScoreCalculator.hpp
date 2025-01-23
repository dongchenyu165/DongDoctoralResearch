#ifndef D13989BD_746A_407D_81CB_22605E0B8834
#define D13989BD_746A_407D_81CB_22605E0B8834

// CalcPointSetDataPtr

#include <Eigen/Dense>
#include <Eigen/src/Core/DenseBase.h>
#include "GlobalTypes.hpp"
#include "GlobalBaseTypes.hpp"
#include "4_PositionScore/PositionScoreCalculatorConfig.hpp"
#include "ScoreCalculator/ScoreCalculatorBase.hpp"


template<typename Scalar, int ForceCount>
class TPositionScoreCalculator : public TScoreCalculatorBase<std::shared_ptr<TSearchSpaceElement<ForceCount>>, PositionScoreCalcConfig>
{
protected:
	using ForcePairType      = Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor>;
private:
	using Super = TScoreCalculatorBase<std::shared_ptr<TSearchSpaceElement<ForceCount>>, PositionScoreCalcConfig>;
	using Self  = TPositionScoreCalculator;
protected:
	POINT_SET_DATA_TYPE_ALIAS(ForceCount);
	DEF_MATRIX_TYPES_ALIASES(Scalar);

	using ForcePairAllocType = Eigen::aligned_allocator<ForcePairType>;
	using ForcePairListType  = std::vector<ForcePairType, ForcePairAllocType>;

public:
	TPositionScoreCalculator(const PositionScoreCalcConfig& InConfigJsonObj, const size_t InDataSize, PointSetDataPtr InPointSetDataPtr, EvaluationStaticData& InStaticData, SPDLog::LoggerType InLogger = nullptr)
		: OperatingPointSetDataPtr(InPointSetDataPtr), Super(InConfigJsonObj, InDataSize, InStaticData, InLogger)
	{
	}

	virtual std::tuple<double, double> GetNormalizedMinMax(const int InScoreComponentIdx) override
	{
		constexpr int DIRECTION_SCORE_IDX = 0;
		constexpr int DISTANCE_SCORE_IDX = 1;
		if ( InScoreComponentIdx == DIRECTION_SCORE_IDX )
		{
			return Super::GetNormalizedMinMax(DIRECTION_SCORE_IDX);
		}
		else if ( InScoreComponentIdx == DISTANCE_SCORE_IDX )
		{
			const auto KnifePlaneNormal = this->StaticData.KnifePose.template block<3, 1>(0, 2);
			const auto KnifePlanePoint  = this->StaticData.KnifePose.template block<3, 1>(0, 3);
			const auto PointMat = this->StaticData.GraspingPC->getMatrixXfMap().leftCols(3).template cast<Scalar>();
			const auto Distance =
				((PointMat.rowwise() - KnifePlanePoint.transpose()) * KnifePlaneNormal.normalized()).cwiseAbs();
			return { Distance.maxCoeff(), Distance.minCoeff() };
		}
		else
		{
			assert(false);
			return { 0.0, 0.0 };
		}
	}

	virtual void CalcRawScore(const Types::CalcPointSetDataPtr& InDataPtr, int InCurrentDataIdx) override
	{
		// Paper 5.5 Section: Position Evaluation function.
		// 1. Calculate [Direction] Score.
		// Using the PCA to calculate the direction of points.
		Vec3 PCA_Dir = CalcPCADirection(InDataPtr->PositionPair);
		// Then, calculate the angle between the direction and the knife direction.
		const auto KnifePlaneNormalDir = this->StaticData.KnifePose.template block<3, 1>(0, 2);
		// Scalar DirectionScore = std::abs(PCA_Dir.dot(KnifePlaneNormalDir));
		Scalar DirectionScore = 1 - PCA_Dir.dot(KnifePlaneNormalDir);

		// 2. Calculate [Distance to knife] Score.
		// Calculate the distance between the points and the knife's XoY plane
		const auto KnifePos = this->StaticData.KnifePose.template block<3, 1>(0, 3);
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Distances(InDataPtr->PositionPair.rows());
		// for ( int i = 0; i < InDataPtr->PositionPair.rows(); ++i )
		// {
		// 	auto& Point   = InDataPtr->PositionPair.row(i).transpose();
		// 	Distances(i) = std::abs((Point - KnifePos).dot(KnifePlaneNormalDir));
		// }
		// Replace the for loop with:
		Distances =
			((InDataPtr->PositionPair.rowwise() - KnifePos.transpose()) * KnifePlaneNormalDir)
				.array()
				.abs();
		// Use the minimum distance as the score
		Scalar DistanceScore = Distances.minCoeff();


		this->ScoreRawData.row (InCurrentDataIdx) << DirectionScore, DistanceScore;
	}

protected:
	static Vec3 CalcPCADirection(Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor> InPoints)
	{
		const Vec3 Mean = InPoints.colwise().mean();
		const decltype(InPoints) CenteredPoints = InPoints.rowwise() - Mean.transpose();
		const Eigen::Matrix3d Covariance = (CenteredPoints.transpose() * CenteredPoints) / double(InPoints.rows() - 1);

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> EigenCalcObj(Covariance);
		
		// The eigenvalues are sorted in increasing order.
		// col 2 means the eigenvector with largest eigenvalue
		return EigenCalcObj.eigenvectors().col(2);  
	}

protected:
	PointSetDataPtr OperatingPointSetDataPtr;
};



#endif /* D13989BD_746A_407D_81CB_22605E0B8834 */
