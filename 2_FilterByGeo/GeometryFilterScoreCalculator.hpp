#ifndef CC84BF01_CAE8_4084_BDC6_79F26687E9AF
#define CC84BF01_CAE8_4084_BDC6_79F26687E9AF


#include "GlobalTypes.hpp"
#include "GlobalBaseTypes.hpp"
#include "2_FilterByGeo/GeoScoreCalcConfig.hpp"
#include "ScoreCalculator/ScoreCalculatorBase.hpp"
#include "Utilities/PCL_Helper/Calculations/PointsPCADirection.hpp"

class GeometryFilterScoreCalculator : public TScoreCalculatorBase<Types::CalcPointSetDataPtr, GeoFilterScoreCalcConfig>
{
	using Super = TScoreCalculatorBase<Types::CalcPointSetDataPtr, GeoFilterScoreCalcConfig>;
	using Self  = GeometryFilterScoreCalculator;

public:
	GeometryFilterScoreCalculator(const GeoFilterScoreCalcConfig& InConfigJsonObj, const size_t InDataSize, EvaluationStaticData& InStaticData, SPDLog::LoggerType InLogger = nullptr)
		: Super(InConfigJsonObj, InDataSize, InStaticData, InLogger)
	{
	}	

	void __OLD__CalcRawScore(const Types::CalcPointSetDataPtr& InDataPtr, int InCurrentDataIdx)
	{
		Types::ConstVec3& P1 = InDataPtr->PositionPair.row (0);
		Types::ConstVec3& P2 = InDataPtr->PositionPair.row (1);
		Types::ConstVec3& N1 = InDataPtr->NormalPair.row (0);
		Types::ConstVec3& N2 = InDataPtr->NormalPair.row (1);

		Types::CalcScalar RawDistanceScore = 1;
		Types::CalcScalar Distance			= (P2 - P1).norm ();
		if ( Distance > ConfigData.DistanceLimit )
		{
			// The contribution of the distance should have an upper limit 
			// 		to prevent selecting points [near the knife].
			RawDistanceScore = 1;
		}
		else
		{
			RawDistanceScore = (exp (ConfigData.DistancePenality * Distance / ConfigData.DistanceLimit) - 1)
								/ 
							(exp (ConfigData.DistancePenality) - 1)
							;
		}

		
		Types::CalcScalar KnifeYPos = StaticData.KnifePose (1, 3);
		Types::CalcScalar RawNormalAngleScore = std::min(abs(P1.y() - KnifeYPos), abs(P2.y() - KnifeYPos));

		Types::Vec3 Dir		   = (P2 - P1).normalized ();
		Types::CalcScalar SameAsKnifeDirAngleScore = abs (StaticData.KnifePose.block<3, 1> (0, 0).dot (Dir));

		ScoreRawData.row (InCurrentDataIdx) << RawDistanceScore, RawNormalAngleScore, SameAsKnifeDirAngleScore, std::min (P1.z (), P2.z ());
	}

	virtual void CalcRawScore(const Types::CalcPointSetDataPtr& InDataPtr, int InCurrentDataIdx) override
	{
		if (InDataPtr->PositionPair.hasNaN() || InDataPtr->NormalPair.hasNaN())
		{
			ScoreRawData.row(InCurrentDataIdx) << -INFINITY, -INFINITY, -INFINITY, -INFINITY;
			return;
		}

		// 1. Finger position distance score (FAR is better)
		Types::CalcScalar RawDistanceScore = 1;
		// 1.1 Find the min distance between inner points storaged in each row of [PositionPair]
		Types::CalcScalar FingersMinDistance			= std::numeric_limits<Types::CalcScalar>::max();
		for (int i = 0; i < InDataPtr->PositionPair.rows(); ++i)
		{
			Types::ConstVec3& P1 = InDataPtr->PositionPair.row(i);

			for (int j = i + 1; j < InDataPtr->PositionPair.rows(); ++j)
			{
				Types::ConstVec3& P2 = InDataPtr->PositionPair.row(j);
				Types::CalcScalar FingerDistance = (P2 - P1).norm();
				if (FingerDistance < FingersMinDistance)
				{
					FingersMinDistance = FingerDistance;
				}
			}
		}
		// 1.2 Calculate the score based on the min fingers' distance
		if ( FingersMinDistance > ConfigData.DistanceLimit )
		{
			// The contribution of the distance should have an upper limit 
			// 		to prevent selecting points [near the knife].
			RawDistanceScore = 1;
		}
		else
		{
			RawDistanceScore = (exp (ConfigData.DistancePenality * FingersMinDistance / ConfigData.DistanceLimit) - 1)
								/ 
							(exp (ConfigData.DistancePenality) - 1)
							;
		}

		// 2. Score of the closest finger to the knife. (FAR is better)
		Types::CalcScalar KnifeYPos = StaticData.KnifePose (1, 3);
		Types::CalcScalar KnifeClosestFingerPos = (InDataPtr->PositionPair.col(2).array() - KnifeYPos).abs().minCoeff();

		// 3. Score of the direction of the finger (Same direction as the knife's blade is better)
		Types::Vec3 Dir		   = PCL_Helper::CalcPCADirection(InDataPtr->PositionPair);
		Types::CalcScalar SameAsKnifeDirAngleScore = abs (StaticData.KnifePose.block<3, 1> (0, 0).dot (Dir));

		ScoreRawData.row (InCurrentDataIdx) << RawDistanceScore, KnifeClosestFingerPos, SameAsKnifeDirAngleScore, InDataPtr->PositionPair.col(2).minCoeff();
	}
};

#endif /* CC84BF01_CAE8_4084_BDC6_79F26687E9AF */
