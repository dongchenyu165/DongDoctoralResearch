#ifndef CC84BF01_CAE8_4084_BDC6_79F26687E9AF
#define CC84BF01_CAE8_4084_BDC6_79F26687E9AF


#include "GlobalTypes.hpp"
#include "GlobalBaseTypes.hpp"
#include "2_FilterByGeo/GeoScoreCalcConfig.hpp"
#include "ScoreCalculator/ScoreCalculatorBase.hpp"

class GeometryFilterScoreCalculator : public TScoreCalculatorBase<Types::CalcPointSetDataPtr, GeoFilterScoreCalcConfig>
{
	using Super = TScoreCalculatorBase<Types::CalcPointSetDataPtr, GeoFilterScoreCalcConfig>;
	using Self  = GeometryFilterScoreCalculator;

public:
	GeometryFilterScoreCalculator(const GeoFilterScoreCalcConfig& InConfigJsonObj, const size_t InDataSize, EvaluationStaticData& InStaticData, SPDLog::LoggerType InLogger = nullptr)
		: Super(InConfigJsonObj, InDataSize, InStaticData, InLogger)
	{
	}

	virtual void CalcRawScore(const Types::CalcPointSetDataPtr& InDataPtr, int InCurrentDataIdx) override
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
};

#endif /* CC84BF01_CAE8_4084_BDC6_79F26687E9AF */
