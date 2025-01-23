#ifndef DCF9AC53_ABDB_4FD0_9E65_16680B5491B1
#define DCF9AC53_ABDB_4FD0_9E65_16680B5491B1

#include "ScoreCalculator/ScoreCalculatorConfig.hpp"

// CalcPointSetDataPtr

// This is the configuration class for the force score calculator.
// However, the method paper proposed DO NOT NEED any configuration.
// So this class is just remain ONLY weight member and empty extra member.
BEGIN_DEF_SCORE_CALCULATOR_CONFIG(ForceScore)
	// MEMBER_DEF_WITH_DEFAULT(float, DistanceLimit, 0.04);
	// MEMBER_DEF_WITH_DEFAULT(float, DistancePenality, 6);
	// SCORE_CALC_MEMBER_DEF(DistanceLimit, DistancePenality);
END_DEF_SCORE_CALCULATOR_CONFIG()

// 5 is the weight components ( should sync with the [params.json] settings) in the score.
using ForceScoreCalcConfig = TScoreCalculatorConfig_ForceScore<3>;  

#endif /* DCF9AC53_ABDB_4FD0_9E65_16680B5491B1 */
