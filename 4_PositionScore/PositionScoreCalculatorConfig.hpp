#ifndef A44BD936_5E0B_4FDD_AE0E_222AE4B22B91
#define A44BD936_5E0B_4FDD_AE0E_222AE4B22B91

#include "ScoreCalculator/ScoreCalculatorConfig.hpp"

// CalcPointSetDataPtr

// This is the configuration class for the force score calculator.
// However, the method paper proposed DO NOT NEED any configuration.
// So this class is just remain ONLY weight member and empty extra member.
BEGIN_DEF_SCORE_CALCULATOR_CONFIG(PositionScore)
	// MEMBER_DEF_WITH_DEFAULT(float, DistanceLimit, 0.04);
	// MEMBER_DEF_WITH_DEFAULT(float, DistancePenality, 6);
	// SCORE_CALC_MEMBER_DEF(DistanceLimit, DistancePenality);
END_DEF_SCORE_CALCULATOR_CONFIG()

// 5 is the weight components ( should sync with the [params.json] settings) in the score.
using PositionScoreCalcConfig = TScoreCalculatorConfig_PositionScore<2>;  


#endif /* A44BD936_5E0B_4FDD_AE0E_222AE4B22B91 */
