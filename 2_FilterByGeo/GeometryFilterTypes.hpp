#ifndef DE0F7908_237F_49CC_9615_573EA91E7350
#define DE0F7908_237F_49CC_9615_573EA91E7350

#include "GlobalTypes.hpp"
#include "2_FilterByGeo/GeoScoreCalcConfig.hpp"
#include "ScoreCalculator/ScoreCalculator2D.hpp"

/* ------------------------ Filter by Geometry Score ------------------------ */
constexpr int GeoScoreComponentCount = 4;
using GeoScoreCalcConfigType = TScoreCalculatorConfig_Geo<GeoScoreComponentCount>;
using GeoScoreCalcDataType = TScoreCalculator2D<Types::CalcPointSetData, GeoScoreCalcConfigType, GeoScoreComponentCount>;


#endif /* DE0F7908_237F_49CC_9615_573EA91E7350 */
