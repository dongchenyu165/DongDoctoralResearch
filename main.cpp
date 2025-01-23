#include <omp.h>
#include <pcl/io/pcd_io.h>

#include "DataTypes/PointSetData.hpp"
#include "DataTypes/KnifeTrajectoryNode.hpp"
#include "GlobalBaseTypes.hpp"
#include "GlobalVars.hpp"
#include "GlobalTypes.hpp"

#include "Utilities/JSON_Helper/StructSerializer.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudConverter.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudInfo.hpp"
#include "Utilities/spdlog/FunctionAutoLogger.hpp"
#include "Utilities/spdlog/LogConfig.hpp"

#include "SearchSpaceGenerator/SearchSpaceGenerator.hpp"
#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "2_FilterByGeo/GeometryFilterScoreCalculator.hpp"
#include "3_KnifeForceCalculator/KnifeForceCalculator.hpp"
#include "4_ForceScore/FingerForceGenerator.hpp"
#include "4_ForceScore/ForceScoreCalculator.hpp"
#include "4_ForceScore/ForceScoreCalculatorConfig.hpp"
#include "4_PositionScore/PositionScoreCalculator.hpp"
#include "4_PositionScore/PositionScoreCalculatorConfig.hpp"


using namespace Types;
std::string gTempCalculationParamJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json";
nlohmann::json gParamJson;
SPDLog::LoggerType gLogger;


auto PrepareData(CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
{
	gLogger->info("PrepareData");

	CuttingFaceMaker Maker(gTempCalculationParamJsonPath, InTrajectoryNode);
	CuttingFaceResult CuttingFaceResultObj = Maker.MakeCuttingFace(InPC);
	CuttingFaceResultObj.StaticData.GraspingPC = CuttingFaceResultObj.GraspingPC;

	// Calculate the center of mass of the input point cloud. (Use the average of all points as CoM)
	PCL_Helper::TPointCloudInfo<Types::CalcPoint> PC_InfoObj(InPC);
	CuttingFaceResultObj.StaticData.CenterOfMass = PC_InfoObj.GetPointsCenter().cast<CalcScalar>();

	SearchSpaceType InitSearchSpace;
	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(CuttingFaceResultObj.GraspingPC);
	size_t RealCombCount = SearchSpaceGenObj.Generate(InitSearchSpace);

	return std::make_tuple(CuttingFaceResultObj, InitSearchSpace);
}

SearchSpaceType FilterByGeoScore(SearchSpaceType InInitSearchSpace, CuttingFaceResult& InCuttingFaceResults, TrajectoryNode InTrajectoryNode)
{
	using EGettingMethod = GeometryFilterScoreCalculator::ReturnDataSelectorType::EMethod;
	static GeoFilterScoreCalcConfig Param;
	static bool bIsParamLoaded = false;
	if ( !bIsParamLoaded )
	{
		JSON_Helper::LoadStructure_ByPath<GeoFilterScoreCalcConfig, nlohmann::ordered_json>(gTempCalculationParamJsonPath, { "FilterByGeoScore", "GeoScoreWeight" },
			Param);			bIsParamLoaded = true;
	}
	GeometryFilterScoreCalculator Filter(Param, InInitSearchSpace.size(), InCuttingFaceResults.StaticData, gLogger);

	// Algorithm2 5-9 rows. 
	// Loop all of the InitSearchSpace
	// 	Calculate the score of each score of point-set data.
	//  Sort the data by the score.
	Filter.CalculateScore(InInitSearchSpace);

	// Algorithm2 10 row.
	// Get the front [GeoFilterRatio] ratio of the data. In paper is [rN]
	float GeoFilterRatio = gParamJson["FilterByGeoScore"]["Filter"]["Ratio"];
	// The method [Good] means the top [GeoFilterRatio] ratio of the data.
	return Filter.GetFinalDataList(EGettingMethod::Good, Filter.Size() * GeoFilterRatio);
}

ForceTorqueType CalKnifeForce(CuttingFaceResult& InCuttingFaceResultObj, TrajectoryNode InTrajectoryNode)
{
	FUNC_LOGGER_ENTER_CUSTOM_LOGGER(gLogger);
	KnifeForceCalculator Calculator(InCuttingFaceResultObj, gParamJson, gLogger);
	return Calculator.CalculateKnifeForce(InTrajectoryNode.Velocity);
}

CalcScalar CalForceScore(ForceTorqueType InKnifeForce, EvaluationStaticData& InStaticData, CalcPointSetDataPtr InPointSetDataPtr)
{
	static bool bIsParamLoaded = false;
	static ForceScoreCalcConfig ForceScoreCalcParam;

	int ThreadID = omp_get_thread_num();
	SPDLog::LoggerType InternalLogger = SPDLog::LoggerManager::GetSubLogger(gLogger->name(), "CalForceScore_" + std::to_string(ThreadID));
	InternalLogger->set_level(spdlog::level::warn);

	using GeneratorType = TFingerForceGeneratorWithinCone<double, FINGER_NUMBER>;
	using GeneratorBaseType = GeneratorType::Super;
	
	// Use GeneratorType object to generate a finger force list.
	std::shared_ptr<GeneratorBaseType> GeneratorPtr =
			std::make_shared<GeneratorType>(InKnifeForce, InStaticData.CenterOfMass, InPointSetDataPtr, 1800, gParamJson, gLogger);  // TODO: ["CalForceScore"]
	size_t SucceedCnt = GeneratorPtr->GenerateFingerForceList();
	if (SucceedCnt == 0)
	{
		gLogger->trace("No finger force generated. point index: {}", InPointSetDataPtr->PointIndexPair);
		return -std::numeric_limits<CalcScalar>::infinity();
	}
	InternalLogger->info("{} finger force generated.", SucceedCnt);

	GeneratorType::ForcePairListType& Result = GeneratorPtr->GetGeneratedFingerForceList();

	// Sort and select the top score of the data.
	if ( !bIsParamLoaded )
	{
		JSON_Helper::LoadStructure_ByPath<ForceScoreCalcConfig, nlohmann::ordered_json>(gTempCalculationParamJsonPath,
			{ "CalForceScore", "Weight" }, ForceScoreCalcParam);
	}
	using ForceCalculatorType = TForceScoreCalculator<CalcScalar, FINGER_NUMBER>;
	using EGettingMethod = ForceCalculatorType::ReturnDataSelectorType::EMethod;
	ForceCalculatorType ForceScoreCalculator(ForceScoreCalcParam, Result.size(), InPointSetDataPtr, InStaticData, InternalLogger);
	ForceScoreCalculator.CalculateScore(Result);
	const ForceCalculatorType::ForcePairType ForceResult = ForceScoreCalculator.GetFinalDataList(EGettingMethod::Good, 1)[0];

	const CalcScalar ForceScore = ForceScoreCalculator.GetScore(ForceResult);
	
	return ForceScore;
}

// 
CalcPCPTR LoadPC(const std::string& InFilePath)
{
	gLogger->info("Load Point Cloud from file: {}", InFilePath);
	NEW_PC_PTR(LoadedPC, PCL_Helper::PointXYZ);
	if (pcl::io::loadPCDFile<PCL_Helper::PointXYZ>(InFilePath, *LoadedPC) == -1) 
	{
		gLogger->error("Couldn't read file %s \n", InFilePath.c_str());
		return nullptr;
	}
	
	gLogger->info("Load Point Cloud with {} points", LoadedPC->size());
	CalcPCPTR FinalCalcPC = PCL_Helper::ConvertPointCloud<CalcPoint,
		PCL_Helper::EConvertRGBField::Const,
		PCL_Helper::EConvertNormalField::Estimate>(LoadedPC);  // , PCL_Helper::EConvertRGBField::Const, PCL_Helper::EConvertNormalField::Estimate
	gLogger->info("Convert to calculating point cloud.");

	return FinalCalcPC;
}

Trajectory MakeTrajectory(CalcPCPTR InPC)
{
	PCL_Helper::TPointCloudInfo<Types::CalcPoint> PC_InfoObj(InPC);
	Mat4x4 TestPose = Mat4x4::Identity();
	TestPose.block<3, 1>(0, 0) = Vec3(-1, 0, 0);
	TestPose.block<3, 1>(0, 1) = Vec3(0, 0, 1);
	TestPose.block<3, 1>(0, 2) = Vec3(0, 1, 0);

	Vec3 AABBCenter = PC_InfoObj.GetAABBCenter().cast<Types::CalcScalar>();
	Vec3 P1 = AABBCenter + Vec3(0.05, 0, 0.004);
	Vec3 P2 = AABBCenter + Vec3(-0.05, 0,  0.001);
	Vec3 P3 = AABBCenter + Vec3(0.05, 0, -0.003);
	SPDLog::Log_T(gLogger, 2, "AABBCenter: [{}] P1: [{}], P2: [{}]", AABBCenter, P1, P2);

	Trajectory TestTrajectory;
	for (const Vec3& Pt : {P1, P2, P3})
	{
		Mat4x4 pose = TestPose;
		pose.block<3, 1>(0, 3) = Pt;
		TestTrajectory.push_back({pose, Vec3::Zero()});
	}
	CalculateVelocity(TestTrajectory, {0.01, 0.01, 0.01});
	
	return TestTrajectory;
}

int main()
{
	gLogger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("Global", LogConfigJsonPath);

	auto OriginPC = LoadPC("./points.pcd");

	std::ifstream f(gTempCalculationParamJsonPath);
	gParamJson = nlohmann::ordered_json::parse(f, nullptr, true, true);

	Trajectory KnifeTrajectory = MakeTrajectory(OriginPC);

	// This is the dictionary mentioned in the paper Algorithm 1.
	// std::map<HoldingPointSet, Types::CalcScalar> HoldingPointSetScoreMap;
	std::map<CalcPointSetDataPtr, Types::CalcScalar> ScoreMap;

	for ( int i = 0; i < KnifeTrajectory.size(); i++ )
	{

		const auto& KnifeTrajectoryNode = KnifeTrajectory[i];

		auto [CuttingFaceResultObj, InitSearchSpace] = PrepareData(OriginPC, KnifeTrajectoryNode);

		SearchSpaceType MainSearchSpace = FilterByGeoScore(InitSearchSpace, CuttingFaceResultObj, KnifeTrajectoryNode);

		auto KnifeForce = CalKnifeForce(CuttingFaceResultObj, KnifeTrajectoryNode);

		gLogger->info("FINISHED, MainSearchSpace size: {}", MainSearchSpace.size());
		continue;
		for ( int j = 0; j < MainSearchSpace.size(); j++ )
		{
			Types::CalcScalar ForceScore = CalForceScore(KnifeForce, MainSearchSpace[j]);

			const Types::CalcScalar& PositionScore = MainSearchSpace[j]->GeoScore;

			const Types::CalcScalar PointSetScore = PositionScore + ForceScore;
			if (ForceScore != -INFINITY)
			{
				auto&& SearchSpaceElement = ScoreMap.find(MainSearchSpace[j]);
				if (SearchSpaceElement == ScoreMap.end())
				{
					ScoreMap.insert({MainSearchSpace[j], PointSetScore});
				}
				else
				{
					SearchSpaceElement->second += PointSetScore;
				}
			}
		}

	}

	// Find the best HoldingPointSet which has the highest score.

	return 0;
}