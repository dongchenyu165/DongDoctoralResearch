#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "2_FilterByGeo/GeometryFilterScoreCalculator.hpp"
#include "3_KnifeForceCalculator/KnifeForceCalculator.hpp"
#include "DataTypes/PointSetData.hpp"
#include "GlobalBaseTypes.hpp"
#include "SearchSpaceGenerator/SearchSpaceGenerator.hpp"
#include "Utilities/JSON_Helper/StructSerializer.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudConverter.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudInfo.hpp"
#include <pcl/io/pcd_io.h>
#include <vector>
#include <tuple>
#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <GlobalTypes.hpp>
#include <Utilities/spdlog/LogConfig.hpp>

using namespace Types;
std::string gTempCalculationParamJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json";
json gParamJson;
SPDLog::LoggerType gLogger;


auto PrepareData(CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
{
	gLogger->info("PrepareData");

	CuttingFaceMaker Maker(gTempCalculationParamJsonPath, InTrajectoryNode);
	CuttingFaceResult CuttingFaceResultObj = Maker.MakeCuttingFace(InPC);

	// Calculate the center of mass of the input point cloud. (Use the average of all points as CoM)
	PCL_Helper::TPointCloudInfo<Types::CalcPoint> PC_InfoObj(InPC);
	CuttingFaceResultObj.StaticData.CenterOfMass = PC_InfoObj.GetPointsCenter().cast<CalcScalar>();

	SearchSpace InitSearchSpace;
	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(CuttingFaceResultObj.GraspingPC);
	size_t RealCombCount = SearchSpaceGenObj.Generate(InitSearchSpace);

	return std::make_tuple(CuttingFaceResultObj, InitSearchSpace);
}

SearchSpace FilterByGeoScore(SearchSpace InInitSearchSpace, CuttingFaceResult& InCuttingFaceResults, TrajectoryNode InTrajectoryNode)
{
	using EGettingMethod = GeometryFilterScoreCalculator::ReturnDataSelectorType::EMethod;
	GeoFilterScoreCalcConfig Param;
	JSON_Helper::LoadStructure_ByPath(gTempCalculationParamJsonPath, {"FilterByGeoScore", "GeoScoreWeight"}, Param);
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

ForceTorqueType CalKnifeForce(CalcPCPTR_List InCuttingPlanePC, TrajectoryNode InTrajectoryNode)
{
	FUNC_LOGGER_ENTER_CUSTOM_LOGGER(gLogger);
	KnifeForceCalculator Calculator(InCuttingFaceResultObj, gParamJson, gLogger);
	return Calculator.CalculateKnifeForce(InTrajectoryNode.Velocity);
}

float CalForceScore(ForceTorqueType InKnifeForce, HoldingPointSet InHoldingPointSet)
{
	return 0.0f;
}

// 
CalcPCPTR LoadPC(const std::string& InFilePath)
{
	gLogger->info("Load Point Cloud");
	NEW_PC_PTR(LoadedPC, PCL_Helper::PointXYZ);
	if (pcl::io::loadPCDFile<PCL_Helper::PointXYZ>(InFilePath, *LoadedPC) == -1) 
	{
		gLogger->error("Couldn't read file %s \n", InFilePath.c_str());
		return nullptr;
	}
	
	gLogger->info("Load Point Cloud with %d points", LoadedPC->size());
	CalcPCPTR FinalCalcPC = PCL_Helper::ConvertPointCloud<CalcPoint>(LoadedPC);  // , PCL_Helper::EConvertRGBField::Const, PCL_Helper::EConvertNormalField::Estimate
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
	Vec3 P1 = AABBCenter + Vec3(0.05, 0, 0.007);
	Vec3 P2 = AABBCenter + Vec3(-0.05, 0,  0.001);
	Vec3 P3 = AABBCenter + Vec3(0.05, 0, -0.008);
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
	gParamJson = json::parse(f, nullptr, true, true);

	Trajectory KnifeTrajectory = MakeTrajectory(OriginPC);

	for ( int i = 0; i < KnifeTrajectory.size(); i++ )
	{

		const auto& KnifeTrajectoryNode = KnifeTrajectory[i];

		auto [CuttingFaceResultObj, InitSearchSpace] = PrepareData(OriginPC, KnifeTrajectoryNode);

		SearchSpace MainSearchSpace = FilterByGeoScore(InitSearchSpace, CuttingFaceResultObj, KnifeTrajectoryNode);

		auto KnifeForce = CalKnifeForce(CuttingFaceResultObj, KnifeTrajectoryNode);

		gLogger->info("FINISHED, MainSearchSpace size: {}", MainSearchSpace.size());
		continue;
		for ( int j = 0; j < MainSearchSpace.size(); j++ )
		{
			HoldingPointSet& HoldingPointSet = MainSearchSpace[j].PositionPair;  // SearchSpacetType  $\mathbf{P}$

			float ForceScore = CalForceScore(KnifeForce, HoldingPointSet);
			
		}

	}

	return 0;
}