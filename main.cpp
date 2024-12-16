#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "2_FilterByGeo/GeometryFilterScoreCalculator.hpp"
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

// #include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <GlobalTypes.hpp>
#include <Utilities/spdlog/LogConfig.hpp>


using namespace Types;
std::string gTempCalculationParamJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json";
json gParamJson;

auto PrepareData(CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
{
	SearchSpace InitSearchSpace;
	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(InPC);
	size_t RealCombCount = SearchSpaceGenObj.Generate(InitSearchSpace);

	CuttingFaceMaker Maker(gTempCalculationParamJsonPath, InTrajectoryNode);
	CuttingFaceResult CuttingFaceResultObj = Maker.MakeCuttingFace(InPC);

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

ForceTorque CalKnifeForce(CalcPCPTR_List InCuttingPlanePC, TrajectoryNode InTrajectoryNode)
{
	return ForceTorque::Zero(6, 1);
}

float CalForceScore(ForceTorque InKnifeForce, HoldingPointSet InHoldingPointSet)
{
	return 0.0f;
}

// 
CalcPCPTR LoadPC(const std::string& InFilePath)
{
	NEW_PC_PTR(LoadedPC, PCL_Helper::PointXYZ);
	if (pcl::io::loadPCDFile<PCL_Helper::PointXYZ>(InFilePath, *LoadedPC) == -1) 
	{
		PCL_ERROR("Couldn't read file %s \n", InFilePath.c_str());
		return nullptr;
	}
	
	CalcPCPTR FinalCalcPC = PCL_Helper::ConvertPointCloud<CalcPoint>(LoadedPC);
	return FinalCalcPC;
}

Trajectory MakeTrajectory(CalcPCPTR InPC)
{
	PCL_Helper::TPointCloudInfo<Types::CalcPoint> PC_InfoObj(InPC);
	Mat4x4 TestPose = Mat4x4::Identity();
	TestPose.block<3, 1>(0, 0) = Vec3(-1, 0, 0);
	TestPose.block<3, 1>(0, 1) = Vec3(0, 0, 1);
	TestPose.block<3, 1>(0, 2) = Vec3(0, 1, 0);

	auto AABBCenter = PC_InfoObj.GetAABBCenter().cast<Types::CalcScalar>();
	Vec3 P1 = AABBCenter + Vec3(-0.05, 0, 0.02);
	Vec3 P2 = AABBCenter + Vec3(0.05, 0, -0.02);

	Trajectory TestTrajectory;
	Mat4x4 Pose1 = TestPose;
	Pose1.block<3, 1>(0, 3) = P1;
	TestTrajectory.push_back({Pose1, Vec3(0, 0, 0)});
	Mat4x4 Pose2 = TestPose;
	Pose2.block<3, 1>(0, 3) = P2;
	TestTrajectory.push_back({Pose2, Vec3(0, 0, 0)});
	
	return TestTrajectory;
}

int main()
{
	auto OriginPC = LoadPC("./points.pcd");

	std::ifstream f(gTempCalculationParamJsonPath);
	gParamJson = json::parse(f, nullptr, true, true);

	Trajectory KnifeTrajectory = MakeTrajectory(OriginPC);

	for ( int i = 0; i < KnifeTrajectory.size(); i++ )
	{

		const auto& KnifeTrajectoryNode = KnifeTrajectory[i];

		auto [CuttingFaceResultObj, InitSearchSpace] = PrepareData(OriginPC, KnifeTrajectoryNode);

		SearchSpace MainSearchSpace = FilterByGeoScore(InitSearchSpace, CuttingFaceResultObj, KnifeTrajectoryNode);

		auto KnifeForce = CalKnifeForce({CuttingFaceResultObj.CuttingFacePC_P, CuttingFaceResultObj.CuttingFacePC_N}, KnifeTrajectoryNode);

		for ( int j = 0; j < MainSearchSpace.size(); j++ )
		{
			HoldingPointSet& HoldingPointSet = MainSearchSpace[j].PositionPair;  // SearchSpacetType  $\mathbf{P}$

			float ForceScore = CalForceScore(KnifeForce, HoldingPointSet);
			
		}

	}

	return 0;
}