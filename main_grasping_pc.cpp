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
#include "Utilities/spdlog/LogConfig.hpp"
#include "Utilities/CustomHashAndCmp.hpp"

#include "SearchSpaceGenerator/SearchSpaceGenerator.hpp"
#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "2_FilterByGeo/GeometryFilterScoreCalculator.hpp"


using namespace Types;
std::string gTempCalculationParamJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json";
nlohmann::json gParamJson;
SPDLog::LoggerType gLogger;

template<typename PointT>
void ReorderPointCloud(const typename pcl::PointCloud<PointT>::Ptr& CloudA,
	typename pcl::PointCloud<PointT>::Ptr& CloudB)
{
	// Create KD-tree for Cloud B
	pcl::KdTreeFLANN<PointT> KdTree;
	KdTree.setInputCloud(CloudB);

	// Temporary cloud to store reordered points
	pcl::PointCloud<PointT> TempCloud;
	TempCloud.points.resize(CloudB->points.size());

	// For each point in Cloud A
	for ( size_t i = 0; i < CloudA->points.size(); ++i )
	{
		std::vector<int> PointIdxNKNSearch(1);
		std::vector<float> PointNKNSquaredDistance(1);

		// Find nearest neighbor in Cloud B
		if ( KdTree.nearestKSearch(CloudA->points[i], 1, PointIdxNKNSearch, PointNKNSquaredDistance) > 0 )
		{
			// Copy point to new position
			TempCloud.points[i] = CloudB->points[PointIdxNKNSearch[0]];
		}
	}

	// Replace Cloud B with reordered points
	*CloudB = TempCloud;
}

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

template<typename Scalar>
inline bool IsValidFloat(Scalar InValue)
{
	return !std::isnan(InValue) && !std::isinf(InValue) && InValue > -std::numeric_limits<Scalar>::max() &&
	       InValue < std::numeric_limits<Scalar>::max();
}

int main2()
{
	gLogger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("Global", LogConfigJsonPath);

	auto OriginPC = LoadPC("./points.pcd");

	std::ifstream f(gTempCalculationParamJsonPath);
	gParamJson = nlohmann::ordered_json::parse(f, nullptr, true, true);
	gLogger->trace("Loaded JSON parameters: {}", gParamJson.dump(4));

	Trajectory KnifeTrajectory = MakeTrajectory(OriginPC);

	size_t FinalValidCount = 0;
	// This is the dictionary mentioned in the paper Algorithm 1.
	// std::map<HoldingPointSet, Types::CalcScalar> HoldingPointSetScoreMap;
	std::unordered_map<CalcPointSetDataPtr, Types::CalcScalar, Utilities::TCalcPointSetDataPtrHash<CalcPointSetDataPtr, FINGER_NUMBER>, Utilities::TCalcPointSetDataPtrEqual<CalcPointSetDataPtr>> ScoreMap;

	// TESTING: Checking difference between each trajectory node.
	using PointSetMap = std::unordered_map<CalcPointSetDataPtr, CalcPointSetDataPtr, Utilities::TCalcPointSetDataPtrPointPosHash<CalcPointSetDataPtr, FINGER_NUMBER>, Utilities::TCalcPointSetDataPtrEqual<CalcPointSetDataPtr>>;
	std::vector<PointSetMap> SearchSpaceList(KnifeTrajectory.size());

	std::vector<CalcPCPTR> GraspingPCList;

	for ( int i = 0; i < KnifeTrajectory.size(); i++ )
	{
		gLogger->set_level(spdlog::level::info);
		const auto& KnifeTrajectoryNode = KnifeTrajectory[i];

		auto [CuttingFaceResultObj, InitSearchSpace] = PrepareData(OriginPC, KnifeTrajectoryNode);
		GraspingPCList.push_back(CuttingFaceResultObj.GraspingPC);

		// save CuttingFaceResultObj.GraspingPC
		pcl::io::savePCDFileASCII(fmt::format("GraspingPC_{}.pcd", i), *CuttingFaceResultObj.GraspingPC);

		SearchSpaceType MainSearchSpace = FilterByGeoScore(InitSearchSpace, CuttingFaceResultObj, KnifeTrajectoryNode);

		for (int j = 0; j < MainSearchSpace.size(); j++)
		{
			SearchSpaceList[i][MainSearchSpace[j]] = MainSearchSpace[j];
		}
		// SearchSpaceList.push_back(MainSearchSpace);

	}

	// Compare the points in each PC in GraspingPCList.
	for (int i = 0; i < GraspingPCList.size() - 1; i++)
	{
		const auto& PC1 = GraspingPCList[i];
		const auto& PC2 = GraspingPCList[i + 1];

		for (int j = 0; j < PC1->size(); j++)
		{
			const auto& Point1 = (*PC1)[j].getVector3fMap();
			const auto& Point2 = (*PC2)[j].getVector3fMap();

			if (!Point1.isApprox(Point2, 1e-6))
			{
				gLogger->error("Point position difference in next node. Trajectory Node: {}, Point Index: {}", i, j);
			}
		}
	}
	exit(0);

	// Compare the search space elements between each trajectory node.
	// for (int i = 0; i < KnifeTrajectory.size() - 1; i++)
	{
		int i = 0;
		const auto& SearchSpace1 = SearchSpaceList[i];
		const auto& SearchSpace2 = SearchSpaceList[i + 1];

		for (const auto& [Key, Value] : SearchSpace1)
		{
			if (SearchSpace2.find(Key) == SearchSpace2.end())
			{
				gLogger->error("Search space element (((NOT FOUND))) in next node. Trajectory Node: {}, Point Index: {}", i, Key->PointIndexPair.transpose());
				continue;
			}

			const auto& Value2 = SearchSpace2.at(Key);
			if (*Value != *Value2)
			{
				gLogger->error("Search space element <<<EQUAL>>> in next node. Trajectory Node: {}, Point Index: {}", i, Key->PointIndexPair.transpose());
			}
		}
	}
}
