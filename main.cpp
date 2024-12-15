#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "2_FilterByGeo/GeometryFilterScoreCalculator.hpp"
#include "DataTypes/PointSetData.hpp"
#include "GlobalBaseTypes.hpp"
#include "SearchSpaceGenerator/SearchSpaceGenerator.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudConverter.hpp"
#include <pcl/io/pcd_io.h>
#include <vector>
#include <tuple>

// #include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <GlobalTypes.hpp>
#include <Utilities/spdlog/LogConfig.hpp>


using namespace Types;
std::string gTempCalculationParamJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json";


auto PrepareData(CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
{
	SearchSpace InitSearchSpace;
	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(InPC);
	size_t RealCombCount = SearchSpaceGenObj.Generate(InitSearchSpace);

	CuttingFaceMaker Maker(gTempCalculationParamJsonPath, InTrajectoryNode);
	CuttingFaceResult CuttingFaceResultObj = Maker.MakeCuttingFace(InPC);

	return std::make_tuple(CuttingFaceResultObj, InitSearchSpace);
}

// template<typename Scalar>
SearchSpace FilterByGeoScore(SearchSpace InInitSearchSpace, CuttingFaceResult& InCuttingFaceResults, TrajectoryNode InTrajectoryNode)
{
	GeoFilterScoreCalcConfig Param;
	JSON_Helper::LoadStructure_ByPath(gTempCalculationParamJsonPath, {"FilterByGeoScore", "GeoScoreWeight"}, Param);
	GeometryFilterScoreCalculator Filter(Param, InInitSearchSpace.size(), InCuttingFaceResults.StaticData);
	return SearchSpace();
}

// template<typename Scalar>
ForceTorque CalKnifeForce(CalcPCPTR_List InCuttingPlanePC, TrajectoryNode InTrajectoryNode)
{
	return ForceTorque::Zero(6, 1);
}

// template<typename Scalar>
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

int main()
{
	HoldingPointSet a;
	// SPDLog::LoggerMaker::GetProgramExecStartTime();
	auto OriginPC = LoadPC("./points.pcd");

	Trajectory KnifeTrajectory;
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