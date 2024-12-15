#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "2_FilterByGeo/GeometryFilterScoreCalculator.hpp"
#include "DataTypes/PointSetData.hpp"
#include "GlobalBaseTypes.hpp"
#include "SearchSpaceGenerator/SearchSpaceGenerator.hpp"
#include <vector>
#include <tuple>

// #include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <GlobalTypes.hpp>
#include <Utilities/spdlog/LogConfig.hpp>


using namespace Types;

auto PrepareData(CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
{
	SearchSpace InitSearchSpace;
	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(InPC);
	size_t RealCombCount = SearchSpaceGenObj.Generate(InitSearchSpace);

	CuttingFaceMaker Maker("/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json", InTrajectoryNode);
	CuttingFaceResult CuttingFaceResultObj = Maker.MakeCuttingFace(InPC);

	return std::make_tuple(CuttingFaceResultObj, InitSearchSpace);
}

// template<typename Scalar>
SearchSpace FilterByGeoScore(SearchSpace InInitSearchSpace, CuttingFaceResult& InCuttingFaceResults, TrajectoryNode InTrajectoryNode)
{

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
	return CalcPCPTR(new CalcPC);
}

int main()
{
	HoldingPointSet a;
	// SPDLog::LoggerMaker::GetProgramExecStartTime();
	auto OriginPC = LoadPC<PCL_Helper::PointXYZ>("");

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