#include <vector>
#include <tuple>

// #include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <GlobalTypes.hpp>
#include <Utilities/spdlog/LogConfig.hpp>


using namespace Types;

auto PrepareData(CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
{
	NEW_CALC_PC_PTR(CuttingPlanePC);
	NEW_CALC_PC_PTR(GraspingPartPC);

	SearchSpace InitSearchSpace;

	// Use [] to make cutting plane point cloud.


	return std::make_tuple(CuttingPlanePC, GraspingPartPC, InitSearchSpace);
}

// template<typename Scalar>
SearchSpace FilterByGeoScore(SearchSpace InInitSearchSpace, CalcPCPTR InPC, TrajectoryNode InTrajectoryNode)
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

		auto [CuttingPlanePC, GraspingPartPC, InitSearchSpace] = PrepareData(OriginPC, KnifeTrajectoryNode);
		SearchSpace MainSearchSpace = FilterByGeoScore(InitSearchSpace, CuttingFaceResultObj.GraspingPC, KnifeTrajectoryNode);

		auto KnifeForce = CalKnifeForce(CuttingPlanePC, KnifeTrajectoryNode);

		for ( int j = 0; j < MainSearchSpace.size(); j++ )
		{
			HoldingPointSet& HoldingPointSet = MainSearchSpace[j];  // SearchSpacetType  $\mathbf{P}$

			float ForceScore = CalForceScore(KnifeForce, HoldingPointSet);
			
		}

	}

	return 0;
}