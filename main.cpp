#include <vector>
#include <tuple>

// #include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <GlobalTypes.hpp>
#include <Utilities/spdlog/LogConfig.hpp>


template<typename InputPointType>
auto PrepareData(PCL_Helper::PCPTR<InputPointType> InPC, Types::TrajectoryNode InTrajectoryNode)
{
	NEW_CALC_PC_PTR(CuttingPlanePC);
	NEW_CALC_PC_PTR(GraspingPartPC);

	Types::SearchSpace InitSearchSpace;
	InitSearchSpace.assign(1919, Types::HoldingPointSet::Zero(FINGER_NUMBER, 3));

	// Use [] to make cutting plane point cloud.


	return std::make_tuple(CuttingPlanePC, GraspingPartPC, InitSearchSpace);
}

// template<typename Scalar>
Types::SearchSpace FilterByGeoScore(Types::SearchSpace InInitSearchSpace, PCL_Helper::PCPTR<PCL_Helper::PointXYZRGBN> InPC, Types::TrajectoryNode InTrajectoryNode)
{
	return Types::SearchSpace();
}

// template<typename Scalar>
Types::ForceTorque CalKnifeForce(PCL_Helper::PCPTR<PCL_Helper::PointXYZRGBN> InCuttingPlanePC, Types::TrajectoryNode InTrajectoryNode)
{
	return Types::ForceTorque::Zero(6, 1);
}

// template<typename Scalar>
float CalForceScore(Types::ForceTorque InKnifeForce, Types::HoldingPointSet InHoldingPointSet)
{
	return 0.0f;
}

// 
template<typename PointType>
PCL_Helper::PCPTR<PointType> LoadPC(const std::string& InFilePath)
{
	return PCL_Helper::PCPTR<PointType>(new PCL_Helper::PC<PointType>);
}

int main()
{
	// SPDLog::LoggerMaker::GetProgramExecStartTime();
	auto OriginPC = LoadPC<PCL_Helper::PointXYZ>("");

	Types::Trajectory KnifeTrajectory;
	for ( int i = 0; i < KnifeTrajectory.size(); i++ )
	{

		const auto& KnifeTrajectoryNode = KnifeTrajectory[i];

		auto [CuttingPlanePC, GraspingPartPC, InitSearchSpace] = PrepareData(OriginPC, KnifeTrajectoryNode);
		auto MainSearchSpace = FilterByGeoScore(InitSearchSpace, GraspingPartPC, KnifeTrajectoryNode);

		auto KnifeForce = CalKnifeForce(CuttingPlanePC, KnifeTrajectoryNode);

		for ( int j = 0; j < MainSearchSpace.size(); j++ )
		{
			auto& HoldingPointSet = MainSearchSpace[j];  // SearchSpacetType  $\mathbf{P}$

			float ForceScore = CalForceScore(KnifeForce, HoldingPointSet);
			
		}

	}

	return 0;
}