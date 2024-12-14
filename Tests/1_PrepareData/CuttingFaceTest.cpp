#include "GlobalBaseTypes.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudInfo.hpp"
#include "Utilities/PCL_Helper/Modules/EstimateNormal.hpp"
#include "Utilities/PCL_Helper/Modules/ReconstructSurface.hpp"
#include <1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp>
#include <chrono>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <Utilities/spdlog/LogConfig.hpp>
#include <sstream>
#include <thread>
#include <spdlog/spdlog.h>



TKnifeTrajectoryNode<Types::CalcScalar> MakeTrajectory(const PCL_Helper::TPointCloudInfo<Types::CalcPoint>& InPC_InfoObj)
{
	TKnifeTrajectoryNode<Types::CalcScalar> Traj;
	Traj.Pose = decltype(Traj.Pose)::Identity();
	Traj.Pose.block<3, 1>(0, 0) = Eigen::Matrix<Types::CalcScalar, 3, 1>(-1, 0, 0);
	Traj.Pose.block<3, 1>(0, 1) = Eigen::Matrix<Types::CalcScalar, 3, 1>(0, 0, 1);
	Traj.Pose.block<3, 1>(0, 2) = Eigen::Matrix<Types::CalcScalar, 3, 1>(0, 1, 0);

	Traj.Pose.block<3, 1>(0, 3) = InPC_InfoObj.GetPointsCenter().cast<Types::CalcScalar>();
	return Traj;
}


int main()
{
	// Prepare logger [Global].
	SPDLog::LoggerType Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("Global", LogConfigJsonPath);

	// Load point cloud.
	NEW_CALC_PC_PTR(SourcePC);
	pcl::io::loadPCDFile("./points.pcd", *SourcePC);

	PCL_Helper::ReconstructSurface(SourcePC, 0.01);
	PCL_Helper::EstimateNormal(SourcePC, PCL_Helper::EstimateNormalParams());

	PCL_Helper::TPointCloudInfo<Types::CalcPoint> PC_InfoObj(SourcePC);
	Logger->info("Loaded point cloud info: \n{}", PC_InfoObj.ToString());

	// Make a testing trajectory
	auto Traj = MakeTrajectory(PC_InfoObj);
	Logger->info("Trajectory pose matrix: \n{}", Traj.Pose);

	CuttingFaceMaker Maker("/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json", Traj);
	CuttingFaceResult CuttingFaceResultObj = Maker.MakeCuttingFace(SourcePC);

	SPDLog::Log_D(Logger, 2, "GraspingPC normal: [{}]", CuttingFaceResultObj.GraspingPC->points[0].getNormalVector3fMap());
	pcl::io::savePCDFileASCII("./GraspingPC.pcd", *CuttingFaceResultObj.GraspingPC);

	return 0;
}