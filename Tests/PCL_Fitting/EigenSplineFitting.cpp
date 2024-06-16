
#include "Utilities/PCL_Helper/Basic/ToEigenMap.hpp"
#include "Utilities/PCL_Helper/Modules/DownSample.hpp"
#include "Utilities/PCL_Helper/Visualizer/PointArrangementViewer.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <string>

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>
#include <Utilities/PCL_Helper/Basic/PointCloudInfo.hpp>
#include <Utilities/PCL_Helper/Apps/PointsNearPlaneGetter.hpp>

#include <Utilities/JSON_Helper/StructSerializer.hpp>
#include <unsupported/Eigen/src/Splines/SplineFwd.h>

#include <boost/graph/kruskal_min_spanning_tree.hpp>

#include <functional>


PCL_Helper::PCPTR<PCL_Helper::PointXYZ> TestFunc(const Eigen::Matrix<double, 3, -1>& InPointsColMat)
{
	constexpr int POINT_NUM = 4000;
	using namespace PCL_Helper;
	NEW_PC_PTR(Result, PointXYZ);
	// Eigen::VectorXi v = Eigen::VectorXi::Random(4);
	// cout << "Here is the vector v:\n";
	// for(auto x : v) cout << x << " ";
	// cout << "\n";

	Eigen::Matrix<double, 3, -1> Mat = Eigen::Matrix<double, 3, -1>::Zero(3, POINT_NUM);
	Mat.row(2) = Eigen::VectorXd::LinSpaced(POINT_NUM, 2, 6).transpose();
	// std::cout << Mat.transpose() << std::endl;

	// const Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(InPointsColMat, InPointsColMat.cols() /200);
	const Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(InPointsColMat, 5);

	Eigen::VectorXd&& U_List = Eigen::VectorXd::LinSpaced(POINT_NUM, 0, 1).col(0);
	// EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION;
  
	PointXYZ pt;
	for (int i = 0; i < U_List.size(); i++)
	{
		pt.getVector3fMap() = spline(U_List(i)).cast<float>();
		Result->push_back(pt);
	}

	return Result;
}

// void CustomPointRemover(PCL_Helper::PCXYZ_Ptr InPC, PCL_Helper::PCXYZ_Ptr OutRemovedPC, bool bShouldBeRemoved(const PCL_Helper::PointXYZ&) )
void CustomPointRemover(PCL_Helper::PCXYZ_Ptr InPC, PCL_Helper::PCXYZ_Ptr OutRemovedPC, std::function<bool(const PCL_Helper::PointXYZ&)> bShouldBeRemoved)
{
	for (int i = 0; i < InPC->size(); i++)
	{
		auto& PointInPC = InPC->points[i];

		if (bShouldBeRemoved(PointInPC))
		{
			continue;
		}
		OutRemovedPC->push_back(PCL_Helper::PointXYZ(PointInPC));
	}
}

int main()
{
	using namespace PCL_Helper;
	using PointType			  = PointXYZ;
	PCPTR<PointType> SourcePC = PCPTR<PointType>(new PC<PointType>);
	pcl::io::loadPCDFile("./points.pcd", *SourcePC);

	auto InfoObj = TPointCloudInfo(SourcePC);
	InfoObj.UpdateInfo();

	// std::cout << "PointCloud GetAABBCenter: " << InfoObj.GetAABBCenter() << std::endl;
	// std::cout << "PointCloud GetPointsCenter: " << InfoObj.GetPointsCenter() << std::endl;
	// std::cout << "PointCloud AABB max: " << InfoObj.GetAABB().Max << std::endl;
	// std::cout << "PointCloud AABB min: " << InfoObj.GetAABB().Min << std::endl;

	PCL_Helper::App::PointsNearPlaneGrabberParam Param;
	JSON_Helper::LoadStructure_ByPath("./params.json", std::vector<std::string>(), Param);

	Param.PlanePosition = InfoObj.GetAABBCenter();
	PCL_Helper::App::TPointsNearPlaneGrabber<PointType> Grabber(Param);

	auto Result = Grabber.GrabPoints(SourcePC, PCL_Helper::App::ONLY_PC);

	auto ResultInfoObj = TPointCloudInfo(Result.PointCloud);
	ResultInfoObj.UpdateInfo();
	ResultInfoObj.GetAABBCenter()(0);

	// Eigen::Vector3f test_pt(0.6, -0.23);

	NEW_PC_PTR(Removed, PointXYZ);
	// CustomPointRemover(Result.PointCloud, Removed, [ResultInfoObj](const PCL_Helper::PointXYZ& InPt)
	CustomPointRemover(Result.PointCloud, Removed, [ResultInfoObj](const PCL_Helper::PointXYZ& InPt)
	{
		// return InPt.x < ResultInfoObj.GetAABBCenter()(0);
		return false;
		// return InPt.x < 0.5;
	});

	PCPTR<PointType> DownSampled = VoxelDownSampler(Removed, {0.005, 0.005, 0.005});
	pcl::visualization::CloudViewer Viewer2("DownSampled");
	// Viewer.showCloud(Result.PointCloud);
	Viewer2.showCloud(DownSampled);
	while ( !Viewer2.wasStopped() )	{ }

	PCPTR<PointType> FittedPoints = TestFunc(TPCL_EigenMapper<PointType>::GetPointEigenMatrixMapConst(DownSampled).cast<double>());

	std::cout << FittedPoints->size() << std::endl;
	std::cout << FittedPoints->getMatrixXfMap().transpose() << std::endl;
	NEW_PC_PTR(FittedRemoved, PointXYZ);
	const auto&& aabb = InfoObj.GetAABB();
	CustomPointRemover(FittedPoints, FittedRemoved, [aabb](const PCL_Helper::PointXYZ& InPt)
	{
		return ! aabb.IsPointInAABB(InPt.getVector3fMap());
	});


	// pcl::visualization::CloudViewer Viewer("Viewer");
	// // Viewer.showCloud(Result.PointCloud);
	// Viewer.showCloud(FittedRemoved);
	// while ( !Viewer.wasStopped() )	{ }
// PointXYZ pt;
// pt.getVector3fMap()
	TPointArrangementViewer<PointType> Viewer;
	Viewer.AddPointCloud(FittedRemoved);
	Viewer.spin();

	return 0;
}