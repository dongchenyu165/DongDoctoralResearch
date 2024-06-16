#include <string>

#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>
#include <Utilities/PCL_Helper/Basic/PointCloudInfo.hpp>
// #include "Utilities/PCL_Helper/Basic/ToEigenMap.hpp"
#include <Utilities/PCL_Helper/Apps/PointsNearPlaneGetter.hpp>
#include <Utilities/PCL_Helper/Apps/PointsArranger.hpp>
#include "Utilities/PCL_Helper/Modules/DownSample.hpp"
#include "Utilities/PCL_Helper/Visualizer/PointArrangementViewer.hpp"
// 
#include <Utilities/JSON_Helper/StructSerializer.hpp>

#include <functional>

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

#if !defined(BUILD_TYPE)
    #define BUILD_TYPE  "<None>"
#endif

int main()
{
	using namespace PCL_Helper;
	using PointType			  = PointXYZ;
	PCPTR<PointType> SourcePC = PCPTR<PointType>(new PC<PointType>);
	pcl::io::loadPCDFile("./points.pcd", *SourcePC);

	auto InfoObj = TPointCloudInfo(SourcePC);
	InfoObj.UpdateInfo();


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

	// PCPTR<PointType> DownSampled = VoxelDownSampler(Removed, {0.007, 0.007, 0.007});
	PCPTR<PointType> DownSampled = VoxelDownSampler(Removed, {0.005, 0.005, 0.005});
	// pcl::visualization::CloudViewer Viewer2("DownSampled");
	// Viewer.showCloud(Result.PointCloud);
	// Viewer2.showCloud(DownSampled);
	// while ( !Viewer2.wasStopped() )	{ }




	App::TPointsArranger<PointType> Arranger;
	Arranger.ArrangePoints(DownSampled);
	// TPointArrangementViewer<PointType> Viewer;
	// Viewer.AddPointCloud(FittedRemoved);
	// Viewer.spin();

	return 0;
}