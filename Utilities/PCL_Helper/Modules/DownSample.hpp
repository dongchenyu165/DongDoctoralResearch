#ifndef E45897A7_4D9C_43A3_B2E0_866E2DFFDBE1
#define E45897A7_4D9C_43A3_B2E0_866E2DFFDBE1

#include <cstddef>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "../Basic/PCL_TypeAlias.hpp"

namespace PCL_Helper {


template<typename PointType>
PCPTR<PointType> VoxelDownSampler(PCPTR<PointType> InSourcePC, Eigen::Vector3f InVoxelSize = {0.002, 0.002, 0.002})
{
	PCPTR<PointType> ResultPC(new PC<PointType>);
	pcl::VoxelGrid<PointType> VoxelGridFilter;
	VoxelGridFilter.setLeafSize(InVoxelSize(0), InVoxelSize(1), InVoxelSize(2)); //Set the size of VoxelGrid.
	VoxelGridFilter.setInputCloud(InSourcePC);
	VoxelGridFilter.filter(*ResultPC);

	return ResultPC;
}

template<typename PointType>
PCPTR<PointType> StatisticRemoval(PCPTR<PointType> InSourcePC, size_t InNumNeighbors = 25, float InStdVar = 0.6)
{
	PCXYZ_Ptr Output(new PCXYZ);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> StatisticalRemovalFilter;
	StatisticalRemovalFilter.setMeanK(InNumNeighbors);
	StatisticalRemovalFilter.setStddevMulThresh(InStdVar);
	StatisticalRemovalFilter.setInputCloud(InSourcePC);
	StatisticalRemovalFilter.filter(*Output);

	return Output;
}

}

#endif /* E45897A7_4D9C_43A3_B2E0_866E2DFFDBE1 */
