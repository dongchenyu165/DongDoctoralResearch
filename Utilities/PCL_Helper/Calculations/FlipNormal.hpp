#ifndef BDAA8E73_FDA6_496D_8AC9_E84919C6B6EB
#define BDAA8E73_FDA6_496D_8AC9_E84919C6B6EB

#include <pcl/features/normal_3d.h>

#include "../Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Basic/FieldChecker.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudInfo.hpp"

namespace PCL_Helper
{


template<typename PointT>
inline void FlipNormal_ByViewpoint (const PCPTR<PointT>& InPC, const Eigen::Vector3f& InViewPoint = Eigen::Vector3f::Zero(), bool bInTowardsViewpoint = false)
{
	constexpr bool bInputHas_Normal = HAS_NORMAL_FIELD<PointT>();
	if constexpr (!bInputHas_Normal)
	{
		return;
	}

	for ( int i = 0; i < InPC->size (); i++ )
	{
		pcl::flipNormalTowardsViewpoint (
			(*InPC)[i], InViewPoint.x(), InViewPoint.y(), InViewPoint.z(), (*InPC)[i].normal[0], (*InPC)[i].normal[1],
			(*InPC)[i].normal[2]
		);
		if (!bInTowardsViewpoint)
		{
			(*InPC)[i].getNormalVector3fMap() = -(*InPC)[i].getNormalVector3fMap();
		}
	}
}
template<typename PointT, typename PointNT>
inline void FlipNormal_ByViewpoint (const std::shared_ptr<pcl::PointCloud<PointT>>& InPC, const std::shared_ptr<pcl::PointCloud<PointNT>>& InNormalPC, const Eigen::Vector3f& InViewPoint = Eigen::Vector3f::Zero(), bool bInTowardsViewpoint = false)
{

	constexpr bool bInputHas_Normal = HAS_NORMAL_FIELD<PointNT>();
	if constexpr (!bInputHas_Normal)
	{
		return;
	}

	for ( int i = 0; i < InPC->size (); i++ )
	{
		pcl::flipNormalTowardsViewpoint (
			(*InPC)[i], InViewPoint.x(), InViewPoint.y(), InViewPoint.z(), (*InNormalPC)[i].normal[0], (*InNormalPC)[i].normal[1],
			(*InNormalPC)[i].normal[2]
		);
		if (!bInTowardsViewpoint)
		{
			(*InNormalPC)[i].getNormalVector3fMap() = -(*InNormalPC)[i].getNormalVector3fMap();
		}
	}
}
template<typename PointT>
inline void FlipNormal_ByAABBCenter (const std::shared_ptr<pcl::PointCloud<PointT>>& InPC, bool bInTowardsCenter = false)
{
	TPointCloudInfo<PointT> InfoObj(InPC);
	FlipNormal_ByViewpoint(InPC, InfoObj.GetAABBCenter(), bInTowardsCenter);
}
template<typename PointT, typename PointNT>
inline void FlipNormal_ByAABBCenter (const std::shared_ptr<pcl::PointCloud<PointT>>& InPC, const std::shared_ptr<pcl::PointCloud<PointNT>>& InNormalPC, bool bInTowardsCenter = false)
{
	TPointCloudInfo<PointT> InfoObj(InPC);
	FlipNormal_ByViewpoint(InPC, InNormalPC, InfoObj.GetAABBCenter(), bInTowardsCenter);
}

};

#endif /* BDAA8E73_FDA6_496D_8AC9_E84919C6B6EB */
