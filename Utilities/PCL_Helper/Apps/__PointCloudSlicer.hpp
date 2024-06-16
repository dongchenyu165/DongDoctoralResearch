#ifndef B420D8DF_18AD_47E5_AD52_EDF36D419716
#define B420D8DF_18AD_47E5_AD52_EDF36D419716

#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>

namespace PCL_Helper
{
namespace App
{

struct SliceParam
{
	// The slice direction.
	Eigen::Vector3d SliceDirection = {0, 0, 1};
	// The 
	Eigen::Vector3d SliceStartPosition = {0, 0, 0};

	// The height of each slice layer. In [Meter]
	float LayerThickness = 0.02f;
	// If True, the points of each sliced layer will be project to each layer plane. If False, the slicer just segment points into each layer.
	bool bProjectToLayerPlane = false;
	// Use the oriented AABB as the range of slicing. If True, [SliceStartPosition] is DISABLED.
	bool bStartFromAABB = false;

	// Helper marco for easily serialize this structure from a json file or string.
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(SliceParam, SliceDirection, SliceStartPosition, LayerThickness, bProjectToLayerPlane,bStartFromAABB);
};

struct SliceLayerInfo
{
	Eigen::Matrix4f Tlocal2world;
	PCIDX_Ptr SlicedPointIndex;
};

template<typename PointType>
class PointCloudSlicer
{
public:
	/**
	 * @brief Construct a new Point Cloud Slicer object for slice only 1 layer
	 * 
	 * @param InDirection Slice layer direction
	 * @param InStartPosition Slice layer position
	 * @param InLayerThickness The layer thickness
	 */
	PointCloudSlicer(Eigen::Vector3d InDirection, Eigen::Vector3d InStartPosition, float InLayerThickness){}



private:
	int SlicedLayerCount = 0;
	Eigen::Matrix4f Tlocal2world;
	PCIDX_Ptr SlicedPointIndex;  // The point index of each sliced point cloud in the origin PC.
};

} // namespace App
} // namespace PCL_Helper

#endif /* B420D8DF_18AD_47E5_AD52_EDF36D419716 */
