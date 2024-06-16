#ifndef B208C09C_977A_4988_87E2_92442C524240
#define B208C09C_977A_4988_87E2_92442C524240

#include "pcl/common/transforms.h"
#include "pcl/filters/passthrough.h"

#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"


/**
 * @brief 保留平面以上的点云
 *
 * @param InPC
 * @param InCuttingPlanePose
 * @param bInRestIsUpper
 * @param InFieldName
 * @return PCL_Helper::PCIDX_Ptr
 */
template<typename PointType>
PCL_Helper::PCIDX_Ptr GetPointIdxAbovePlane(
	PCL_Helper::PCPTR<PointType> InPC,
	const Eigen::Matrix4f& InPlanePose,
	const bool& bInGetNegative,
	const std::string& InNormalFieldName = "y"
)
{
	PCL_Helper::PCIDX_Ptr PointIdx = PCL_Helper::PCIDX_Ptr(new PCL_Helper::PCIDX);
	PCL_Helper::PCPTR<PointType> TransformedPC = PCL_Helper::PCPTR<PointType>(new PCL_Helper::PC<PointType>);

	pcl::transformPointCloud(*InPC, *TransformedPC, InPlanePose.inverse());

	pcl::PassThrough<PCL_Helper::PointXYZ> PT;
	PT.setFilterFieldName(InNormalFieldName);
	PT.setFilterLimits(0, std::numeric_limits<float>::max());
	PT.setNegative(bInGetNegative);
	PT.setInputCloud(TransformedPC);
	PT.filter(PointIdx->indices);

	PCL_Helper::PCPTR<PointType> RestPC = PCL_Helper::PCPTR<PointType>(new PCL_Helper::PC<PointType>);
	pcl::copyPointCloud(*InPC, PointIdx->indices, *RestPC);

	return PointIdx;
}

#endif /* B208C09C_977A_4988_87E2_92442C524240 */
