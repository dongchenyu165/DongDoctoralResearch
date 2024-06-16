#ifndef B80FE47B_1E8B_411C_AB9E_7D0303921473
#define B80FE47B_1E8B_411C_AB9E_7D0303921473

// #include "GlobalTypes.hpp"
#include "GlobalVars.hpp"
#include "Utilities/PCL_Helper/Visualizer/DebugViewerManager.hpp"
#include "Utilities/spdlog/LogConfig.hpp"
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/io.h>

#include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>
#include <Utilities/PCL_Helper/Basic/FieldChecker.hpp>
#include <Utilities/PCL_Helper/Basic/PointCloudInfo.hpp>
#include <pcl/type_traits.h>

namespace PCL_Helper
{

struct EstimateNormalParams
{
	PCIDX_Ptr IndexMask       = nullptr;
	float RadiusSearch        = 0.01f;
	bool bNormalTowardsCenter = false;
};

/**
 * @brief Estimate normal data for inputing point cloud
 *
 * @tparam PointType
 * @param InOutOperatingPC Point cloud data for estimating normal, if [PointType] has normal field, the estimated normal
 * will storage to that field directly.
 * @param InEstimateParams Some parameters for estimating normal.
 * @param OutNormal Output the normal data. Only avaliable when the input cloud's PointType has NO normal field.
 */
template<typename PointType>
void EstimateNormal(PCPTR<PointType> InOutOperatingPC,
	const EstimateNormalParams& InEstimateParams = EstimateNormalParams(),
	PCN_Ptr OutNormal                            = PCN_Ptr(new PCN))
{
	auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_GlobalLogger", LogConfigJsonPath);
	LOG_FUNC_ENTER(Logger, debug, 2);

	constexpr bool bHAS_NORMAL = PCL_Helper::HAS_NORMAL_FIELD<PointType>();

	TPointCloudInfo<PointType> InputPC_Info(InOutOperatingPC);
	const auto& AABBCenter = InputPC_Info.GetAABBCenter();
	SPDLog::Log_D(Logger, 2, "Input point type {} [Normal] field.", bHAS_NORMAL ? "HAS" : "has NO");
	SPDLog::Log_T(Logger, 3, "PointCloud's info: \n{}", InputPC_Info.ToString());

	typename pcl::search::KdTree<PointType>::Ptr KDTreeObj(new pcl::search::KdTree<PointType>());

	using NormalOutType = std::conditional<bHAS_NORMAL, PointType, pcl::PointNormal>;
	pcl::NormalEstimationOMP<PointType, typename NormalOutType::type> NormalEstimator;

	NormalEstimator.setRadiusSearch(InEstimateParams.RadiusSearch);
	NormalEstimator.setSearchMethod(KDTreeObj);
	NormalEstimator.setInputCloud(InOutOperatingPC);
	NormalEstimator.setViewPoint(AABBCenter.x(), AABBCenter.y(), AABBCenter.z());
	if ( InEstimateParams.IndexMask && InEstimateParams.IndexMask->indices.size() > 0 )
	{
		NormalEstimator.setIndices(InEstimateParams.IndexMask);
	}

	auto FlipNormal = [InEstimateParams, InOutOperatingPC]()
	{
		for ( int i = 0; i < InOutOperatingPC->size(); i++ )
		{
			if ( InEstimateParams.bNormalTowardsCenter )
			{
				(*InOutOperatingPC)[i].getNormalVector3fMap() = -(*InOutOperatingPC)[i].getNormalVector3fMap();
			}
		}
	};

	if constexpr ( bHAS_NORMAL )
	{
		NormalEstimator.compute(*InOutOperatingPC);
		FlipNormal();
		SPDLog::Log_D(Logger, 2, "Result normal: [{}]", InOutOperatingPC->points[0].getNormalVector3fMap());

		BEGIN_DEBUG_SHOW("EstimateNormal", "PCL_Helper", PointType);
		ViewerPtr->AddPointCloudList({ InOutOperatingPC }, "Normal PC");
		ViewerPtr->AddNormal(InOutOperatingPC, 1.0f);
		END_DEBUG_SHOW();
	}
	else
	{
		NormalEstimator.compute(*OutNormal);
		FlipNormal();

		BEGIN_DEBUG_SHOW("EstimateNormal", "PCL_Helper", PointType);
		ViewerPtr->AddPointCloudList({ InOutOperatingPC }, "Normal PC");
		ViewerPtr->AddNormal(InOutOperatingPC, OutNormal, 1.0f);
		END_DEBUG_SHOW();
	}

	LOG_FUNC_EXIT(Logger, debug, 2);
}
} // namespace PCL_Helper

#endif /* B80FE47B_1E8B_411C_AB9E_7D0303921473 */
