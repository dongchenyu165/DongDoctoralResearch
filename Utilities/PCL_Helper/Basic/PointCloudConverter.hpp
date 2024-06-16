#ifndef CA3699CE_8642_4F82_808F_97D59D59A5F6
#define CA3699CE_8642_4F82_808F_97D59D59A5F6

#include <pcl/features/normal_3d.h>

#include "PCL_TypeAlias.hpp"
#include "FieldChecker.hpp"

namespace PCL_Helper
{

enum class EConvertRGBField
{
	Copy,
	Const
};
enum class EConvertNormalField
{
	Copy,
	Const,
	Estimate
};

/**
 * @brief Convert a point cloud from point type [SourcePointType] to another point type [TargetPointType].
 * 
 * @tparam TargetPointType The target point type of point cloud.
 * @tparam RGBStrategy The behavior of converting point cloud when taget point type has rgb field; [Const]: Use inputed [InRGB] parameter as rgb field.; [Copy]: Copy from rgb field of the source point cloud.
 * @tparam NormalStrategy [Const]: Use inputed [InNormal] parameter as normal field.; [Copy]: Copy from normal field of the source point cloud.; [Estimate]: Use [] to estimate the normal field.
 * @tparam SourcePointType The source point type of point cloud. Often deduced automatically by the compiler.
 * @param InSourcePC The input point cloud smart pointer object of the source.
 * @param InRGB The integer RGB value in range [0 - 255]. Only used when [RGBStrategy] set to [Const]
 * @param InNormal The 3D normal vector. Only used when [NormalStrategy] set to [Const].
 * @param InSearchRadius The search radius used in normal estimation. Larger means more smoother normals.
 * @return PCPTR<TargetPointType> The converted point cloud smart pointer object.
 */
template<
	class TargetPointType,
	EConvertRGBField RGBStrategy	   = EConvertRGBField::Const,
	EConvertNormalField NormalStrategy = EConvertNormalField::Const,
	class SourcePointType			   = pcl::PointXYZRGBNormal>
PCPTR<TargetPointType> ConvertPointCloud(
	PCPTR<SourcePointType> InSourcePC,
	Eigen::Vector3i InRGB	 = Eigen::Vector3i(255, 255, 255),
	Eigen::Vector3f InNormal = Eigen::Vector3f(0, 0, 0),
	float InSearchRadius	 = 0.01
)
{
	using TargetPtrType					= PCPTR<TargetPointType>;
	using SourcePtrType					= PCPTR<SourcePointType>;

	constexpr bool bTargetHas_z			= HAS_z_FIELD<TargetPointType>();
	constexpr bool bTargetHas_RGB		= HAS_RGB_FIELD<TargetPointType>();
	constexpr bool bTargetHas_Normal	= HAS_NORMAL_FIELD<TargetPointType>();

	constexpr bool bSourceHas_z			= HAS_z_FIELD<TargetPointType>();
	constexpr bool bSourceHas_RGB		= HAS_RGB_FIELD<TargetPointType>();
	constexpr bool bSourceHas_Normal	= HAS_NORMAL_FIELD<TargetPointType>();

	constexpr bool bCopyRGBField		= (bTargetHas_RGB && bSourceHas_RGB) && RGBStrategy == EConvertRGBField::Copy;
	constexpr bool bSetRGBFieldConst	= (bTargetHas_RGB) && RGBStrategy == EConvertRGBField::Const;

	constexpr bool bCopyNormalField		= (bTargetHas_Normal && bSourceHas_Normal) && NormalStrategy == EConvertNormalField::Copy;
	constexpr bool bSetNormalFieldConst = (bTargetHas_Normal) && NormalStrategy == EConvertNormalField::Const;
	constexpr bool bEstimateNormalField = (bTargetHas_Normal) && NormalStrategy == EConvertNormalField::Estimate;

	TargetPtrType ConvertedPC			= TargetPtrType(new pcl::PointCloud<TargetPointType>);
	TargetPointType TempPoint;

	for ( SourcePointType point : InSourcePC->points )
	{
		TempPoint.x = point.x;
		TempPoint.y = point.y;
		if constexpr ( bTargetHas_z && bSourceHas_z )
		{
			TempPoint.z = point.z;
		}

		if constexpr ( bCopyRGBField )
		{
			TempPoint.r = point.r;
			TempPoint.g = point.g;
			TempPoint.b = point.b;
		}
		if constexpr ( bSetRGBFieldConst )
		{
			TempPoint.r = InRGB.x();
			TempPoint.g = InRGB.y();
			TempPoint.b = InRGB.z();
		}

		if constexpr ( bCopyNormalField )
		{
			TempPoint.normal_x = point.normal_x;
			TempPoint.normal_y = point.normal_y;
			TempPoint.normal_z = point.normal_z;
		}
		if constexpr ( bSetNormalFieldConst )
		{
			TempPoint.normal_x = InNormal.x();
			TempPoint.normal_y = InNormal.y();
			TempPoint.normal_z = InNormal.z();
		}

		ConvertedPC->push_back(TempPoint);
	}

	if constexpr ( bEstimateNormalField )
	{
		typename pcl::search::KdTree<TargetPointType>::Ptr tree(new pcl::search::KdTree<TargetPointType>());
		pcl::NormalEstimation<TargetPointType, pcl::Normal> ne;
		ne.setInputCloud(InSourcePC);
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(InSearchRadius);
		ne.compute(*ConvertedPC);
	}

	return ConvertedPC;
}

} // namespace PCL_Helper

#endif /* CA3699CE_8642_4F82_808F_97D59D59A5F6 */
