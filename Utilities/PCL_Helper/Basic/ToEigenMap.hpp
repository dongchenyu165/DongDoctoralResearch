#ifndef BBEF7D0E_66AB_4CD9_9C0F_46D14AB4383E
#define BBEF7D0E_66AB_4CD9_9C0F_46D14AB4383E

#include "FieldChecker.hpp"
#include "PCL_TypeAlias.hpp"

namespace PCL_Helper
{

template<typename PointType>
class TPCL_EigenMapper
{

	// Point field map
	static constexpr int POINT_DIM	  = HAS_z_FIELD<PointType>() ? 3 : 2;

	using PointMapMatrixType		  = const Eigen::Matrix<float, POINT_DIM, Eigen::Dynamic>;
	static constexpr int POINT_STRIDE = sizeof(PointType) / sizeof(float);
	using PointStrideType			  = Eigen::Stride<POINT_STRIDE, 1>;

public:
	/**
	 * @brief Get (3, N) or (2, N) CONST Eigen Matrix Map object for a point cloud smart pointer;
	 * pcl::PointCloud::getMatrixXfMap() can get a row&column all dynamic Eigen Matrix Map object
	 *
	 * @param InOperatingPC
	 * @return const Eigen::Map<MapMatrixType, Eigen::Aligned, StrideType>
	 */
	static const Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType> GetPointEigenMatrixMapConst(PCPTR<PointType> InOperatingPC)
	{
		if ( PointMapMatrixType::Flags & Eigen::RowMajorBit )
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<PointType*>(&InOperatingPC->points[0])),
				InOperatingPC->size(),
				POINT_DIM
			));
		}
		else
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<PointType*>(&InOperatingPC->points[0])),
				POINT_DIM,
				InOperatingPC->size()
			));
		}
	}
	/**
	 * @brief Get (3, N) or (2, N) Eigen Matrix Map object for a point cloud smart pointer;
	 * pcl::PointCloud::getMatrixXfMap() can get a row&column all dynamic Eigen Matrix Map object
	 *
	 * @param InOperatingPC
	 * @return const Eigen::Map<MapMatrixType, Eigen::Aligned, StrideType>
	 */
	static Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType> GetPointEigenMatrixMap(PCPTR<PointType> InOperatingPC)
	{
		if ( PointMapMatrixType::Flags & Eigen::RowMajorBit )
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<PointType*>(&InOperatingPC->points[0])),
				InOperatingPC->size(),
				POINT_DIM
			));
		}
		else
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<PointType*>(&InOperatingPC->points[0])),
				POINT_DIM,
				InOperatingPC->size()
			));
		}
	}

	/**
	* @brief Get (3, N) Eigen Matrix Map object for point cloud normal data
	*/
	static Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType> GetNormalEigenMatrixMap(PCPTR<PointType> InOperatingPC)
	{
		static_assert(pcl::traits::has_normal<PointType>::value, "Point type must have normal field");
		
		if (PointMapMatrixType::Flags & Eigen::RowMajorBit)
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<float*>(&InOperatingPC->points[0].normal_x)),
				InOperatingPC->size(),
				3  // Normal always has 3 dimensions
			));
		}
		else 
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<float*>(&InOperatingPC->points[0].normal_x)),
				3,
				InOperatingPC->size()
			));
		}
	}

	/**
	* @brief Get (3, N) or (4, N) Eigen Matrix Map object for point cloud color data
	*/
	static Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType> GetColorEigenMatrixMap(PCPTR<PointType> InOperatingPC)
	{
		static_assert(pcl::traits::has_color<PointType>::value, "Point type must have color field");
		
		constexpr int COLOR_DIM = pcl::traits::has_field<PointType, pcl::fields::a>::value ? 4 : 3;
		
		if (PointMapMatrixType::Flags & Eigen::RowMajorBit)
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<uint8_t*>(&InOperatingPC->points[0].r)),
				InOperatingPC->size(),
				COLOR_DIM
			));
		}
		else
		{
			return (Eigen::Map<PointMapMatrixType, Eigen::Aligned, PointStrideType>(
				reinterpret_cast<float*>(const_cast<uint8_t*>(&InOperatingPC->points[0].r)),
				COLOR_DIM,
				InOperatingPC->size()
			));
		}
	}
};
} // namespace PCL_Helper

#endif /* BBEF7D0E_66AB_4CD9_9C0F_46D14AB4383E */
