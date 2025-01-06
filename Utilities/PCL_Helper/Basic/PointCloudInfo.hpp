#ifndef BCE53D3D_013E_4A06_A96E_C3095C3922F2
#define BCE53D3D_013E_4A06_A96E_C3095C3922F2

#include <Eigen/Core>
#include <Eigen/src/Core/util/Constants.h>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include "PCL_TypeAlias.hpp"
#include "FieldChecker.hpp"
#include "ToEigenMap.hpp"

namespace PCL_Helper
{

template<typename Scalar, int Dim = 3>
struct TAABB
{
	TAABB() { }

	TAABB(const Eigen::Matrix<Scalar, -1, Dim>& InPointList)
	{
		for ( int i = 0; i < Dim; i++ )
		{
			Min[i] = InPointList.col(i).minCoeff();
			Max[i] = InPointList.col(i).maxCoeff();
		}
	}

	void UpdateAABB(const Eigen::Matrix<Scalar, -1, Dim>& InPointList)
	{
		for ( int i = 0; i < Dim; i++ )
		{
			Min[i] = InPointList.col(i).minCoeff();
			Max[i] = InPointList.col(i).maxCoeff();
		}
	}

	bool IsPointInAABB(const Eigen::Matrix<Scalar, Dim, 1>& InPt) const
	{
		return (Min.array() < InPt.array()).all() & (Max.array() > InPt.array()).all();
	}

	void Reset()
	{
		Min.setZero();
		Max.setZero();
	}

	Eigen::Matrix<Scalar, Dim, 1> Min = Eigen::Matrix<Scalar, Dim, 1>::Zero();
	Eigen::Matrix<Scalar, Dim, 1> Max = Eigen::Matrix<Scalar, Dim, 1>::Zero();
};

// template <typename PointType>

template<typename PointType>
class TPointCloudInfo
{
	static constexpr int PointDim = HAS_z_FIELD<PointType>() ? 3 : 2;

  public:
	TPointCloudInfo(PCPTR<PointType> InPC) : OperatingPC(InPC) { UpdateInfo(); }

	void SetCalculationPC(PCPTR<PointType> InPC)
	{
		OperatingPC = InPC;
		UpdateInfo();
	}

	size_t GetSize() { return OperatingPC->size(); }
	TAABB<float, PointDim> GetAABB() const { return AABB; }
	Eigen::Matrix<float, PointDim, 1> GetPointsCenter() const { return PointsCenter; }
	Eigen::Matrix<float, PointDim, 1> GetAABBCenter() const { return AABBCenter; }

	void UpdateInfo()
	{
		if ( OperatingPC->size() == 0 )
		{
			AABB.Reset();
			return;
		}
		auto PointsMatrix = TPCL_EigenMapper<PointType>::GetPointEigenMatrixMapConst(OperatingPC).transpose();
		AABB.UpdateAABB(PointsMatrix);
		PointsCenter = PointsMatrix.colwise().mean();
		AABBCenter	 = AABB.Min + (AABB.Max - AABB.Min) / 2.0;
	}

	std::string ToString()
	{
		std::stringstream StrStream;

		StrStream 	<< "Info of point cloud [0x" << std::hex << ((uint64_t)(OperatingPC.get())) << std::dec << "] with size: " << OperatingPC->size() << std::endl
					<< "\tPoint center: " << PointsCenter.transpose() << std::endl
					<< "\tAABB center: " << AABBCenter.transpose() << std::endl
					<< "\tAABB MAX: " << AABB.Max.transpose() << std::endl
					<< "\tAABB MIN: " << AABB.Min.transpose() << std::endl;
		return StrStream.str();
	}


private:
	PCPTR<PointType> OperatingPC;

	TAABB<float, PointDim> AABB;
	Eigen::Matrix<float, PointDim, 1> PointsCenter;
	Eigen::Matrix<float, PointDim, 1> AABBCenter;
};
} // namespace PCL_Helper

#endif /* BCE53D3D_013E_4A06_A96E_C3095C3922F2 */
