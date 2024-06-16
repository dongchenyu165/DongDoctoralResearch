#ifndef A56A3744_1F22_4275_AE8B_D003DD3EAC16
#define A56A3744_1F22_4275_AE8B_D003DD3EAC16

#include <Eigen/Core>
#include <iostream>

namespace EigenHelper
{

template<typename Scalar>
using Vec3InType = const Eigen::Matrix<Scalar, 3, 1>&;

template<typename Scalar>
inline Scalar CalVectorAngle(Vec3InType<Scalar> InMatA, Vec3InType<Scalar> InMatB)
{
	return std::atan2(InMatA.cross(InMatB).norm(), InMatA.dot(InMatB));
}

template<typename Scalar>
inline Scalar PointToPlaneDistance(
	Vec3InType<Scalar> InPoint,
	Vec3InType<Scalar> InPlaneNormal,
	Vec3InType<Scalar> InPlanePoint
)
{
	// InOutPlaneNormal.normalize();
	return InPlaneNormal.normalized().dot(InPoint - InPlanePoint);
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> ProjectPointToPlane(
	Vec3InType<Scalar> InPoint,
	Vec3InType<Scalar> InPlaneNormal,
	Vec3InType<Scalar> InPlanePoint
)
{
	const Scalar Distance = PointToPlaneDistance(InPoint, InPlaneNormal, InPlanePoint);
	return InPoint - InPlaneNormal * Distance;
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1>
	MovePoint(Vec3InType<Scalar> InPoint, Vec3InType<Scalar> InMoveDir, const Scalar& InDistance)
{
	return InPoint - InMoveDir * InDistance;
}

template<typename Scalar, int VecDim, int VecNum>
inline Eigen::Matrix<Scalar, VecNum, 1>
	VectorListDotProduct(const Eigen::Matrix<Scalar, VecNum, VecDim>& InVectorList, const Eigen::Matrix<Scalar, VecDim, 1>& InDotVector)
{
	if constexpr ( VecNum == -1 )
	{
		Eigen::Matrix<Scalar, VecNum, 1> DotResult = Eigen::Matrix<Scalar, VecNum, 1>::Zero(InVectorList.rows(), 1);
		for ( int i = 0; i < InVectorList.cols(); ++i )
		{
			DotResult += InVectorList.col(i) * InDotVector(i);
		}
		return DotResult;
	}
	else
	{
		Eigen::Matrix<Scalar, VecNum, 1> DotResult = Eigen::Matrix<Scalar, VecNum, 1>::Zero();
		for ( int i = 0; i < VecDim; ++i )
		{
			DotResult += InVectorList.col(i) * InDotVector(i);
		}
		return DotResult;
	}
}

} // namespace EigenHelper

#endif /* A56A3744_1F22_4275_AE8B_D003DD3EAC16 */
