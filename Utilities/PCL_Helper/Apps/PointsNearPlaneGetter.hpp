#ifndef FE0CB062_450C_4F2C_9100_FD3608F81976
#define FE0CB062_450C_4F2C_9100_FD3608F81976

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>

#include <pcl/common/transforms.h>

#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>
#include <Utilities/EigenHelper/MakeRotationMat.hpp>
#include <Utilities/EigenHelper/Calculations.hpp>

namespace PCL_Helper
{
namespace App
{

enum EPlaneSegmentType
{
	AbovePlane = 0x01,
	BelowPlane = 0x02,
	NearPlane = 0x04,
};

struct PointsNearPlaneGrabberParam
{
	Eigen::Vector3f PlaneNormal	  = { 0, 0, 1 };
	Eigen::Vector3f PlanePosition = { 0, 0, 0 };
	// The thickness(range) of getting points. In [Meter]. [0.02] means get points from range [-0.01 ~ +0.01].
	float PlaneThickness = 0.02f;

	// A reference direction of the secondary axis when generate [Tlocal2world].
	Eigen::Vector3f Advance_PlanePoseSecondaryAxisRefDir = { 1, 0, 0 };

	// If True, the got points will be projected to the plane. If False, just get these points.
	bool bProjectToPlane = false;

	// 
	EPlaneSegmentType PlaneSegmentType = EPlaneSegmentType::NearPlane;

	// Helper marco for easily serialize this structure from a json file or string.
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PointsNearPlaneGrabberParam, PlaneNormal, PlanePosition, PlaneThickness, Advance_PlanePoseSecondaryAxisRefDir, bProjectToPlane);
};

template<typename PointType>
struct TPointsNearPlaneGrabberResultInfo
{
	PCPTR<PointType> PointCloud = PCPTR<PointType>(new PC<PointType>);
	PCIDX_Ptr PointIndex		= PCIDX_Ptr(new PCIDX);

	/**
	 * @brief A transform matrix for transforming result points from [local] space to [world] space.
	 Designed for easily operating grabbed points in a 2D space (Like: Calculate the cross-section area of a object).
	 * [local] space means: The grabbing plane on the XoY plane and the origin point is [PlanePosition].
	 *
	 * auto TransformedOrgPC = PCXYZ_Ptr(new PCXYZ);
	 * pcl::transformPointCloud(*InOrgPC, *TransformedOrgPC, ResultObj.Tlocal2world.inverse());
	 */
	Eigen::Matrix4f Tlocal2world;
};

enum EPointsNearPlaneGrabberResult
{
	ONLY_PC	   = 0x01, // Return a point cloud smart pointer object contains the result.
	ONLY_INDEX = 0x02, // Return a point index smart pointer object contains the index of the result points.
	BOTH	   = 0x03  // Return both of above.
};

template<typename PointType>
class TPointsNearPlaneGrabber
{
	using ResultType = TPointsNearPlaneGrabberResultInfo<PointType>;

public:
	TPointsNearPlaneGrabber(const PointsNearPlaneGrabberParam& InParam) : GrabberParam(InParam) { }

	ResultType GrabPoints(PCL_Helper::PCPTR<PointType> InSourcePC, EPointsNearPlaneGrabberResult InResultType = ONLY_PC)
	{
		ResultType ResultObj;

		GrabberParam.PlaneNormal.normalize();
		GrabberParam.Advance_PlanePoseSecondaryAxisRefDir.normalize();
		const bool bGrabAbove = (GrabberParam.PlaneSegmentType & AbovePlane) == AbovePlane;
		const bool bGrabBelow = (GrabberParam.PlaneSegmentType & BelowPlane) == BelowPlane;
		const bool bGrabNear = (GrabberParam.PlaneSegmentType & NearPlane) == NearPlane;
		const float DistanceThreshold = GrabberParam.PlaneThickness / 2;

		PointType TempPoint;

		// Loop each point in the source point cloud.
		for ( int i = 0; i < InSourcePC->size(); i++ )
		{
			const Eigen::Matrix<float, 3, 1>& PointRef = (*InSourcePC)[i].getVector3fMap();

			// Calculate & Check point-plane distance.
			float PointPlaneDistance = EigenHelper::PointToPlaneDistance(PointRef, GrabberParam.PlaneNormal, GrabberParam.PlanePosition);
			if ( bGrabNear && abs(PointPlaneDistance) > DistanceThreshold ||
				bGrabAbove && PointPlaneDistance < DistanceThreshold ||
				bGrabBelow && PointPlaneDistance > -DistanceThreshold )
			{
				continue;
			}

			// Insert the point index if needed.
			if ( InResultType == (InResultType & ONLY_INDEX) )
			{
				ResultObj.PointIndex->indices.push_back(i);
			}

			// Insert the point if needed.
			if ( InResultType == (InResultType & ONLY_PC) )
			{
				// Insert projected point to the result point cloud.
				if ( GrabberParam.bProjectToPlane )
				{
					TempPoint.getVector3fMap() = EigenHelper::MovePoint(PointRef, GrabberParam.PlaneNormal, PointPlaneDistance);
					// const Eigen::Matrix<float, 3, 1>& PointOnPlane = EigenHelper::MovePoint(PointRef, GrabberParam.PlaneNormal, PointPlaneDistance);
					ResultObj.PointCloud->push_back(TempPoint);
					continue;
				}

				// Insert point to the result point cloud.
				ResultObj.PointCloud->push_back((*InSourcePC)[i]);
			}
		}

		// Finally, create transform matrix.
		Eigen::Matrix3f RotationMat
			= EigenHelper::MakeRotationMat(GrabberParam.PlaneNormal, 'z', GrabberParam.Advance_PlanePoseSecondaryAxisRefDir, 'x');
		ResultObj.Tlocal2world							  = Eigen::Matrix4f::Identity();
		ResultObj.Tlocal2world.template block<3, 3>(0, 0) = RotationMat;
		ResultObj.Tlocal2world.template block<3, 1>(0, 3) = GrabberParam.PlanePosition;

		return ResultObj;
	}

	PointsNearPlaneGrabberParam GrabberParam;
};

} // namespace App
} // namespace PCL_Helper

#endif /* FE0CB062_450C_4F2C_9100_FD3608F81976 */
