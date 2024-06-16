#ifndef E4B48B54_72CB_4680_A21B_F8AB86CDF600
#define E4B48B54_72CB_4680_A21B_F8AB86CDF600

#include <Eigen/Core>
#include <cstdint>
#include <pcl/common/transforms.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// #include "Eigen/src/Geometry/Transform.h"
#include "GlobalTypes.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/spdlog/LogConfig.hpp"
#include <Utilities/EigenHelper/MakeRotationMat.hpp>
#include <Utilities/EigenHelper/Calculations.hpp>
#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>

namespace PCL_Helper
{

struct PointsNearPlaneGrabberParam2
{
	Eigen::Vector3f PlaneNormal   = { 0, 0, 1 };
	Eigen::Vector3f PlanePosition = { 0, 0, 0 };
	// The thickness(range) of getting points. In [Meter]. [0.02] means get points from range [-0.01
	// ~ +0.01].
	float PlaneThickness = 0.02f;
	//
	// EPlaneSegmentType PlaneSegmentType = EPlaneSegmentType::NearPlane;

	// A reference direction of the secondary axis when generate [Tlocal2world].
	Eigen::Vector3f Advance_PlanePoseSecondaryAxisRefDir = { 1, 0, 0 };

	// If True, the got points will be projected to the plane. If False, just get these points.
	bool bProjectToPlane = false;

	// Helper marco for easily serialize this structure from a json file or string.
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PointsNearPlaneGrabberParam2,
		PlaneNormal,
		PlanePosition,
		PlaneThickness,
		Advance_PlanePoseSecondaryAxisRefDir,
		bProjectToPlane);
};

template<typename PointType>
class TPlaneSegment
{
public:
	enum EPlaneSegmentType : int
	{
		AbovePlane = 0x01,
		BelowPlane = 0x02,
		NearPlane  = 0x04,
	};

	/* -------------------------------------------------------------------------- */
	/*                              Static Functions                              */
	/* -------------------------------------------------------------------------- */

private:
	static void __Segment__(PCPTR<PointType> InPC,
		Types::ConstVec3& InPlaneNormal,
		Types::ConstVec3& InPlanePosition,
		std::function<void(const int /* Index */, const PointType& /* PointData */, const float /* Point-plane
	                                                                                               Distance*/
			)> Func,
		const EPlaneSegmentType& InPlaneSegmentType = EPlaneSegmentType::NearPlane,
		PCIDX_Ptr InMask                            = PCIDX_Ptr(new PCIDX),
		const float& InPlaneThickness               = 0.0f)
	{
		auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_GlobalLogger", LogConfigJsonPath);

		Types::Vec3 PlaneNormal       = InPlaneNormal.normalized();
		const bool bGrabAbove         = (InPlaneSegmentType & AbovePlane) == AbovePlane;
		const bool bGrabBelow         = (InPlaneSegmentType & BelowPlane) == BelowPlane;
		const bool bGrabNear          = (InPlaneSegmentType & NearPlane) == NearPlane;
		const float DistanceThreshold = InPlaneThickness / 2;
		SPDLog::Log_T(Logger, 1, "PlaneSegment: Segment with DistanceThreshold:[{}]  Near:[{}]  Above:[{}]  Below:[{}]",
			DistanceThreshold, bGrabNear, bGrabAbove, bGrabBelow);

		PCIDX_Ptr RetPCIndex(new PCIDX);

		// Loop the source point cloud by index mask.
		if ( InMask && InMask->indices.size() > 0 )
		{
			SPDLog::Log_T(Logger, 1, "PlaneSegment:\t Segment with a mask (mask size [{}])  (input points size: [{}])", InMask->indices.size(), InPC->size());
			for ( int i = 0; i < InMask->indices.size(); i++ )
			{
				const int& IndexRef        = InMask->indices[i];
				const Types::Vec3 PointRef = (*InPC)[IndexRef].getVector3fMap().template cast<Types::CalcScalar>();

				// Calculate & Check point-plane distance.
				const float PointPlaneDistance =
					EigenHelper::PointToPlaneDistance(PointRef, PlaneNormal, InPlanePosition);
				// Logger->trace("PlaneSegment:\t\t Distance:[{}]  Threshold:[{}]", PointPlaneDistance,
				// DistanceThreshold);

				if ( (bGrabNear && abs(PointPlaneDistance) < DistanceThreshold) ||
					 (bGrabAbove && PointPlaneDistance > DistanceThreshold) ||
					 (bGrabBelow && PointPlaneDistance < -DistanceThreshold) )
				{
					// Logger->trace("PlaneSegment:\t\t\t Found satisfied point:  Index:[{}]  PointData:[{}"
					// 			  "]  Distance:[{}]",
					// 	IndexRef, (*InPC)[IndexRef], PointPlaneDistance);
					Func(IndexRef, (*InPC)[IndexRef], PointPlaneDistance);
				}
			}

			SPDLog::Log_T(Logger, 1, "PlaneSegment: Input size: [{}] -> Segment size: [{}]", InPC->size(),
				InMask->indices.size());
			return;
		}

		// Otherwise, loop each point in the source point cloud.
		SPDLog::Log_T(Logger, 1, "PlaneSegment:\t Segment with all input points (size: [{}])", InPC->size());
		int j = 0;
		for ( int i = 0; i < InPC->size(); i++ )
		{
			const Types::Vec3 PointRef = (*InPC)[i].getVector3fMap().template cast<Types::CalcScalar>();

			// Calculate & Check point-plane distance.
			const float PointPlaneDistance = EigenHelper::PointToPlaneDistance(PointRef, PlaneNormal, InPlanePosition);
			// Logger->trace("PlaneSegment:\t\t Distance:[{}]  Threshold:[{}]", PointPlaneDistance, DistanceThreshold);

			if ( (bGrabNear && abs(PointPlaneDistance) < DistanceThreshold) ||
				 (bGrabAbove && PointPlaneDistance > DistanceThreshold) ||
				 (bGrabBelow && PointPlaneDistance < -DistanceThreshold) )
			{
				// Logger->trace("PlaneSegment:\t\t\t Found satisfied point:  Index:[{}]  PointData:[{}, {}, "
				// 			  "{}]  Distance:[{}]",
				// 	InMask->indices[i], (*InPC)[i].x, (*InPC)[i].y, (*InPC)[i].z, PointPlaneDistance);
				Func(i, (*InPC)[i], PointPlaneDistance);
				j++;
			}
		}
		SPDLog::Log_T(Logger, 1, "PlaneSegment:\t Input size: [{}] -> Segment size: [{}]", InPC->size(), j);
	}

public:
	static PCIDX_Ptr Segment_ToIdx(PCPTR<PointType> InPC,
		Types::ConstVec3& InPlaneNormal,
		Types::ConstVec3& InPlanePosition,
		const EPlaneSegmentType& InPlaneSegmentType = EPlaneSegmentType::NearPlane,
		PCIDX_Ptr InMask                            = PCIDX_Ptr(new PCIDX),
		const float& InPlaneThickness               = 0.0f)
	{
		PCIDX_Ptr RetPCIndex(new PCIDX);
		auto RetPCIndexPtr = RetPCIndex.get();
		__Segment__(
			InPC, InPlaneNormal, InPlanePosition,
			[RetPCIndexPtr](const int Index, const PointType& PointData, const float InDistance)
			{
				RetPCIndexPtr->indices.push_back(Index);
			},
			InPlaneSegmentType, InMask, InPlaneThickness);
		return RetPCIndex;
	}

	static PCPTR<PointType> Segment_ToPC(PCPTR<PointType> InPC,
		Types::ConstVec3& InPlaneNormal,
		Types::ConstVec3& InPlanePosition,
		const EPlaneSegmentType& InPlaneSegmentType = EPlaneSegmentType::NearPlane,
		PCIDX_Ptr InMask                            = PCIDX_Ptr(new PCIDX),
		const float& InPlaneThickness               = 0.0f,
		const bool bInProjectToPlane                = false)
	{
		auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_GlobalLogger", LogConfigJsonPath);
		LOG_FUNC_ENTER(Logger, debug, 0);

		const Eigen::Matrix<float, 3, 1> PlaneNormal = InPlaneNormal.cast<float>();
		PCPTR<PointType> RetPC(new PC<PointType>);
		auto RetPCPtr = RetPC.get();
		__Segment__(
			InPC, InPlaneNormal, InPlanePosition,
			[RetPCPtr, PlaneNormal, bInProjectToPlane](const int Index, const PointType& PointData,
				const float InDistance)
			{
				if ( bInProjectToPlane )
				{
					const Eigen::Matrix<float, 3, 1> point = PointData.getVector3fMap();
					RetPCPtr->push_back(PointType());
					(*RetPCPtr).back().getVector3fMap() = EigenHelper::MovePoint(point, PlaneNormal, InDistance);
				}
				else
				{
					RetPCPtr->push_back(PointData);
				}
			},
			InPlaneSegmentType, InMask, InPlaneThickness);
		SPDLog::Log_D(Logger, 0, "PlaneSegment:\t Input size: [{}] -> Segment size: [{}] ;; Project: [{}]",
			InPC->size(), RetPC->size(), bInProjectToPlane);
		LOG_FUNC_EXIT(Logger, debug, 0);
		return RetPC;
	}
};

}; // namespace PCL_Helper

#endif /* E4B48B54_72CB_4680_A21B_F8AB86CDF600 */
