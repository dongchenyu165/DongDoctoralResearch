#include "CuttingFaceMaker.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <pcl/common/transforms.h>
#include <string>
#include <string_view>

#include <pcl/common/io.h>
#include <Eigen/src/Geometry/Transform.h>
#include "spdlog/logger.h"

#include <GlobalTypes.hpp>
#include <GlobalVars.hpp>
#include <GlobalConstantVars.hpp>
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Modules/DownSample.hpp"
#include "Utilities/PCL_Helper/Modules/PlaneSegment.hpp"
#include "Utilities/spdlog/LogConfig.hpp"
#include <Utilities/JSON_Helper/StructSerializer.hpp>
#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>
#include "Utilities/PCL_Helper/Modules/DownSample.hpp"
#include <Utilities/PCL_Helper/Modules/EuclideanCluster.hpp>
#include <Utilities/PCL_Helper/Basic/PointCloudInfo.hpp>
#include <Utilities/PCL_Helper/Apps/PointsArranger.hpp>
#include <Utilities/PCL_Helper/Visualizer/DebugViewerManager.hpp>

/* -------------------------------------------------------------------------- */
/*                               Inner Functions                              */
/* -------------------------------------------------------------------------- */
PCL_Helper::PCIDX_Ptr UnionPointIndex(PCL_Helper::PCIDX_Ptr InA, PCL_Helper::PCIDX_Ptr InB)
{
	NEW_PC_IDX_PTR(PointIdx);
	PointIdx->indices.reserve(InA->indices.size() + InB->indices.size());
	std::copy(InA->indices.begin(), InA->indices.end(), std::back_inserter(PointIdx->indices));
	std::copy(InB->indices.begin(), InB->indices.end(), std::back_inserter(PointIdx->indices));
	std::sort(PointIdx->indices.begin(), PointIdx->indices.end());
	auto DuplicatedInterator = std::unique(PointIdx->indices.begin(), PointIdx->indices.end());
	PointIdx->indices.erase(DuplicatedInterator, PointIdx->indices.end());

	return PointIdx;
}

Types::Vec3 AveragePoint(const Types::CalcPoint& InP1, const Types::CalcPoint& InP2)
{
	return ((InP1.getVector3fMap() + InP2.getVector3fMap()) / 2).cast<Types::CalcScalar>();
}

auto CalculateIntersect(const Types::CalcPoint& InLine1P1,
	const Types::CalcPoint& InLine1P2,
	const Types::CalcPoint& InLine2P1,
	const Types::CalcPoint& InLine2Dir)
{
	const auto Line1Direction = InLine1P2.getVector3fMap() - InLine1P1.getVector3fMap();
	const auto CrossVector    = InLine2P1.getVector3fMap() - InLine1P1.getVector3fMap();

	const Eigen::Vector3f h = InLine2Dir.getVector3fMap().cross(CrossVector);
	const Eigen::Vector3f k = InLine2Dir.getVector3fMap().cross(Line1Direction);

	const bool bNoIntersetion = h.norm() == 0 || k.norm() == 0; // either is 0.
	if ( bNoIntersetion )
	{
		return std::make_tuple(Eigen::Vector3f(), false);
	}

	float sign = h.dot(k) > 0 ? 1.0f : -1.0f;

	const Eigen::Vector3f FinalIntersection =
		InLine1P1.getVector3fMap() + sign * (h.norm() / k.norm()) * Line1Direction;

	Types::CalcPoint FinalIntersectionPoint(FinalIntersection.x(), FinalIntersection.y(), FinalIntersection.z());
	// return std::make_tuple(FinalIntersectionPoint, true);
	return std::make_tuple(FinalIntersection, true);
}

using PolyEdgeType = std::vector<Types::CalcPoint>;

struct FIntersectInfo
{
	Types::CalcPoint IntersectPoint;
	float t;
};

void GetIntersectedPolyEdge(std::vector<PolyEdgeType>& InPolyEdgeList,
	std::vector<FIntersectInfo>& OutIntersectInfoList,
	Types::CalcPoint InStartPoint)
{
	const Types::CalcPoint TEST_DIRECTION(1.f, 0.f, 0.f);
	float PointHeight = InStartPoint.y;

	OutIntersectInfoList.reserve(InPolyEdgeList.size());

	for ( int i = 0; i < InPolyEdgeList.size(); i++ )
	{
		const PolyEdgeType CurEdge = InPolyEdgeList[i];
		const bool bWithinHeight1  = CurEdge[0].y < PointHeight && PointHeight < CurEdge[1].y;
		const bool bWithinHeight2  = CurEdge[1].y < PointHeight && PointHeight < CurEdge[0].y;

		if ( bWithinHeight1 || bWithinHeight2 )
		{
			auto [IntersectPoint, bHasIntersection] =
				CalculateIntersect(CurEdge[0], CurEdge[1], InStartPoint, TEST_DIRECTION);

			FIntersectInfo info;
			info.IntersectPoint.getVector3fMap() = IntersectPoint;
			info.t = (IntersectPoint - InStartPoint.getVector3fMap()).norm() / TEST_DIRECTION.getVector3fMap().norm();

			OutIntersectInfoList.push_back(info);
		}
	}

	std::sort(OutIntersectInfoList.begin(), OutIntersectInfoList.end(),
		[](const FIntersectInfo& InLeft, const FIntersectInfo& InRight) -> bool
		{
			return InLeft.t < InRight.t;
		});
}

// Types::CalcPCPTR FillAreaWithPoints(Types::CalcPCPTR InBoundaryPC, Types::Vec3& InGridSize) { }

/* -------------------------------------------------------------------------- */
/*                              Member Functions                              */
/* -------------------------------------------------------------------------- */
CuttingFaceMaker::CuttingFaceMaker(const std::string& InConfigJsonPath,
	const TKnifeTrajectoryNode<Types::CalcScalar>& InTrajectoryNode)
	: ConfigJsonPath(InConfigJsonPath), TrajectoryInfo(InTrajectoryNode)
{
	Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("Global", LogConfigJsonPath);
	Logger->debug("CONSTRUCTOR: A [CuttingFaceMaker] object is created.");

	// Load json config object.
	std::ifstream f(InConfigJsonPath);
	Logger->debug("Loading json config from file {}.", InConfigJsonPath);
	ConfigObj = json::parse(f, nullptr, true, true)["Prepare-Data"]["MakeCuttingFace"];

	// json::dump() will not execute if no need to log.
	if ( Logger->should_log(spdlog::level::debug) )
	{
		// Logger->debug("Loaded json content:\n{}", ConfigObj.dump(8));
	}
}

auto CuttingFaceMaker::MakeCuttingPlaneEdge(Types::CalcPCPTR InPC,
	const float InKnifeBladeHalfAngle,
	Types::ConstMat4x4& InKnifeBasePose)
{
	using namespace PCL_Helper;
	LOG_FUNC_ENTER(Logger, debug, 1);

	/* -------------------------------------------------------------------------- */
	/*           1. Calculate [Blade Direction] and [CutPlane Position]           */
	/* -------------------------------------------------------------------------- */
	SPDLog::Log_D(Logger, 1, "1. Calculate [Blade Direction] and [CutPlane Position]");
	const Types::Vec3 BladeDirection = InKnifeBasePose.block<3, 1>(0, 0);
	SPDLog::Log_T(Logger, 2, "Blade Direction: [{}]", BladeDirection);
	const Types::Vec3 CutPlanePosition = InKnifeBasePose.block<3, 1>(0, 3);
	SPDLog::Log_T(Logger, 2, "CutPlane Position: [{}]", CutPlanePosition);
	const Types::Vec3 KnifePlaneNormal = InKnifeBasePose.block<3, 1>(0, 2);
	SPDLog::Log_T(Logger, 2, "Knife Plane Normal: [{}]", CutPlanePosition);

	/* -------------------------------------------------------------------------- */
	/*                     2. Calculate segment plane normal.                     */
	/* -------------------------------------------------------------------------- */
	SPDLog::Log_D(Logger, 1, "2. Calculate segment plane normal.");

	// Make rotation matrix by Angle-Axis: Angle: [BladeHalfAngle]   Axis: Blade Direction
	const auto PositiveRotMat = Eigen::AngleAxis<Types::CalcScalar>(InKnifeBladeHalfAngle, BladeDirection);
	const auto NegativeRotMat = Eigen::AngleAxis<Types::CalcScalar>(-InKnifeBladeHalfAngle, BladeDirection);

	// Make [Positive] & [Negative] plane normal by rotate blade's normal.
	/// Rotation part
	Types::Mat4x4 PositiveCutPlanePose =
		Eigen::Transform<Types::CalcScalar, 3, Eigen::Affine>(PositiveRotMat) * InKnifeBasePose;
	Types::Mat4x4 NegativeCutPlanePose =
		Eigen::Transform<Types::CalcScalar, 3, Eigen::Affine>(NegativeRotMat) * InKnifeBasePose;
	/// Translation part
	PositiveCutPlanePose.block<3, 1>(0, 3) = InKnifeBasePose.block<3, 1>(0, 3);
	NegativeCutPlanePose.block<3, 1>(0, 3) = InKnifeBasePose.block<3, 1>(0, 3);

	// Get the normal vector of [Positive] & [Negative] plane.
	const Types::Vec3 P_Normal = PositiveCutPlanePose.block<3, 1>(0, 2);
	const Types::Vec3 N_Normal = NegativeCutPlanePose.block<3, 1>(0, 2);
	SPDLog::Log_T(Logger, 2, "P_Normal: [{}]", P_Normal);
	SPDLog::Log_T(Logger, 2, "N_Normal: [{}]", N_Normal);

	/* -------------------------------------------------------------------------- */
	/*                 3.1 Make Rest Part(JUST For Visualization)                 */
	/* -------------------------------------------------------------------------- */
	using PlaneSegType      = TPlaneSegment<Types::CalcPoint>;
	using EPlaneSegmentType = PlaneSegType::EPlaneSegmentType;

	SPDLog::Log_D(Logger, 1, "3. Make Rest Part(JUST For Visualization).");

	SPDLog::Log_T(Logger, 2, "Segment Positive Index");
	PCIDX_Ptr PositivePointIdx =
		PlaneSegType::Segment_ToIdx(InPC, P_Normal, CutPlanePosition, EPlaneSegmentType::AbovePlane);

	SPDLog::Log_T(Logger, 2, "Segment Negative Index");
	PCIDX_Ptr NegativePointIdx =
		PlaneSegType::Segment_ToIdx(InPC, N_Normal, CutPlanePosition, EPlaneSegmentType::BelowPlane);

	SPDLog::Log_T(Logger, 2, "Union Neg and Positive Index");
	PCIDX_Ptr RestPointIdx = UnionPointIndex(PositivePointIdx, NegativePointIdx);
	NEW_CALC_PC_PTR(RestPC);
	pcl::copyPointCloud(*InPC, *RestPointIdx, *RestPC);

	/* -------------------------------------------------------------------------- */
	/*                           3.2 Make Grasping Part                           */
	/* -------------------------------------------------------------------------- */
	const float GraspingPartSliceThickness =
		ConfigObj["1.MakeCuttingEdgeParam"]["GraspingPartSliceThickness"].get<float>();
	SPDLog::Log_T(Logger, 2, "Union Neg and Positive Index");
	PCIDX_Ptr GraspingPointIdx = // Reserve point under the Negative plane
		PlaneSegType::Segment_ToIdx(InPC, N_Normal, CutPlanePosition, EPlaneSegmentType::BelowPlane, PCIDX_Ptr(),
			GraspingPartSliceThickness);
	GraspingPointIdx = // Reserve point under the Knife plane.
		PlaneSegType::Segment_ToIdx(InPC, KnifePlaneNormal, CutPlanePosition, EPlaneSegmentType::BelowPlane,
			GraspingPointIdx, GraspingPartSliceThickness);
	NEW_CALC_PC_PTR(GraspingPC);
	pcl::copyPointCloud(*InPC, *GraspingPointIdx, *GraspingPC);

	/* ----------------------- GraspingPart_DownSampleSize ---------------------- */
	auto GraspingPart_DownSampleSize = ConfigObj["1.MakeCuttingEdgeParam"]["GraspingPart_DownSampleSize"].template get<Eigen::Vector3f>();
	auto GraspingPart_DownSampled = VoxelDownSampler(GraspingPC, GraspingPart_DownSampleSize);

	SPDLog::Log_T(Logger, 2, "Segment result PC size: P: [{}] == N: [{}] == R: [{}]", PositivePointIdx->indices.size(),
		NegativePointIdx->indices.size(), RestPointIdx->indices.size());

	DEBUG_SHOW_PCIDX_LIST("RestPart", "CuttingFace", InPC, RestPointIdx, GraspingPointIdx);

	/* -------------------------------------------------------------------------- */
	/*                            4. Make Cutting Edge                            */
	/* -------------------------------------------------------------------------- */
	SPDLog::Log_D(Logger, 1, "4. Make Cutting Edge ");
	const float CuttingEdgeSliceThickness =
		ConfigObj["1.MakeCuttingEdgeParam"]["CuttingEdgeSliceThickness"].get<float>();

	SPDLog::Log_T(Logger, 2, "Slice Cross-Section of food PC by [Positive].");
	Types::CalcPCPTR CutEdge_P_Init = PlaneSegType::Segment_ToPC(InPC, P_Normal, CutPlanePosition,
		EPlaneSegmentType::NearPlane, PCIDX_Ptr(), CuttingEdgeSliceThickness, true);

	SPDLog::Log_T(Logger, 2, "Slice Cross-Section of food PC by [Negative].");
	Types::CalcPCPTR CutEdge_N_Init = PlaneSegType::Segment_ToPC(InPC, N_Normal, CutPlanePosition,
		EPlaneSegmentType::NearPlane, PCIDX_Ptr(), CuttingEdgeSliceThickness, true);

	SPDLog::Log_T(Logger, 2, "Cut lower part of [Positive] Cross-Section.");
	Types::CalcPCPTR CutEdge_P =
		PlaneSegType::Segment_ToPC(CutEdge_P_Init, N_Normal, CutPlanePosition, EPlaneSegmentType::AbovePlane);

	SPDLog::Log_T(Logger, 2, "Cut lower part of [Negative] Cross-Section.");
	Types::CalcPCPTR CutEdge_N =
		PlaneSegType::Segment_ToPC(CutEdge_N_Init, P_Normal, CutPlanePosition, EPlaneSegmentType::BelowPlane);

	SPDLog::Log_T(Logger, 2, "Cross-Section size: P: [{}] == N: [{}]", CutEdge_P_Init->size(), CutEdge_P_Init->size());
	SPDLog::Log_T(Logger, 2, "Cut Edge final size: P: [{}] == N: [{}]", CutEdge_P->size(), CutEdge_N->size());

	DEBUG_SHOW_PC_LIST("FinalCutEdge", "CuttingFace", CutEdge_P, CutEdge_N);

	LOG_FUNC_EXIT(Logger, debug, 1);
	return std::make_tuple(CutEdge_P, PositiveCutPlanePose, CutEdge_N, NegativeCutPlanePose, GraspingPart_DownSampled);
}

CuttingFaceResult CuttingFaceMaker::MakeCuttingFace(Types::CalcPCPTR InPC)
{
	using namespace PCL_Helper;

	LOG_FUNC_ENTER(Logger, debug, 0);
	Logger->info("Start to make cutting face with [{}] size point cloud.", InPC->size());

	CuttingFaceResult Result;
	OperatingPC = InPC;

	/* ------------- Make two [Cutting Edge] and [Grasping part PC] ------------- */
	SPDLog::Log_D(Logger, 1, "Make two [Cutting Edge] and [Grasping part PC]");
	const float KnifeBladeHalfAngle = ((float)(ConfigObj["KnifeParam"]["KnifeBladeAngle"])) * TO_RAD / 2;
	const Eigen::Matrix<Types::CalcScalar, 4, 4> InKnifeBasePose = TrajectoryInfo.Pose;
	// Return values:
	// xxxxOrgEdgePC:
	// xxxxCutPlanePose: This matrix is for rotating edge point cloud to XoY-plane for further arranging points.
	auto [PositiveOrgEdgePC, PositiveCutPlanePose, NegativeOrgEdgePC, NegativeCutPlanePose, GraspingPC] =
		MakeCuttingPlaneEdge(OperatingPC, KnifeBladeHalfAngle, InKnifeBasePose);
	Logger->info("Positive edge: [{}]; Neg edge: [{}]; Grasping part: [{}]", PositiveOrgEdgePC->size(),
		NegativeOrgEdgePC->size(), GraspingPC->size());
	Result.GraspingPC = GraspingPC;

	/* ------------------------- Down Sample the edge PC ------------------------ */
	const Types::Vec3 DownSampleSize = ConfigObj["2.CuttingEdgeDownSample"]["DownSampleSize"].get<Types::Vec3>();
	auto DownSampled_PEdge           = PCL_Helper::VoxelDownSampler(PositiveOrgEdgePC, DownSampleSize.cast<float>());
	auto DownSampled_NEdge           = PCL_Helper::VoxelDownSampler(NegativeOrgEdgePC, DownSampleSize.cast<float>());
	SPDLog::Log_D(Logger, 1, "Down Sample to [P Edge](size: [{}]) and [N Edge](size: [{}])", DownSampled_PEdge->size(),
		DownSampled_NEdge->size());

	/* ----------------------- Arrange Cutting Edge Points ---------------------- */
	App::TPointsArranger<Types::CalcPoint> Arranger;
	Arranger.ArrangePoints(DownSampled_PEdge);
	Types::CalcPCPTR Arranged_PEdge = Arranger.GetArrangedPC();
	App::TPointsArranger<Types::CalcPoint> ArrangerN;
	ArrangerN.ArrangePoints(DownSampled_NEdge);
	Types::CalcPCPTR Arranged_NEdge = ArrangerN.GetArrangedPC();

	/* ------------------------ Filling Cutting Edge Area ----------------------- */
	Types::Vec3 GridSize   = ConfigObj["4.FillCuttingFace"]["FillingGridSize"].get<Types::Vec3>();
	Result.CuttingFacePC_P = FillPolygon(Arranged_PEdge, PositiveCutPlanePose, GridSize);
	Result.CuttingFacePC_N = FillPolygon(Arranged_NEdge, NegativeCutPlanePose, GridSize);

	/* ----------------------------- Make Knife Line ---------------------------- */
	double StepSize                   = ConfigObj["5.BladeCurve"]["StepSize"].get<double>();
	Types::Vec3 StartPt               = AveragePoint(Arranged_PEdge->front(), Arranged_NEdge->front());
	Types::Vec3 EndPt                 = AveragePoint(Arranged_PEdge->back(), Arranged_NEdge->back());
	Result.KnifeBladePC = MakeKnifeBladeCurvePC(StartPt, EndPt, StepSize);
	DEBUG_SHOW_PC_LIST("FillResult", "CuttingFace", GraspingPC, Result.CuttingFacePC_P, Result.CuttingFacePC_N,
		Result.KnifeBladePC);
	/* -------------------------------------------------------------------------- */

	Result.StaticData.KnifePose = InKnifeBasePose;
	EvaluationStaticData::CuttingFaceInfoType CuttingFaceInfo;
	CuttingFaceInfo.Normal = PositiveCutPlanePose.block<3, 1>(0, 2);  // Use [Positive] pose's X-axis as normal.
	CuttingFaceInfo.Center = Result.CuttingFacePC_P->getMatrixXfMap().block(0, 0, 3, Result.CuttingFacePC_P->size() - 1).rowwise().mean().cast<Types::CalcScalar>();
	CuttingFaceInfo.PlanePose = PositiveCutPlanePose;
	Result.StaticData.CuttingFaceInfoList.push_back(CuttingFaceInfo);

	CuttingFaceInfo = EvaluationStaticData::CuttingFaceInfoType();
	CuttingFaceInfo.Normal = NegativeCutPlanePose.block<3, 1>(0, 2);  // Use [Negative] pose's X-axis as normal.
	CuttingFaceInfo.Center = Result.CuttingFacePC_N->getMatrixXfMap().block(0, 0, 3, Result.CuttingFacePC_N->size() - 1).rowwise().mean().cast<Types::CalcScalar>();
	CuttingFaceInfo.PlanePose = NegativeCutPlanePose;
	Result.StaticData.CuttingFaceInfoList.push_back(CuttingFaceInfo);

	// Fill the cutting face's normal field.
	Result.CuttingFacePC_P->getMatrixXfMap().row(3).setConstant(Result.StaticData.CuttingFaceInfoList[0].Normal.x());
	Result.CuttingFacePC_P->getMatrixXfMap().row(4).setConstant(Result.StaticData.CuttingFaceInfoList[0].Normal.y());
	Result.CuttingFacePC_P->getMatrixXfMap().row(5).setConstant(Result.StaticData.CuttingFaceInfoList[0].Normal.z());
	Result.CuttingFacePC_N->getMatrixXfMap().row(3).setConstant(Result.StaticData.CuttingFaceInfoList[1].Normal.x());
	Result.CuttingFacePC_N->getMatrixXfMap().row(4).setConstant(Result.StaticData.CuttingFaceInfoList[1].Normal.y());
	Result.CuttingFacePC_N->getMatrixXfMap().row(5).setConstant(Result.StaticData.CuttingFaceInfoList[1].Normal.z());

	LOG_FUNC_EXIT(Logger, debug, 0);
	return Result;
}

Types::CalcPCPTR CuttingFaceMaker::FillPolygon(Types::CalcPCPTR InPolygonPC,
	const Types::Mat4x4& InTlocal2world,
	const Types::Vec3& InGridSize)
{
	using namespace PCL_Helper;

	LOG_FUNC_ENTER(Logger, debug, 0);

	struct FValidSegment
	{
		float St = -INFINITY;
		float Ed = -INFINITY;
	};

	NEW_CALC_PC_PTR(TransformedPolygonPC);
	pcl::transformPointCloud(*InPolygonPC, *TransformedPolygonPC, InTlocal2world.inverse().cast<float>());

	std::vector<PolyEdgeType> PolyEdgeList;
	PolyEdgeList.reserve(TransformedPolygonPC->size() * 2);

	for ( int i = 0; i < TransformedPolygonPC->size(); i++ )
	{
		int NextIndex = (i + 1) % TransformedPolygonPC->size();
		PolyEdgeList.push_back({ (*TransformedPolygonPC)[i], (*TransformedPolygonPC)[NextIndex] });
	}

	NEW_CALC_PC_PTR(OutFinalFilledPC);
	pcl::copyPointCloud(*TransformedPolygonPC, *OutFinalFilledPC);

	PCL_Helper::TPointCloudInfo<Types::CalcPoint> PC_Info(TransformedPolygonPC);
	Types::CalcPoint Min;
	Types::CalcPoint Max;
	Min.getVector3fMap() = PC_Info.GetAABB().Min;
	Max.getVector3fMap() = PC_Info.GetAABB().Max;

	const int FILL_ROW_COUNT = (int)((Max.y - Min.y) / InGridSize.y()) - 1;

	// Fill Rows
	for ( int i = 1; i < FILL_ROW_COUNT + 1; i++ )
	{
		float CurrentY = Min.y + i * InGridSize.y();

		std::vector<FIntersectInfo> IntersectInfoList;
		GetIntersectedPolyEdge(PolyEdgeList, IntersectInfoList, Types::CalcPoint(Min.x - 0.01, CurrentY, 0));

		if ( IntersectInfoList.size() == 0 )
		{
			continue;
		}

		// Iterate each intersect point, intersected between row line and the polygon edge
		bool bFlipFlop = true; // Initial value set true, because The segment between intersect point and the next
		                       // intersect point is in the polygon.
		for ( int j = 0; j < IntersectInfoList.size() - 1; j++ )
		{
			const FIntersectInfo& Info   = IntersectInfoList[j];
			const FIntersectInfo& InfoEd = IntersectInfoList[j + 1];
			if ( bFlipFlop )
			{
				// Fill points in this segment.
				for ( float X = 0; Info.IntersectPoint.x + X < InfoEd.IntersectPoint.x; X += InGridSize.x() )
				{
					OutFinalFilledPC->push_back(Types::CalcPoint(Info.IntersectPoint.x + X, CurrentY, 0));
				}
				OutFinalFilledPC->push_back(InfoEd.IntersectPoint);
			}
			bFlipFlop = !bFlipFlop;
		}
	}

	pcl::transformPointCloud(*OutFinalFilledPC, *OutFinalFilledPC, InTlocal2world.cast<float>());

	LOG_FUNC_EXIT(Logger, debug, 0);
	return OutFinalFilledPC;
}

Types::CalcPCPTR CuttingFaceMaker::MakeKnifeBladeCurvePC(const Types::Vec3& InStartPt,
	const Types::Vec3& InEndPt,
	const float InGridSize)
{
	using namespace Types;
	Vec3 Dir          = InEndPt - InStartPt;
	double Length     = Dir.norm();
	size_t PointCount = (size_t)(Length / InGridSize) - 1;

	CalcPCPTR KnifeLinePC = CalcPCPTR(new CalcPC);
	CalcPoint TempPt;
	Vec3 Step = Dir * InGridSize / Length;
	for ( size_t i = 0; i < PointCount; i++ )
	{
		TempPt.getVector3fMap() = (InStartPt + i * Step).cast<float>();
		KnifeLinePC->push_back(TempPt);
	}
	return KnifeLinePC;
}

void CuttingFaceMaker::DenoiseByEuclideanCluster()
{
	using namespace PCL_Helper;
	PCPTR_List<Types::CalcPoint> ClusterList;
	EC_Setting EuclideanSetting = ConfigObj["Denoise_EuclideanClusterSetting"].get<EC_Setting>();
	ExtractEuclideanCluster(OperatingPC, ClusterList, EuclideanSetting);

	OperatingPC = ClusterList[0]; // Get the largest point cloud.
}
