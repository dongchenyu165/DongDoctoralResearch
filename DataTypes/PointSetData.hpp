#ifndef E27503BF_E094_4CD3_85DA_CEAE08A17985
#define E27503BF_E094_4CD3_85DA_CEAE08A17985

#include "GlobalVars.hpp"
#include <Eigen/Core>
#include <Eigen/src/Core/IO.h>

template<int ForceCount = 3>
struct TSearchSpaceElement
{
	// For search space generating.
	static constexpr int FORCE_COUNT = ForceCount;
	static constexpr int FINGER_COUNT = bCONSIDER_GRAVITY ? ForceCount - 1 : ForceCount;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/* ------------------------------- Basic Vars ------------------------------- */
	// Generate by [Search space generator]
	Eigen::Matrix<float, ForceCount, 3> PositionPair;
	Eigen::Matrix<float, ForceCount, 3> NormalPair;
	Eigen::Matrix<int, ForceCount, 1> PointIndexPair;

	/* ------------------------------ Runtime vars ------------------------------ */
	bool bIsSucceedInfo = true; // PositionPair 是否含有一个可使用的抓取力。,标记没有找到力的点对。

	// Score calculation var
	float PositionScore = 0.0;
	float ForceScore    = 0.0;
	Eigen::Matrix<double, ForceCount, 3> ForcePair; // Force score calculation.
	Eigen::Matrix<float, 4, 4> KnifePose;          // Force score calculation.
	///// Eigen::Matrix<double, ForceCount, 3> KnifeMovingDirection;  // DEP

	// Vis var
	Eigen::Matrix<float, 3, 1> CuttingFaceCenter;
	///// Eigen::Matrix<float, 3, 1> CuttingFaceNormal;
	std::vector<Eigen::Matrix<float, ForceCount, 3>> ForcePairCandidateList;

	std::string ToString()
	{
		std::stringstream StrStream;

		Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, 0, ", ", ", ", "\n\t\t", "]");
		StrStream 	<< "Info of point set in ADDR:[0x" << std::hex << ((uint64_t)(this)) << "]. Basic Vars:" << std::dec << std::endl
					<< "\tPoint Position: " << PositionPair.format(CommaInitFmt) << std::endl
					<< "\tPoint Normal: " << NormalPair.format(CommaInitFmt) << std::endl
					<< "\tPoint Index: " << PointIndexPair.transpose() << std::endl;
		
		return StrStream.str();
	}
};

struct EvaluationStaticData
{
	Eigen::Matrix<float, 4, 4> KnifePose;
	Eigen::Matrix<double, 3, 1> CuttingFaceCenter;
	Eigen::Matrix<double, 3, 1> CuttingFaceNormal;
};

template<int ForceCount = 3>
struct TSearchSpaceElement_ForceEvaluation : public TSearchSpaceElement<ForceCount>, public EvaluationStaticData
{
	bool bIsSucceedInfo = true; // PositionPair 

	// Score calculation var
	float PositionScore = 0.0;
	float ForceScore    = 0.0;
	Eigen::Matrix<double, ForceCount, 3> ForcePair; // Force score calculation.
};

template<int ForceCount = 3>
struct TGraspingResult : public TSearchSpaceElement<ForceCount>, public EvaluationStaticData
{
	float PositionScore = 0.0;
	float FinalScore    = 0.0;

	// Grasping force for each trajectory node.
	std::vector<Eigen::Matrix<float, ForceCount, 3>> GraspingForcePairList;

};


#endif /* E27503BF_E094_4CD3_85DA_CEAE08A17985 */
