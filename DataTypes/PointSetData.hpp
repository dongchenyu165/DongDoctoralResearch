#ifndef E27503BF_E094_4CD3_85DA_CEAE08A17985
#define E27503BF_E094_4CD3_85DA_CEAE08A17985

#include <GlobalBaseTypes.hpp>
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

	// These matrix are generate by [Search space generator]
	Eigen::Matrix<Types::CalcScalar, ForceCount, 3> PositionPair;
	Eigen::Matrix<Types::CalcScalar, ForceCount, 3> NormalPair;
	Eigen::Matrix<int, ForceCount, 1> PointIndexPair;
	
	bool bIsIgnored = false;  // Flag this element is ignored from all of the further calculation.
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
	Eigen::Matrix<Types::CalcScalar, 4, 4> KnifePose;
	struct CuttingFaceInfoType
	{
		// The point center of the cutting face.
		Types::Vec3 Center;
		// The normal vector of the cutting face.
		Types::Vec3 Normal;
		// The pose of the cutting face. Calculated by rotating the knife pose to the cutting face normal.
		// Y-axis to cutting face normal, X-axis to the blade
		Types::Mat4x4 PlanePose;
	} ;
	std::vector<CuttingFaceInfoType> CuttingFaceInfoList;
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

	// Grasping force list for each trajectory node.
	std::vector<Eigen::Matrix<float, ForceCount, 3>> GraspingForcePairList;

};


#endif /* E27503BF_E094_4CD3_85DA_CEAE08A17985 */
