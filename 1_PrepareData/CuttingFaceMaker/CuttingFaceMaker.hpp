#ifndef DDD056D7_A1AC_460A_A418_2018CD6E2D17
#define DDD056D7_A1AC_460A_A418_2018CD6E2D17

// #include "Utilities/spdlog/LogConfig.hpp"
#include <string>

#include <nlohmann/json.hpp>

#include <GlobalTypes.hpp>
#include <DataTypes/KnifeTrajectoryNode.hpp>

// Just a forward declaration
namespace spdlog
{
class logger;
}


struct CuttingFaceResult
{
	Types::CalcPCPTR CuttingFacePC_P;
	Types::CalcPCPTR CuttingFacePC_N;
	Types::CalcPCPTR KnifeBladePC;
	Types::CalcPCPTR GraspingPC;
};

class CuttingFaceMaker
{
public:
	CuttingFaceMaker(const std::string& InConfigJsonPath,
		const TKnifeTrajectoryNode<Types::CalcScalar>& InTrajectoryNode);

	/**
	 * @brief Main using function.
	 * Take a food point cloud, and output the cutting face point cloud
	 *
	 * @param InPC Input food point cloud.
	 * @return Types::CalcPCPTR Generated point cloud of cutting face.
	 */
	CuttingFaceResult MakeCuttingFace(Types::CalcPCPTR InPC);

protected:
	Types::CalcPCPTR FillPolygon(Types::CalcPCPTR InPolygonPC, const Types::Mat4x4& InTlocal2world, const Types::Vec3& InGridSize);
	Types::CalcPCPTR MakeKnifeBladeCurvePC(const Types::Vec3& InStartPt, const Types::Vec3& InEndPt, const float InGridSize);
	void DenoiseByEuclideanCluster();
	auto MakeCuttingPlaneEdge(Types::CalcPCPTR InPC,
		const float InKnifeBladeHalfAngle,
		Types::ConstMat4x4& InKnifeBasePose);

	/* -------------------------------------------------------------------------- */
	/*                                  Variables                                 */
	/* -------------------------------------------------------------------------- */
protected:
	std::string ConfigJsonPath;
	TKnifeTrajectoryNode<Types::CalcScalar> TrajectoryInfo;

	Types::CalcPCPTR OperatingPC;
	nlohmann::json ConfigObj;

	std::shared_ptr<spdlog::logger> Logger;
};

#endif /* DDD056D7_A1AC_460A_A418_2018CD6E2D17 */
