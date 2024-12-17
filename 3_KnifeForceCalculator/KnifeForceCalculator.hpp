#ifndef E32F9EAB_BC75_40C9_A084_C7080C22DD41
#define E32F9EAB_BC75_40C9_A084_C7080C22DD41


// Standard library headers
// #include <cassert>
#include <vector>

// External library headers
#include <nlohmann/json.hpp>

// Project headers
#include "1_PrepareData/CuttingFaceMaker/CuttingFaceMaker.hpp"
#include "DataTypes/PointSetData.hpp"
#include "GlobalBaseTypes.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Basic/PlyMeshCreator.hpp"
#include "Utilities/spdlog/FunctionAutoLogger.hpp"
#include "Utilities/spdlog/LogConfig.hpp"


class FaceForceCalculator
{
public:
	FaceForceCalculator(Types::CalcPCPTR InCuttingFacePC,
		const nlohmann::json& InConfigJson,
		SPDLog::LoggerType InLogger = nullptr)
		: CuttingFacePC(InCuttingFacePC), CalKnifeForceConfigJson(InConfigJson), Logger(InLogger)
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);

		LOG_INDENT(Logger, info, "Creating cutting face MESH from point cloud");
		CuttingFaceMesh = PCL_Helper::CreatePlyMesh(CuttingFacePC, CalKnifeForceConfigJson["CuttingFaceTriangulation"], InLogger);
		assert(CuttingFaceMesh);
		LOG_INDENT(Logger, debug, "Mesh created with {} vertices and {} faces", CuttingFaceMesh->cloud.width, CuttingFaceMesh->polygons.size());

		LOG_INDENT(Logger, info, "Storage the face (3 point index of a face) list.");
		for ( auto FaceIt = CuttingFaceMesh->polygons.begin(); FaceIt != CuttingFaceMesh->polygons.end(); FaceIt++ )
		{
			const pcl::Indices& VerticesList = (*FaceIt).vertices;
			ValidFaceList.insert(ValidFaceList.end(), VerticesList);

			LOG_INDENT(Logger, trace, "Face point index: [{}], [{}], [{}]", VerticesList[0], VerticesList[1], VerticesList[2]);
		}
	}

	using FaceOperationCallbackType = std::function<
		void(const Types::Vec3& /* InP1 */, const Types::Vec3& /* InP2 */, const Types::Vec3& /* InP3 */, 
			const int& /* InP1_Index */, const int& /* InP2_Index */, const int& /* InP3_Index */)>;

	Types::ForceTorqueType CalculateForces(const std::string& InFoodName = "Potato", const Types::Vec3& InKnifeVelocity = Types::Vec3::Zero(), const Types::Vec3& InCenterOfMass = Types::Vec3::Zero())
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);
		CalculateTotalArea();

		FoodParamJson = CalKnifeForceConfigJson["FoodParams"][InFoodName];
		TotalKnifeMoveDirection = InKnifeVelocity.normalized();
		mu = FoodParamJson["Friction_Mu"].template get<double>();
		CenterOfMass = InCenterOfMass;
		CalculateFrictionForceAndTorque(InFoodName);

		return ResultForce;
	}

protected:
	// Custom pressure distribution calculation.
	virtual Types::CalcScalar PressureDistribution(const Types::Vec3& P1, const Types::Vec3& P2, const Types::Vec3& P3, const int& P1_Index,
				const int& P2_Index, const int& P3_Index)
	{
		return static_cast<double>(FoodParamJson["FrictionPressForce"]) / TotalArea;
	}

private:
	void FaceWiseOperation(FaceOperationCallbackType FaceOperation)
	{
		for ( auto FaceIt = CuttingFaceMesh->polygons.begin(); FaceIt != CuttingFaceMesh->polygons.end(); FaceIt++ )
		{
			const pcl::Vertices& vertices = *FaceIt;
			if ( vertices.vertices.size() == 3 )
			{
				const Types::Vec3& P1 = (*CuttingFacePC)[vertices.vertices[0]].getVector3fMap().cast<Types::CalcScalar>();
				const Types::Vec3& P2 = (*CuttingFacePC)[vertices.vertices[1]].getVector3fMap().cast<Types::CalcScalar>();
				const Types::Vec3& P3 = (*CuttingFacePC)[vertices.vertices[2]].getVector3fMap().cast<Types::CalcScalar>();

				FaceOperation(P1, P2, P3, vertices.vertices[0], vertices.vertices[1], vertices.vertices[2]);
			}
		}
	}

	void CalculateTotalArea()
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);
		TotalArea = 0.0f;
		FaceWiseOperation(
			[this](const Types::Vec3& P1, const Types::Vec3& P2, const Types::Vec3& P3, const int& P1_Index,
				const int& P2_Index, const int& P3_Index)
			{
				Types::ConstVec3 Edge1 = P2 - P1;
				Types::ConstVec3 Edge2 = P3 - P1;
				const Types::CalcScalar Area  = std::abs(0.5 * Edge1.cross(Edge2).norm());
				TotalArea         += Area;

				if (Logger->should_log(spdlog::level::trace))
				{
					FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);
					LOG_INDENT(Logger, trace, "Edge1: [{}], Edge2: [{}], Area: [{}], TotalArea: [{}] ", Edge1, Edge2, Area, TotalArea);
				}
			});
	}

	void CalculateFrictionForceAndTorque(const std::string& InFoodName = "Potato")
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);
		ResultForce.setZero();
		LOG_INDENT(Logger, debug, "Friction mu: [{}]; Knife Vel Dir: [{}] CenterOfMass: [{}]", FoodParamJson["Friction_Mu"].template get<double>(), TotalKnifeMoveDirection, CenterOfMass);

		FaceWiseOperation(
			[this](const Types::Vec3& P1, const Types::Vec3& P2, const Types::Vec3& P3, const int& P1_Index,
				const int& P2_Index, const int& P3_Index)
			{
				const Types::Vec3 FaceCenter = (P1 + P2 + P3) / 3;

				// Single triangle area
				const Types::Vec3 Edge1 = P2 - P1;
				const Types::Vec3 Edge2 = P3 - P1;
				const double Area = std::abs(0.5 * Edge1.cross(Edge2).norm());

				const double Pressure = PressureDistribution(P1, P2, P3, P1_Index, P2_Index, P3_Index);

				// Force calculation.
				const Types::Vec3 dForceFriction = TotalKnifeMoveDirection * mu * Pressure * Area;
				ResultForce.template block<3, 1>(0, 0) += dForceFriction;

				// Torque calculation.
				const Types::Vec3 PointToCenter = FaceCenter - CenterOfMass;
				const Types::Vec3 dTorqueFriction = PointToCenter.cross(dForceFriction);
				ResultForce.template block<3, 1>(3, 0) += dTorqueFriction;
				
				// Prevent the enter message printing when the logging level is not enough.
				if (Logger->should_log(spdlog::level::trace))
				{
					FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);
					LOG_INDENT(Logger, trace, "Edge1: [{}], Edge2: [{}], Area: [{}], Pressure: [{}] ", Edge1, Edge2, Area, Pressure);
					LOG_INDENT(Logger, trace, "dForceFriction: [{}]", dTorqueFriction);

					LOG_INDENT(Logger, trace, "FaceCenter: [{}]  PointToCenter[{}]", FaceCenter, PointToCenter);
					LOG_INDENT(Logger, trace, "dTorqueFriction: [{}]", dTorqueFriction);

					LOG_INDENT(Logger, trace, "ResultForce: [{}]", ResultForce);
				}
			});
	}

protected:
	Types::CalcPCPTR CuttingFacePC;
	const nlohmann::json& CalKnifeForceConfigJson;  // Param json with key ["CalKnifeForce"]
	SPDLog::LoggerType Logger;

	PCL_Helper::PlyMesh_Ptr CuttingFaceMesh;
	std::vector<pcl::Indices> ValidFaceList;

protected:
	double TotalArea = 0.0f;
	double mu = 0.0f;  // 从ConfigJson读取
	nlohmann::json FoodParamJson;
	Types::ForceTorqueType ResultForce = Types::ForceTorqueType::Zero();
	// Types::Vec3 ResultTorque = Types::Vec3::Zero();
	Types::Vec3 CenterOfMass = Types::Vec3::Zero();
	Types::Vec3 TotalKnifeMoveDirection = Types::Vec3::Zero();

};



class KnifeForceCalculator
{
public: 
/**
 * "CalKnifeForce":{
		"FoodParams":{
			"Potato":{
				"Friction_Mu": 0.6,
				"FractureToughness": 200
			},
			"Onion":{
				"Friction_Mu": 0.6,
				"FractureToughness": 600
			}
		},
		"PushingForce": 20.0
	}
 * 
 */

/**
 * @brief Construct a new Knife Force Calculator object
 * 
 * @param InPC 
 * @param InConfigJson 
 */
	KnifeForceCalculator(CuttingFaceResult& InCuttingFaceResultObj, const nlohmann::json& InConfigJson, SPDLog::LoggerType InLogger = nullptr)
	: 	CuttingFaceResultObj(InCuttingFaceResultObj), 
		CalculationStaticData(InCuttingFaceResultObj.StaticData),
		ConfigJson(InConfigJson["CalKnifeForce"]), Logger(InLogger)
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);

		LOG_INDENT(Logger, info, "Initial KnifeForceCalculator.");
		FaceForceCalculatorList.push_back({InCuttingFaceResultObj.CuttingFacePC_P, ConfigJson, InLogger});
		FaceForceCalculatorList.push_back({InCuttingFaceResultObj.CuttingFacePC_N, ConfigJson, InLogger});
	}

	Types::ForceTorqueType CalculateKnifeForce(const Types::Vec3& InKnifeVelocity, const std::string& InFoodName = "Potato")
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);

		LOG_INDENT(Logger, info, "1. Calculate Friction Force.");
		Types::ForceTorqueType OutCuttingForce = Types::ForceTorqueType::Zero(6, 1);
		for (auto& FaceForceCalculator : FaceForceCalculatorList)
		{
			OutCuttingForce += FaceForceCalculator.CalculateForces(InFoodName, InKnifeVelocity, CalculationStaticData.CenterOfMass);
			LOG_INDENT(Logger, info, "--> Face Force Result: [{}]", OutCuttingForce);
		}

		LOG_INDENT(Logger, info, "2. Calculate Fracture Force.");
		Types::ForceTorqueType OutCuttingFractureForce = CuttingFractureForce(CuttingFaceResultObj.KnifeBladePC, 
			ConfigJson["FoodParams"][InFoodName]["FractureToughness"], CalculationStaticData.CenterOfMass, InKnifeVelocity, Logger);
		OutCuttingForce += OutCuttingFractureForce;

		LOG_INDENT(Logger, info, "Knife Force Result: [{}]", OutCuttingForce);
		return OutCuttingForce;
	}

private:
	static Types::ForceTorqueType CuttingFractureForce(
		Types::CalcPCPTR InBladeCurvePointList, 
		Types::CalcScalar InFractureToughness, 
		const Types::Vec3& CenterOfMass,
		Types::ConstVec3& InKnifeVelocity,
		SPDLog::LoggerType Logger = nullptr)
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);
		static_assert(pcl::traits::has_normal<Types::CalcPoint>::value, "The point type must have a normal field.");

		const Types::CalcPC& BladeCurve = *InBladeCurvePointList;
		LOG_INDENT(Logger, debug, "0. Blade has {} points.", InBladeCurvePointList->size());
		
		Types::ConstVec3 KnifeVelocityDir = InKnifeVelocity.normalized();
		LOG_INDENT(Logger, debug, "0. Knife velocity direction: [{}], FractureToughness: [{}]", KnifeVelocityDir, InFractureToughness);

		Types::ForceTorqueType OutCuttingForce = Types::ForceTorqueType::Zero(6, 1);

		for(int i = 0; i < InBladeCurvePointList->size() - 1; i++)
		{
			const Types::CalcPoint& SegPointA = BladeCurve[i];
			const Types::CalcPoint& SegPointB = BladeCurve[i + 1];

			Types::ConstVec3 SegNormal = -((SegPointA.getNormalVector3fMap() + SegPointB.getNormalVector3fMap()) / 2.0f).cast<double>();
			Types::ConstVec3 SegCenter = ((SegPointA.getVector3fMap() + SegPointB.getVector3fMap()) / 2.0f).cast<double>();
			const double SegLength = (SegPointA.getVector3fMap() - SegPointB.getVector3fMap()).norm();  // ds_\beta

			LOG_INDENT(Logger, trace, "Segment {}: Normal=[{}], Center=[{}], Length={}", 
				i, SegNormal, SegCenter, SegLength);

			Types::ConstVec3&& df_c = InFractureToughness * KnifeVelocityDir.dot(SegNormal) * KnifeVelocityDir * SegLength;
			OutCuttingForce.template block<3, 1>(0, 0) += df_c;

			Types::ConstVec3&& dt_c = (SegCenter - CenterOfMass).cross(df_c);
			OutCuttingForce.template block<3, 1>(3, 0) += dt_c;

			LOG_INDENT(Logger, trace, "Force contribution: [{}], Torque contribution: [{}]", df_c, dt_c);
		}

		LOG_INDENT(Logger, info, "1. Total fracture force and torque: [{}]", OutCuttingForce);
		return OutCuttingForce;
	}

private:
	CuttingFaceResult& CuttingFaceResultObj;
	Types::CalcPCPTR CuttingFacePC;
	EvaluationStaticData& CalculationStaticData;
	const nlohmann::json& ConfigJson;
	SPDLog::LoggerType Logger;

	std::vector<FaceForceCalculator> FaceForceCalculatorList;

};

#endif /* E32F9EAB_BC75_40C9_A084_C7080C22DD41 */
