#ifndef D844E03F_138B_4DA6_9F45_E4236E72A781
#define D844E03F_138B_4DA6_9F45_E4236E72A781

#include <algorithm>
#include <cstddef>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <fstream>
#include "4_ForceScore/FingerForceGenerator.hpp"
#include "GlobalBaseTypes.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
// #include "Utilities/JSON_Helper/EigenSeiralizer.hpp"

class FingerForceGeneratorTest : public ::testing::Test
{
protected:
	using GeneratorType = TFingerForceGeneratorWithinCone<double, 3>;
	using GeneratorBaseType = GeneratorType::Super;
	using TestingCalcPointSetData = TSearchSpaceElement<3>;
	using TestingCalcPointSetDataPtr = std::shared_ptr<TestingCalcPointSetData>;

	void SetUp() override
	{
		// Initialize with test parameters
		Seed = 42; // Specific seed for reproducibility
		
		// CoM
		// {0.53020584583282471, -0.2218005508184433, 0.049539912492036819}
		// ExtForce
		// {4.2400116920471191, -6.9533286210677125E-310, 11.777081418037415, -1.0552048683166504, -1.6387150287628174, 2.2629725933074951}
		
		// Points
		// {0.516943336, -0.238809869, 0.0564656109, 
		// 0.507865191, -0.220309898, 0.0582243688, 
		// 0.527730584, -0.221327618, 0.010671692}

		// Normals
		// {0.36580804, 0.476029277, -0.799737811, 
		// 0.497173399, 0.183084592, -0.848114788, 
		// 0, 0, 1}

		// Types::ForceTorqueType InKnifeForce;
		// InKnifeForce << 0, 0, -10, 0, 0, 0;
		// Types::Vec3 InCoM;
		// InCoM << 0, 0, 0;
		// InPointSetDataPtr->PositionPair << 1, 0, -2, -1, 0, -2;
		// InPointSetDataPtr->NormalPair << 1, 0, 1, -1, 0, 1;
		// InPointSetDataPtr->NormalPair.row(0).normalized();
		// InPointSetDataPtr->NormalPair.row(1).normalized();

		Types::ForceTorqueType InKnifeForce;
		InKnifeForce << 4.2400116920471191, -6.9533286210677125E-310, 11.777081418037415, -1.0552048683166504, -1.6387150287628174, 2.2629725933074951;
		Types::Vec3 InCoM;
		InCoM << 0.53020584583282471, -0.2218005508184433, 0.049539912492036819;
		InPointSetDataPtr->PositionPair << 	0.516943336, -0.238809869, 0.0564656109, 
											0.507865191, -0.220309898, 0.0582243688, 
											0.527730584, -0.221327618, 0.010671692;
		InPointSetDataPtr->NormalPair << 	0.36580804, 0.476029277, -0.799737811, 
											0.497173399, 0.183084592, -0.848114788, 
											0, 0, 1;
		InPointSetDataPtr->NormalPair.row(0).normalized();
		InPointSetDataPtr->NormalPair.row(1).normalized();
		InPointSetDataPtr->NormalPair.row(2).normalized();


		// InPointSetDataPtr->;
		size_t InFingerForceGenCount = 800;

		std::string JsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/params.json";

		// Load json from [JsonPath]
		std::string JsonStr = R"(
{	
	"CalForceScore":{
		"ForceGenBasicParam":{
			"AngleLimit": 70,
			"GeneratingRetryTimes" : 600
		},
		"FingerForceGeneratorWithinCone_Param":{
			"ForceRangeMin" : 1,
			"ForceRangeMax" : 600,
			"RandomHalfAngle" : 30
		},
		"FingerForceGeneratorByRandomK_Param":{
			"ForceRangeMin" : 5,
			"ForceRangeMax" : 300
		}
	}
})";
		json JsonObj        = json::parse(JsonStr);

		Generator =
			new GeneratorType(InKnifeForce, InCoM, InPointSetDataPtr, InFingerForceGenCount, JsonObj);

		const json& BasicParamJsonObj = JsonObj["CalForceScore"]["ForceGenBasicParam"];
		const json& GeneratorJsonObj  = JsonObj["CalForceScore"]["FingerForceGeneratorWithinCone_Param"];

		AngleLimit           = BasicParamJsonObj["AngleLimit"].get<double>() * M_PI / 180.0;
		GeneratingRetryTimes = BasicParamJsonObj["GeneratingRetryTimes"];
		ForceRangeMin        = GeneratorJsonObj["ForceRangeMin"];
		ForceRangeMax        = GeneratorJsonObj["ForceRangeMax"];
		RandomHalfAngle      = GeneratorJsonObj["RandomHalfAngle"].get<double>() * M_PI / 180.0;

		AngleLimit = std::min(AngleLimit, RandomHalfAngle);
	}

	double ForceRangeMin;
	double ForceRangeMax;
	double RandomHalfAngle;
	double AngleLimit;
	int GeneratingRetryTimes;
	unsigned int Seed;
	GeneratorBaseType* Generator;
	TestingCalcPointSetDataPtr InPointSetDataPtr{ new TestingCalcPointSetData };
};

// If change the test name, please change the friend class name in the all classes [FingerForceGenerator] and it's
// sub-class.
TEST_F(FingerForceGeneratorTest, ParamSetTest)
{
	// [FingerForceGenerator]'s config parameters.
	EXPECT_EQ(Generator->GeneratingRetryTimes, GeneratingRetryTimes);
	EXPECT_NEAR(Generator->AngleLimit, AngleLimit, 1e-6); // Convert degree to radian internally.

	// [FingerForceGenerator]'s sub-classes config parameters.
	auto ConeGenPtr = dynamic_cast<GeneratorType*>(Generator);
	if ( ConeGenPtr )
	{
		EXPECT_NEAR(ConeGenPtr->LengthDist.min(), ForceRangeMin, 1e-6);
		EXPECT_NEAR(ConeGenPtr->LengthDist.max(), ForceRangeMax, 1e-6);
		EXPECT_NEAR(ConeGenPtr->AngleDist.max(), RandomHalfAngle, 1e-6);
	}
	auto RandomKGenPtr = dynamic_cast<GeneratorType*>(Generator);
	if ( RandomKGenPtr )
	{
		EXPECT_NEAR(RandomKGenPtr->LengthDist.min(), ForceRangeMin, 1e-6);
		EXPECT_NEAR(RandomKGenPtr->LengthDist.max(), ForceRangeMax, 1e-6);
	}
}

TEST_F(FingerForceGeneratorTest, GenerateRandomVectorTest)
{
	size_t SucceedCnt = Generator->GenerateFingerForceList();
	InPointSetDataPtr->ForcePair = Generator->GetGeneratedFingerForceList()[0];
	std::cout << "SucceedCnt: " << SucceedCnt << std::endl;
	EXPECT_GT(SucceedCnt, 0);

	for ( int i = 0; i < GeneratorType::FORCE_COUNT; i++ )
	{
		Types::Vec3 RandomVec = InPointSetDataPtr->ForcePair.row(i);
		Types::Vec3 NormalVec = InPointSetDataPtr->NormalPair.row(i);
		// Check if the length is within the specified range
		double Length = RandomVec.norm();
		EXPECT_GE(Length, ForceRangeMin);
		EXPECT_LE(Length, ForceRangeMax);

		// Check if the direction is within the specified cone
		double Angle = acos(RandomVec.normalized().dot(NormalVec.normalized()));
		EXPECT_LE(Angle, RandomHalfAngle);
		std::cout << "Angle: " << Angle << " RandomHalfAngle: " << RandomHalfAngle << std::endl;
	}
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

#endif /* D844E03F_138B_4DA6_9F45_E4236E72A781 */
