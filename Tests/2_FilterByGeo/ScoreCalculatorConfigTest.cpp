#include <iostream>

#include <gtest/gtest.h>
#include "ScoreCalculator/ScoreCalculatorConfig.hpp"


BEGIN_DEF_SCORE_CALCULATOR_CONFIG(Test2)
	MEMBER_DEF_WITH_DEFAULT(float, NewVar, 2.3847);
	MEMBER_DEF_WITH_DEFAULT(float, NewVar2, 2.3847);
	MEMBER_DEF_WITH_DEFAULT(float, NewVar3, 2.3847);

	SCORE_CALC_MEMBER_DEF(NewVar, NewVar2, NewVar3)
END_DEF_SCORE_CALCULATOR_CONFIG()

	void TEST_JsonLoad()
	{
		// Literal json string, like Python ["""]
		std::string json_str = R"(
{
	"Weight_PointDistance" : 1.5885,
	"Weight_PointNormalAngle" : 4.4,
	"Weight_SameAsKnifeDir" : 2,
	"Weight_SameAsKnifeDir_OLD" : 4,
	"Weight_GraspHeight": 6,
	"NewVar" : 0.04,
	"NewVar2" : 6,
	"NewVar3" : 6
}
)";
		using TestType = TScoreCalculatorConfig_Test2<4>;
		TestType::SelfJsonType j2      = TestType::SelfJsonType::parse(json_str);

		TestType Obj = j2.get<TestType>();
		std::cout << Obj.WeightVector.transpose() << '\n';
		std::cout << Obj.NewVar << '\n';
		std::cout << Obj.NewVar2 << '\n';
		std::cout << Obj.NewVar3 << '\n';
		TestType Obj2 = j2.get<TestType>();
}


// Define the derived configuration class outside of TEST_F
BEGIN_DEF_SCORE_CALCULATOR_CONFIG(Test)
	MEMBER_DEF_WITH_DEFAULT(float, NewVar, 0.04);
	MEMBER_DEF_WITH_DEFAULT(float, NewVar2, 6.0);
	MEMBER_DEF_WITH_DEFAULT(float, NewVar3, 6.0);
	SCORE_CALC_MEMBER_DEF(NewVar, NewVar2, NewVar3);
END_DEF_SCORE_CALCULATOR_CONFIG()


// Unit test fixture class
class ScoreCalculatorConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Optional setup before each test
    }
};

// Test JSON deserialization into WeightVector
TEST_F(ScoreCalculatorConfigTest, WeightVectorInitialization) {
    constexpr int ScoreComponentCount = 5;
    TScoreCalculatorConfig<ScoreComponentCount> ConfigObj;

    // JSON input reflecting real field names
    nlohmann::ordered_json JsonObj = {
        {"Weight_PointDistance", 1.5885},
        {"Weight_PointNormalAngle", 4.4},
        {"Weight_SameAsKnifeDir", 2.0},
        {"Weight_SameAsKnifeDir_OLD", 4.0},
        {"Weight_GraspHeight", 6.0}
    };

	std::cout << JsonObj.size() << '\n';

    // Deserialize JSON into config
    from_json(JsonObj, ConfigObj);

    // Verify WeightVector values
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(0), 1.5885);
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(1), 4.4);
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(2), 2.0);
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(3), 4.0);
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(4), 6.0);
}

// Test handling of extra fields in JSON
TEST_F(ScoreCalculatorConfigTest, ExtraFieldsIgnored) {
    constexpr int ScoreComponentCount = 3;
    TScoreCalculatorConfig<ScoreComponentCount> ConfigObj;

    // JSON input with additional fields not used in WeightVector
    nlohmann::ordered_json JsonObj = {
        {"Weight_PointDistance", 1.5885},
        {"Weight_PointNormalAngle", 4.4},
        {"Weight_SameAsKnifeDir", 2.0},
        {"Weight_SameAsKnifeDir_OLD", 4.0},
        {"Weight_GraspHeight", 6.0},
        {"ExtraField", "ignore_this"}
    };

    // Deserialize JSON into config
    from_json(JsonObj, ConfigObj);

    // Verify that only the first three weights are set
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(0), 1.5885);
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(1), 4.4);
    EXPECT_FLOAT_EQ(ConfigObj.WeightVector(2), 2.0);
}

// Test default values and additional derived class fields
TEST_F(ScoreCalculatorConfigTest, DerivedClassWithExtraFields) {
    TScoreCalculatorConfig_Test<5> DerivedConfigObj;

    // JSON input for derived config with extra parameters
    nlohmann::ordered_json JsonObj = {
        {"Weight_PointDistance", 1.5885},
        {"Weight_PointNormalAngle", 4.4},
        {"Weight_SameAsKnifeDir", 2.0},
        {"Weight_SameAsKnifeDir_OLD", 4.0},
        {"Weight_GraspHeight", 6.0},
        {"NewVar", 0.04},
        {"NewVar2", 6.0},
        {"NewVar3", 6.0}
    };

    from_json(JsonObj, DerivedConfigObj);

    // Verify WeightVector values and derived class defaults
    EXPECT_FLOAT_EQ(DerivedConfigObj.WeightVector(0), 1.5885);
    EXPECT_FLOAT_EQ(DerivedConfigObj.WeightVector(1), 4.4);
    EXPECT_FLOAT_EQ(DerivedConfigObj.WeightVector(2), 2.0);
    EXPECT_FLOAT_EQ(DerivedConfigObj.WeightVector(3), 4.0);
    EXPECT_FLOAT_EQ(DerivedConfigObj.WeightVector(4), 6.0);
    EXPECT_FLOAT_EQ(DerivedConfigObj.NewVar, 0.04);
    EXPECT_FLOAT_EQ(DerivedConfigObj.NewVar2, 6.0);
    EXPECT_FLOAT_EQ(DerivedConfigObj.NewVar3, 6.0);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // Custom setup code can be added here if needed
	TEST_JsonLoad();

    return RUN_ALL_TESTS();
}