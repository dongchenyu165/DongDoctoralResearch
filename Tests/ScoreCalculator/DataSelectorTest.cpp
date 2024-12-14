#include <gtest/gtest.h>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <ScoreCalculator/DataSelector.hpp>

// Define some mock data types
using DataListType = std::vector<int>;
using ConfigObjType = std::set<std::string>;

// Mock class that inherits TDataSelector for testing protected methods
class TDataSelectorTest : public TDataSelector<DataListType, ConfigObjType>
{
public:
    TDataSelectorTest(const DataListType& InDataList, const ConfigObjType& InConfigObj)
        : TDataSelector<DataListType, ConfigObjType>(InDataList, InConfigObj)
    {    }

    using TDataSelector<DataListType, ConfigObjType>::DoSelectingByMethodName;
    using TDataSelector<DataListType, ConfigObjType>::GenerateRandomIntList;
};

// Test Fixture
class TDataSelectorTestFixture : public ::testing::Test
{
protected:
    DataListType mockDataList = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    ConfigObjType mockConfig = {"Data1", "Data2"};
    TDataSelectorTest* dataSelector;

    void SetUp() override
    {
        dataSelector = new TDataSelectorTest(mockDataList, mockConfig);
    }

    void TearDown() override
    {
        delete dataSelector;
    }
};

// Test for DoSelectingByMethodName with Good method
TEST_F(TDataSelectorTestFixture, DoSelectingByMethodName_Good)
{
    DataListType outDataList;
    dataSelector->DoSelectingByMethodName(TDataSelectorTest::EMethod::Good, 3, outDataList);
    EXPECT_EQ(outDataList.size(), 3);
    EXPECT_EQ(*(outDataList.begin() + 0), 1);  // Check first value
    EXPECT_EQ(*(outDataList.begin() + 1), 2);  // Check first value
    EXPECT_EQ(*(outDataList.begin() + 2), 3);  // Check first value
}

// Test for DoSelectingByMethodName with Mid method
TEST_F(TDataSelectorTestFixture, DoSelectingByMethodName_Mid)
{
    DataListType outDataList;
    dataSelector->DoSelectingByMethodName(TDataSelectorTest::EMethod::Mid, 3, outDataList);
    EXPECT_EQ(outDataList.size(), 3);
    EXPECT_EQ(*(outDataList.begin() + 0), 5);  // Check first value
    EXPECT_EQ(*(outDataList.begin() + 1), 6);  // Check first value
    EXPECT_EQ(*(outDataList.begin() + 2), 7);  // Check first value
}

// Test for DoSelectingByMethodName with Bad method
TEST_F(TDataSelectorTestFixture, DoSelectingByMethodName_Bad)
{
    DataListType outDataList;
    dataSelector->DoSelectingByMethodName(TDataSelectorTest::EMethod::Bad, 3, outDataList);
    EXPECT_EQ(outDataList.size(), 3);
    EXPECT_EQ(*(outDataList.begin() + 0), 8);  // Check first value
    EXPECT_EQ(*(outDataList.begin() + 1), 9);  // Check first value
    EXPECT_EQ(*(outDataList.begin() + 2), 10);  // Check first value
}

// Test for GenerateRandomIntList method
TEST_F(TDataSelectorTestFixture, GenerateRandomIntList)
{
    std::set<int> randomIndices = dataSelector->GenerateRandomIntList(0, 9, 5);
    EXPECT_EQ(randomIndices.size(), 5);  // Ensure correct number of indices
    for (const auto& idx : randomIndices) {
        EXPECT_GE(idx, 0);
        EXPECT_LE(idx, 9);
    }
}

// Test for DoSelectingByMethodName with Random method
TEST_F(TDataSelectorTestFixture, DoSelectingByMethodName_Random)
{
    DataListType outDataList;
    dataSelector->DoSelectingByMethodName(TDataSelectorTest::EMethod::Random, 3, outDataList);
    EXPECT_EQ(outDataList.size(), 3);
}

// Main entry point for tests
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}