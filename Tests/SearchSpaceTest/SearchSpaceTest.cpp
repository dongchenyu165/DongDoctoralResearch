#include "GlobalBaseTypes.hpp"
#include "SearchSpaceGenerator/SearchSpaceGenerator.hpp"
#include "GlobalTypes.hpp"
#include "GlobalVars.hpp"
#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>
#include <Utilities/spdlog/LogConfig.hpp>
#include <gtest/gtest.h>
#include <cstddef>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


bool TEST_GenerateInitSearchSpace(const SPDLog::LoggerType& InLogger, Types::CalcPCPTR InPC)
{
	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(InPC);
	Types::SearchSpace ResultSearchSpace;
	size_t RealCombCount = SearchSpaceGenObj.Generate(ResultSearchSpace);

	return true;
}

// Assuming CalcPoint and CalcPCPTR are available from the PCL_Helper namespace in GlobalTypes.hpp.
using Types::CalcPCPTR;
using Types::CalcPoint;

class SearchSpaceGeneratorTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Initialize a mock point cloud with 5 points, where each point is of type CalcPoint.
		OperatingPC = PCL_Helper::PCPTR<CalcPoint>(new PCL_Helper::PC<CalcPoint>);
		GenTestingPointCloud();
		Generator = std::make_unique<PCL_Helper::SearchSpaceGenerator>(OperatingPC);
	}

	void GenTestingPointCloud()
	{
		for ( size_t i = 0; i < 5; ++i )
		{
			CalcPoint _Point;
			_Point.x        = static_cast<float>(i);
			_Point.y        = static_cast<float>(i);
			_Point.z        = static_cast<float>(i);
			_Point.normal_x = -static_cast<float>(i);
			_Point.normal_y = -static_cast<float>(i);
			_Point.normal_z = -static_cast<float>(i);
			OperatingPC->push_back(_Point);
		}
	}

	NEW_CALC_PC_PTR(OperatingPC);
	std::unique_ptr<PCL_Helper::SearchSpaceGenerator> Generator;
};

TEST_F(SearchSpaceGeneratorTest, GenerateSearchSpace_ValidCombinationCount)
{
	Types::SearchSpace SearchSpace;
	size_t ResultCount = Generator->Generate(SearchSpace);

	EXPECT_EQ(ResultCount, 10);        // Combinations C(5, 2) == 10
	EXPECT_EQ(SearchSpace.size(), 10); // Verify allocated space
}

TEST_F(SearchSpaceGeneratorTest, GenerateSearchSpace_ContentVerification)
{
	Types::SearchSpace SearchSpace;
	Generator->Generate(SearchSpace);
	// Eigen::Matrix<float, 2, 3> PositionPair;
	// Eigen::Matrix<float, 1, 3> PositionPair2;
	// CalcPoint point1;
	// PositionPair2.row(0) == point1.getVector3fMap();

	for ( const auto& set_data : SearchSpace )
	{
		for ( int i = 0; i < Types::CalcPointSetData::FINGER_COUNT; ++i )
		{
			int point_idx          = set_data->PointIndexPair(i);
			const CalcPoint& point = (*OperatingPC)[point_idx];

			EXPECT_EQ(set_data->PositionPair.row(i).transpose(), point.getVector3fMap().cast<Types::CalcScalar>());
			EXPECT_EQ(set_data->NormalPair.row(i).transpose(), point.getNormalVector3fMap().cast<Types::CalcScalar>());
		}
	}
}

// Define CalcCombination for use in testing
size_t CalcCombination(const size_t InElementCount, const size_t InSelectCount, SPDLog::LoggerType& InLogger)
{
	if ( InSelectCount == 0 || InSelectCount >= InElementCount )
	{
		return 1;
	}

	size_t Upper = InElementCount;
	size_t Lower = 1;
	for ( size_t i = 2; i <= InSelectCount; i++ )
	{
		Lower *= i;
		Upper *= InElementCount - (i - 1);
		if ( InLogger->should_log(spdlog::level::trace) )
		{
			SPDLog::Log_T(InLogger, 1, "LowerNew: [{}]; UpperNew: [{}] ==== Lower:[{}] / Upper:[{}]", i,
				InElementCount - (i - 1), Upper, Lower);
		}
	}

	return Upper / Lower;
}

TEST_F(SearchSpaceGeneratorTest, CalcCombination_HandlesEdgeCases)
{
	SPDLog::LoggerType Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("FilterGeo", LogConfigJsonPath);

	EXPECT_EQ(CalcCombination(5, 0, Logger), 1);  // C(5, 0) == 1
	EXPECT_EQ(CalcCombination(5, 5, Logger), 1);  // C(5, 5) == 1
	EXPECT_EQ(CalcCombination(5, 3, Logger), 10); // C(5, 3) == 10
}


int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	
	// Custom setup code can be added here if needed
	auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("FilterGeo", LogConfigJsonPath);

	// Load point cloud.
	NEW_CALC_PC_PTR(SourcePC);
	pcl::io::loadPCDFile("./GraspingPC.pcd", *SourcePC);
	Logger->info("Load point cloud with size: [{}]", SourcePC->size());

	// Do test of init search space generator.
	TEST_GenerateInitSearchSpace(Logger, SourcePC);

	return RUN_ALL_TESTS();
}