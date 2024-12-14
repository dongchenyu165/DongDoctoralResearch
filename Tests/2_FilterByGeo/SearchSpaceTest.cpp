#include "2_FilterByGeo/SearchSpaceGenerator.hpp"
#include "GlobalTypes.hpp"
#include "GlobalVars.hpp"
#include "ScoreCalculator/ScoreCalculator2D.hpp"
#include "nlohmann/detail/macro_scope.hpp"
#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>
#include <Utilities/spdlog/LogConfig.hpp>
#include <gtest/gtest.h>
#include <cstddef>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// #include "ScoreCalculator/ScoreCalculatorBase.hpp"

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
		mock_pc_ = PCL_Helper::PCPTR<CalcPoint>(new PCL_Helper::PC<CalcPoint>);
		GenTestingPointCloud();
		generator_ = std::make_unique<PCL_Helper::SearchSpaceGenerator>(mock_pc_);
	}

	void GenTestingPointCloud()
	{
		for ( size_t i = 0; i < 5; ++i )
		{
			CalcPoint point;
			point.x        = static_cast<float>(i);
			point.y        = static_cast<float>(i);
			point.z        = static_cast<float>(i);
			point.normal_x = -static_cast<float>(i);
			point.normal_y = -static_cast<float>(i);
			point.normal_z = -static_cast<float>(i);
			mock_pc_->push_back(point);
		}
	}

	NEW_CALC_PC_PTR(mock_pc_);
	std::unique_ptr<PCL_Helper::SearchSpaceGenerator> generator_;
};

TEST_F(SearchSpaceGeneratorTest, GenerateSearchSpace_ValidCombinationCount)
{
	std::vector<Types::CalcPointSetData> search_space;
	size_t result_count = generator_->Generate(search_space);

	EXPECT_EQ(result_count, 10);        // Combinations C(5, 2) == 10
	EXPECT_EQ(search_space.size(), 10); // Verify allocated space
}

TEST_F(SearchSpaceGeneratorTest, GenerateSearchSpace_ContentVerification)
{
	std::vector<Types::CalcPointSetData> search_space;
	generator_->Generate(search_space);
	// Eigen::Matrix<float, 2, 3> PositionPair;
	// Eigen::Matrix<float, 1, 3> PositionPair2;
	// CalcPoint point1;
	// PositionPair2.row(0) == point1.getVector3fMap();

	for ( const auto& set_data : search_space )
	{
		for ( int i = 0; i < Types::CalcPointSetData::FINGER_COUNT; ++i )
		{
			int point_idx          = set_data.PointIndexPair(i);
			const CalcPoint& point = (*mock_pc_)[point_idx];

			EXPECT_EQ(set_data.PositionPair.row(i).transpose(), point.getVector3fMap());
			EXPECT_EQ(set_data.NormalPair.row(i).transpose(), point.getNormalVector3fMap());
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
	SPDLog::LoggerType logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("FilterGeo", LogConfigJsonPath);

	EXPECT_EQ(CalcCombination(5, 0, logger), 1);  // C(5, 0) == 1
	EXPECT_EQ(CalcCombination(5, 5, logger), 1);  // C(5, 5) == 1
	EXPECT_EQ(CalcCombination(5, 3, logger), 10); // C(5, 3) == 10
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