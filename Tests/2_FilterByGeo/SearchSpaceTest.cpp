#include "2_FilterByGeo/SearchSpaceGenerator.hpp"
#include "GlobalTypes.hpp"
#include "GlobalVars.hpp"
#include <Utilities/spdlog/LogConfig.hpp>
#include <cstddef>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("FilterGeo", LogConfigJsonPath);

	// Load point cloud.
	NEW_CALC_PC_PTR(SourcePC);
	pcl::io::loadPCDFile("./GraspingPC.pcd", *SourcePC);
	Logger->info("Load point cloud with size: [{}]", SourcePC->size());

	PCL_Helper::SearchSpaceGenerator SearchSpaceGenObj(SourcePC);
	Types::SearchSpace ResultSearchSpace;
	size_t RealCombCount = SearchSpaceGenObj.Generate(ResultSearchSpace);

	Logger->info("Generate [{}] size of Search Space. Pre-Allocated size: [{}]", RealCombCount, ResultSearchSpace.size());
	Logger->info("First element: [{}]", ResultSearchSpace.front().ToString());

	return 0;
}