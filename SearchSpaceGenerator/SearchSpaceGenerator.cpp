#include "SearchSpaceGenerator.hpp"
#include "GlobalTypes.hpp"
#include "GlobalVars.hpp"
#include "Utilities/spdlog/LogConfig.hpp"

#include <cppitertools/combinations.hpp>
#include <spdlog/common.h>


namespace PCL_Helper
{
SearchSpaceGenerator::SearchSpaceGenerator(Types::CalcPCPTR InPC) : OperatingPC(InPC) { 
	Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("FilterGeo", LogConfigJsonPath);
}

/**
 * @brief Calculate the combinations
 *
 * @param InElementCount The full element count
 * @param InSelectCount The size of chose element.
 * @return size_t Result
 */
size_t CalcCombination(const size_t InElementCount, const size_t InSelectCount, SPDLog::LoggerType& InLogger)
{
	if ( InSelectCount >= InElementCount )
	{
		return 1;
	}

	size_t Upper = InElementCount;
	size_t Lower = 1;
	for ( size_t i = 2; i <= InSelectCount; i++ )
	{
		Lower *= i;
		Upper *= InElementCount - (i - 1);
		// SPDLog::
		if (InLogger->should_log(spdlog::level::trace))
		{
			SPDLog::Log_T(InLogger, 1, "LowerNew: [{}]; UpperNew: [{}] ==== Lower:[{}] / Upper:[{}]", i, InElementCount - (i - 1), Upper, Lower);
		}
	}

	return Upper / Lower;
}

size_t SearchSpaceGenerator::Generate(std::vector<Types::CalcPointSetData>& OutSearchSpace)
{
	using namespace Types;
	LOG_FUNC_ENTER(Logger, debug, 0);

	// Initial [PointIndexList] to contains index of each point.
	// {0, 1, 2, 3, 4, ... , PC.size()}
	std::vector<int> PointIndexList;
	PointIndexList.assign(OperatingPC->size(), -1);
	for ( int i = 0; i < PointIndexList.size(); i++ )
	{
		PointIndexList[i] = i;
	}
	SPDLog::Log_D(Logger, 0, "Pre allocate [PointIndexList] with size: [{}].", OperatingPC->size());

	// Initial the output [OutSearchSpace] std::vector.
	const size_t SearchSpaceCount = CalcCombination(PointIndexList.size(), CalcPointSetData::FINGER_COUNT, Logger);
	OutSearchSpace.assign(SearchSpaceCount, CalcPointSetData());
	SPDLog::Log_D(Logger, 0, "Pre allocate [OutSearchSpace] with size: [{}].", SearchSpaceCount);

	size_t CombIdx = 0;
	// PointIndexSetIt is a vector<int> contains a [points] combination of index of point.
	/// If choose 2, [iter::combinations] is an iterator of {{0, 1}, {0, 2}, {0, 3}, ... , {n - 1, n}}.
	/// 			[PointIndexSetIt] is the iterator of {0, 1}
	for ( auto&& PointIndexSetIt : iter::combinations(PointIndexList, CalcPointSetData::FINGER_COUNT) )
	{
		CalcPointSetData& OperatingData = OutSearchSpace[CombIdx];

		size_t DataIdx = 0;
		for ( size_t PointIdx : PointIndexSetIt )
		{
			// Storage point position of each point in this point set.
			OperatingData.PositionPair.row(DataIdx) = (*OperatingPC)[PointIdx].getVector3fMap().cast<Types::CalcScalar>();
			// Storage point normal of each point in this point set.
			OperatingData.NormalPair.row(DataIdx) = (*OperatingPC)[PointIdx].getNormalVector3fMap().cast<Types::CalcScalar>();
			// Storage index of each point in this point set.
			OperatingData.PointIndexPair(DataIdx) = PointIdx;
			DataIdx++;
		}

		CombIdx++;
	}
	SPDLog::Log_I(Logger, 0, "Finish Search Space Generation");

	LOG_FUNC_EXIT(Logger, debug, 0);
	return CombIdx;
}
} // namespace PCL_Helper