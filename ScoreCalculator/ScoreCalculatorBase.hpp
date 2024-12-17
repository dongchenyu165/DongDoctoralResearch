#ifndef BA74CF56_6B15_4A96_9B54_92D61F2CCA22
#define BA74CF56_6B15_4A96_9B54_92D61F2CCA22

#include <vector>
#include <map>

#include <Eigen/Core>

#include "DataTypes/PointSetData.hpp"
#include "DataSelector.hpp"
#include "Utilities/spdlog/LogConfig.hpp"
#include "spdlog/common.h"
#include "Utilities/spdlog/FunctionAutoLogger.hpp"
#include <Utilities/JSON_Helper/StructSerializer.hpp>

/**
 * @brief Basic class for calculating score.
 Input [InputDataType] datas by [AddCalculatingData] function.
    The raw score of added data'll calculated simutaneously.

 After all datas added, call [CalcFinalScore] function to calculate the final score,
    and also sort datas by the final score in descending order.
 *
 * @tparam InputDataType Input data's type to calculate the raw score. like [position normal ...] of a point-set
 * @tparam ConfigDataType Config object type of this object, mainly contains [Weight Vector] or other config parameter
 for calculating.
 * @tparam ScoreComponentCount How many score component to calculate the final score of the input data. Exp:
 Final Score = ScoreComp_1 * Weight_1 + ScoreComp_2 * Weight_2 + ... + ScoreComp_n * Weight_n  (n =
 [ScoreComponentCount])
 */
template<typename InputDataType, typename ConfigObjType>
class TScoreCalculatorBase
{
public:
	static constexpr int ScoreComponentCount = ConfigObjType::ScoreComponentCountVar;

protected:
	using Self = TScoreCalculatorBase<InputDataType, ConfigObjType>;

	using DataScorePairType     = std::pair<double /* Score */, InputDataType /* Input Data Obj*/>;
	using DataScorePairListType = std::vector<DataScorePairType>;
	using ReturnDataListType    = std::vector<InputDataType>;

	// A matrix type to storage the score data with size (Pre-Allocated DataSize, ComponentSize)
	// 		[Pre-Allocated DataSize]: Dynamic size. Set by the construct function's [InDataSize] argument.
	using ScoreMatType          = Eigen::Matrix<Types::CalcScalar, -1, ScoreComponentCount>;
	using RawScoreVecType       = Eigen::Matrix<Types::CalcScalar, -1, 1>;
	using ScoreWeightVectorType = Eigen::Matrix<Types::CalcScalar, ScoreComponentCount, 1>;

public:
	using ReturnDataSelectorType = TDataSelector<ReturnDataListType, ConfigObjType>;

public:
	TScoreCalculatorBase(const ConfigObjType& InConfigJsonObj, const size_t InDataSize, EvaluationStaticData& InStaticData, SPDLog::LoggerType InLogger = nullptr)
		: StaticData(InStaticData), Logger(InLogger)
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);  // Create a logger if [Logger] is [nullptr].
		// Load [ConfigData] from json object.
		ConfigData = InConfigJsonObj;
		ScoreWeight = ConfigData.WeightVector;

		if (InLogger && InLogger->should_log(spdlog::level::info))
		{
			LOG_INDENT(Logger, info, "Initial score calculator.");
			LOG_INDENT(Logger, info, "Score Weight Vector: [{}]", ScoreWeight);
		}
		FUNC_LOGGER_RET;
	}

protected:
	/**
	 * @brief Calculate a raw score of each score-component by using the input [InData] and the [StaticData].
	 MUST be implementated in child class.
	 MEANS: YOU MUST implementate:
	 1. Calculate each score component:
	    ScoreComp_1 = CalcScoreComp_1(InData.data1, StaticData.data2);
	    ScoreComp_2 = CalcScoreComp_2(InData.data1, InData.data2, InData.data4);
	    ...
	    ScoreComp_n = CalcScoreComp_n(InData.data3, InData.data4, StaticData.data3);
	 2. Fill the matrix row the row index [CurrentDataIdx] with each score-component:
	    ScoreRawData[CurrentDataIdx] = [ScoreComp_1, ScoreComp_2, ... , ScoreComp_n]
	 *
	 * @param InData The data for calculating score.
	 */
	virtual void CalcRawScore(const InputDataType& InData, int InCurrentDataIdx) = 0;

	/**
	 * @brief Sort the [DataScoreList] by their score in descending order.
	 *
	 */
	void SortByScore() { std::stable_sort(DataScoreList.begin(), DataScoreList.end(), &Self::SortPredicate); }

	/**
	 * @brief A function to decide data A is LARGER than B.
	 *
	 * @param InA data A
	 * @param InB data B
	 * @return true A is LARGER than B.
	 * @return false A is SMALLER than B.
	 */
	static bool SortPredicate(const DataScorePairType& InA, const DataScorePairType& InB)
	{
		return InA.first > InB.first;
	}

	/**
	 * @brief Mainly used function.
	            Normalize each score component at their own.
	            Also apply the weight of each component.
	            Should called after all calculating datas was added.
	 */
	virtual void CalcFinalScore()
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);  // Create a logger if [Logger] is [nullptr].
		// Resize the score matrix to the fit the real size of datas.

		LOG_INDENT(Logger, debug, "Normalize score column.");
		// Normalize each score component.
		for ( int i = 0; i < ScoreComponentCount; i++ )
		{
			float min                         = this->ScoreRawData.col(i).minCoeff();
			float max                         = this->ScoreRawData.col(i).maxCoeff();
			this->ScoreRawData.col(i).array() += min;
			this->ScoreRawData.col(i) / (max - min);
		}

		// Apply the weight to each component for all datas.
		// NOTE: return value type can not be [auto] here, because the very low access speed of the Eigen::Matrix.
		LOG_INDENT(Logger, debug, "Apply weight.");
		/* (DataCount, 1) */ RawScoreVecType ScoreVector = this->ScoreRawData * this->ScoreWeight;
		LOG_INDENT(Logger, trace, "Copy score of size: [{}] .", DataScoreList.size());
		for ( int i = 0; i < this->DataScoreList.size(); i++ )
		{
			// Storage the final score to each element's first member var through the [DataScoreList].
			this->DataScoreList[i].first = ScoreVector(i);
		}

		LOG_INDENT(Logger, debug, "Sort by score.");
		// Sort the datas by the final score.
		this->SortByScore();

		// Finally fill the [ReturnDataList] with the sorted datas.
		ReturnDataList.reserve(this->DataScoreList.size());

		LOG_INDENT(Logger, debug, "Push back to [ReturnDataList].");
		for ( const auto& DataScorePair : this->DataScoreList )
		{
			ReturnDataList.push_back(DataScorePair.second);
		}
	}

public:
	void CalculateScore(const std::vector<InputDataType>& InDataList)
	{
		FUNC_LOGGER_ENTER_CUSTOM_LOGGER(Logger);  // Create a logger if [Logger] is [nullptr].

		LOG_INDENT(Logger, info, "Start to calculate score with input data size [{}].", InDataList.size());
		LOG_INDENT(Logger, debug, "Start to resize [DataScoreList] and [ScoreRawData]");

		DataScoreList.resize(InDataList.size(), { -INFINITY, InputDataType() });
		ScoreRawData.resize(DataScoreList.size(), ScoreComponentCount);
		ScoreRawData.setConstant(-INFINITY); // Init to min of the float.

		LOG_INDENT(Logger, info, "Start to calculate the raw score.");
		LOG_INDENT_CHECK_SHOULD_LOG(Logger, debug, "[ScoreRawData] front 10 elements (SHOULD ALL [-INFINITY]): \n{}", ScoreRawData.block(0, 0, 10, ScoreComponentCount));
		for ( int i = 0; i < InDataList.size(); i++ )
		{
			CalcRawScore(InDataList[i], i);

			// Initial final score to 0.
			// DataScoreList[i].first = 0;
			DataScoreList[i].second = InDataList[i];
		}

		LOG_INDENT_CHECK_SHOULD_LOG(Logger, debug, "[ScoreRawData] front 10 elements: \n{}", ScoreRawData.block(0, 0, 10, ScoreComponentCount));
		LOG_INDENT_CHECK_SHOULD_LOG(Logger, debug, "[DataScoreList] front 10 elements' score (SHOULD ALL [-INFINITY]):  {:f10}", DataScoreList);

		// Calculate the final (normalize, weighted and sorted) score.
		LOG_INDENT(Logger, info, "Start to calculate the final (normalize, weighted and sorted) score.");
		CalcFinalScore();

		LOG_INDENT(Logger, debug, "[ScoreRawData] front 10 elements: \n{:+07.4}", ScoreRawData.block(0, 0, 10, ScoreComponentCount));
		LOG_INDENT_CHECK_SHOULD_LOG(Logger, debug, "[DataScoreList] front 10 elements' score (SHOULD SORTED DECENT ORDER): {:f10}", DataScoreList);
		FUNC_LOGGER_RET;
	}

	/**
	 * @brief Get the final data list by the selecting method name and the size of data.
	 after sorting by score.
	 *
	 * @param InMethodName The input method name used to filter the data.
	 * @param InDataCount The input data count used to limit the number of returned data.
	 * @return ReturnDataListType A list containing the selected data.
	 */
	virtual ReturnDataListType GetFinalDataList(typename ReturnDataSelectorType::EMethod InGettingMethodName, int InGettingDataSize) const
	{
		ReturnDataSelectorType DataSelector(ReturnDataList, ConfigData);
		return DataSelector.GetSelectedDataByMethodName(InGettingMethodName, InGettingDataSize);
	}

	size_t Size() { return DataScoreList.size(); }

protected:
	DataScorePairListType DataScoreList;
	ReturnDataListType ReturnDataList;  // The final data list after sorting by score.
	ConfigObjType ConfigData;
	EvaluationStaticData StaticData;

	// A matrix to storage the score data with size (Pre-Allocated DataSize, ComponentSize).
	// 		[Pre-Allocated DataSize] set by the construct function's [InDataSize] argument.
	// 			will shrink to the real size of datas at the start of [CalcFinalScore] function.
	ScoreMatType ScoreRawData;
	ScoreWeightVectorType ScoreWeight;

	// Storage how many datas added to this calculator.
	/// Changed in [AddCalculatingData] function/
	// size_t CurrentDataIdx = 0;

	SPDLog::LoggerType Logger;
};

#endif /* BA74CF56_6B15_4A96_9B54_92D61F2CCA22 */
