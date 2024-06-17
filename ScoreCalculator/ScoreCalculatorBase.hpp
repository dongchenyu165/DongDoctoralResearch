#ifndef BA74CF56_6B15_4A96_9B54_92D61F2CCA22
#define BA74CF56_6B15_4A96_9B54_92D61F2CCA22

#include <vector>
#include <map>

#include <Eigen/Core>

#include <Utilities/JSON_Helper/StructSerializer.hpp>


/**
 * @brief Basic class for calculating score.
 Input [InputDataType] datas by [AddCalculatingData] function.
	The raw score of added data'll calculated simutaneously.

 After all datas added, call [CalcFinalScore] function to calculate the final score,
	and also sort datas by the final score in decent.
 * 
 * @tparam InputDataType Input data's type to calculate the raw score. like [position normal ...] of a point-set
 * @tparam ConfigDataType Config object type of this object, mainly contains [Weight Vector] or other config parameter for calculating.
 * @tparam ScoreComponentCount How many score component to calculate the final score of the input data. Exp: 
 Final Score = ScoreComp_1 * Weight_1 + ScoreComp_2 * Weight_2 + ... + ScoreComp_n * Weight_n  (n = [ScoreComponentCount])
 */
template<typename InputDataType, typename ConfigObjType, int ScoreComponentCount>
class TScoreCalculatorBase
{
	using Self = TScoreCalculatorBase<InputDataType, ConfigObjType, ScoreComponentCount>;

public:
	using DataScorePairType     = std::pair<double /* Score */, InputDataType /* Input Data Obj*/>;
	using DataScorePairListType = std::vector<DataScorePairType>;

	// A matrix type to storage the score data with size (Pre-Allocated DataSize, ComponentSize)
	// 		[Pre-Allocated DataSize]: Dynamic size. Set by the construct function's [InDataSize] argument.
	using ScoreMatType         = Eigen::Matrix<float, -1, ScoreComponentCount>;
	using ScoreMatRowType      = Eigen::Matrix<float, 1, ScoreComponentCount>;
	using ScoreWeightVectorType = Eigen::Matrix<float, ScoreComponentCount, 1>;

	using StrategiesType = std::map<std::string, DataScoreListType, std::less<std::string>>;

	TScoreCalculatorBase(const std::string& InJsonPath, const std::vector<std::string>& InKey)
	{
		JSON_Helper::LoadStructure_ByPath(InJsonPath, InKey, ConfigData);
	}

	TScoreCalculatorBase(const std::string& InJsonPath) { JSON_Helper::LoadStructure_ByPath(InJsonPath, {}, ConfigData); }

	TScoreCalculatorBase() { }

protected:
	/**
	 * @brief Calculate a raw score of each score-component by using the input [InData].
	 MUST be implementated in child class.
	 MEANS: YOU MUST implementate:
	 1. Calculate each score component:
	 	ScoreComp_1 = CalcScoreComp_1(InData.data1);
	 	ScoreComp_2 = CalcScoreComp_2(InData.data1, InData.data2, InData.data4);
		...
	 	ScoreComp_n = CalcScoreComp_n(InData.data3, InData.data4);
	 2. Fill the matrix row the row index [CurrentDataIdx] with each score-component:
	 	ScoreRawData[CurrentDataIdx] = [ScoreComp_1, ScoreComp_2, ... , ScoreComp_n]
	 * 
	 * @param InData The data for calculating score.
	 * @return double Score.
	 */
	virtual double CalcRawScore(const InputDataType& InData) = 0;


	void SortByScore() { std::stable_sort(DataScoreList.begin(), DataScoreList.end(), &Self::SortPredicate); }
	static bool SortPredicate(const DataScorePairType& InA, const DataScorePairType& InB) { return InA.first > InB.first; }

public:
/**
 * @brief Mainly used function. 
 * 			Calculate score of the input [InputDataType] data, 
 * 			and add it to list for further score-based-sorting.
 * 
 * @param InData [InputDataType] usually be [] type, for score calculating.
 */
	void AddCalculatingData(const InputDataType& InData)
	{
		DataScoreList.push_back({CalcRawScore(InData), InData});
		CurrentDataIdx++;
	}

	size_t Size() { return DataScoreList.size(); }

protected:
	DataScorePairListType DataScoreList;
	ConfigObjType ConfigData;

	// A matrix to storage the score data with size (Pre-Allocated DataSize, ComponentSize).
	// 		[Pre-Allocated DataSize] set by the construct function's [InDataSize] argument.
	// 			will shrink to the real size of datas at the start of [CalcFinalScore] function. 
	ScoreMatType ScoreRawData;
	ScoreWeightVectorType ScoreWeight;

	// Storage how many datas added to this calculator.
	/// Changed in [AddCalculatingData] function/
	size_t CurrentDataIdx = 0;
};

#endif /* BA74CF56_6B15_4A96_9B54_92D61F2CCA22 */
