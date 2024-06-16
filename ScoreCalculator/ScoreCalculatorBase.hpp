#ifndef BA74CF56_6B15_4A96_9B54_92D61F2CCA22
#define BA74CF56_6B15_4A96_9B54_92D61F2CCA22

#include <vector>
#include <map>

#include <Eigen/Core>

#include <Utilities/JSON_Helper/StructSerializer.hpp>

template<typename DataType, typename ConfigDataType, int ScoreComponentCount>
class TScoreCalculatorBase
{
	using Self = TScoreCalculatorBase<DataType, ConfigDataType, ScoreComponentCount>;

public:
	using DataScoreType     = std::pair<double, DataType>;
	using DataScoreListType = std::vector<DataScoreType>;

	using ScoreDataRowType      = Eigen::Matrix<float, 1, ScoreComponentCount>;
	using ScoreDataType         = Eigen::Matrix<float, -1, ScoreComponentCount>;
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
	 * @brief Calculation score by using the input [InData].
	 MUST be implementated in child class.
	 * 
	 * @param InData The data for calculating score.
	 * @return double Score.
	 */
	virtual double CalcScore(const DataType& InData) = 0;


	void SortByScore() { std::stable_sort(DataScoreList.begin(), DataScoreList.end(), &Self::SortPredicate); }
	static bool SortPredicate(const DataScoreType& InA, const DataScoreType& InB) { return InA.first > InB.first; }

public:
	virtual void GetStrategies(StrategiesType& OutStrategies, bool bDebugMode = false) = 0;

	void SetDataSize(size_t InDataSize)
	{
		DataScoreList.reserve(InDataSize);
		ScoreRawData.resize(InDataSize, ScoreComponentCount);
		CurrentDataIdx = 0;
	}

	void AddData(const DataType& InData)
	{
		DataScoreList.push_back(std::make_pair(CalcScore(InData), InData));
		CurrentDataIdx++;
	}

	size_t Size() { return DataScoreList.size(); }

protected:
	DataScoreListType DataScoreList;
	ConfigDataType ConfigData;

	ScoreDataType ScoreRawData;
	ScoreWeightVectorType ScoreWeight;

	size_t CurrentDataIdx = 0;
};

#endif /* BA74CF56_6B15_4A96_9B54_92D61F2CCA22 */
