#ifndef BA74CF56_6B15_4A96_9B54_92D61F2CCA22
#define BA74CF56_6B15_4A96_9B54_92D61F2CCA22

#include <vector>
#include <map>

#include <Eigen/Core>

#include "DataTypes/PointSetData.hpp"
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

	// A matrix type to storage the score data with size (Pre-Allocated DataSize, ComponentSize)
	// 		[Pre-Allocated DataSize]: Dynamic size. Set by the construct function's [InDataSize] argument.
	using ScoreMatType          = Eigen::Matrix<Types::CalcScalar, -1, ScoreComponentCount>;
	using ScoreWeightVectorType = Eigen::Matrix<Types::CalcScalar, ScoreComponentCount, 1>;

public:
	TScoreCalculatorBase(const json& InConfigJsonObj, const size_t InDataSize, EvaluationStaticData& InStaticData)
		: StaticData(InStaticData)
	{
		// Load [ConfigData] from json object.
		ConfigData = InConfigJsonObj.get<ConfigObjType>();
		ScoreWeight = ConfigData.WeightVector;

		// std::vector init;
		DataScoreList.resize(InDataSize, { INTMAX_MIN, InputDataType() });

		// Eigen matrix init.
		ScoreRawData.resize(InDataSize, ScoreComponentCount);
		ScoreRawData.setConstant(-INFINITY); // Init to min of the float.
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
	 * @return double Score.
	 */
	virtual double CalcRawScore(const InputDataType& InData) = 0;

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
		CalcRawScore(InData);

		// Initial final score to 0.
		DataScoreList.push_back({ 0, InData });
		CurrentDataIdx++;
	}

	/**
	 * @brief Mainly used function.
	            Normalize each score component at their own.
	            Also apply the weight of each component.
	            Should called after all calculating datas was added.
	 */
	virtual void CalcFinalScore()
	{
		// Resize the score matrix to the fit the real size of datas.
		this->ScoreRawData.resize(this->CurrentDataIdx, ScoreComponentCount);

		// Normalize each score component.
		for ( int i = 0; i < ScoreComponentCount; i++ )
		{
			float min                         = this->ScoreRawData.col(i).minCoeff();
			float max                         = this->ScoreRawData.col(i).maxCoeff();
			this->ScoreRawData.col(i).array() += min;
			this->ScoreRawData.col(i) / (max - min);
		}

		// Apply the weight to each component for all datas.
		/* (DataCount, 1) */ auto ScoreVector = this->ScoreRawData * this->ScoreWeight;
		for ( int i = 0; i < this->DataScoreList.size(); i++ )
		{
			// Storage the final score to each element's first member var through the [DataScoreList].
			this->DataScoreList[i].first = ScoreVector(i);
		}

		// Finally sort the datas by the final score.
		this->SortByScore();
	}

	size_t Size() { return DataScoreList.size(); }

protected:
	DataScorePairListType DataScoreList;
	ConfigObjType ConfigData;
	EvaluationStaticData StaticData;

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
