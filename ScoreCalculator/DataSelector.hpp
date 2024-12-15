#ifndef A4C7E1E5_F899_4914_A068_04DAF55229B1
#define A4C7E1E5_F899_4914_A068_04DAF55229B1

#include <cassert>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>
#include <random>
#include <boost/concept/assert.hpp>


template<typename DataListType, typename ConfigObjType>
class TDataSelector
{
public:
	TDataSelector(const DataListType& InDataList, const ConfigObjType& InConfigObj)
		: DataListRef(InDataList), ConfigObj(InConfigObj)
	{
	}

	enum class EMethod: int{
		Good,
		Mid,
		Bad,
		Random,

		Last
	};
	std::string ToString(const EMethod InMethod) const
	{
		switch ( InMethod )
		{
		case EMethod::Good: return "Good";
		case EMethod::Mid: return "Mid";
		case EMethod::Bad: return "Bad";
		case EMethod::Random: return "Random";
		default: assert(false); return "";
		}
	}


	/**
	* @brief Get selected data based on the method name (for outside use).
	*
	* This function filters data from a list based on the provided method name and the specified data count.
	*
	* @param InMethodName The input method name used to filter the data.
	* @param InDataCount The input data count used to limit the number of returned data.
	* @return DataListType A list containing the selected data.
	*/
	DataListType GetSelectedDataByMethodName(const EMethod InMethodName, const int InDataCount) const
	{
		DataListType DataList;
		DoSelectingByMethodName(InMethodName, InDataCount, DataList);
		return DataList;
	}

protected:
	/**
	* @brief (Use inside class) Perform data selection based on the method name.
	*
	* This function selects data from a list based on the provided method name and the specified data count.
	* You can customize this function to implement different selection methods by overriding it in derived classes.
	*
	* @param InMethodName The input method name used to determine the selection method.
	* @param InDataCount The input data count used to limit the number of selected data.
	* @param OutDataList A reference to the output data list where the selected data will be stored.
	*/
	virtual void DoSelectingByMethodName(const EMethod InMethodName, const int InDataCount, DataListType& OutDataList) const
	{
		switch ( InMethodName )
		{
		case EMethod::Good:
			OutDataList = { this->DataListRef.begin(), this->DataListRef.begin() + InDataCount };
			break;
		case EMethod::Mid:
			OutDataList = { 
				this->DataListRef.begin() + (this->DataListRef.size() / 2) + (-(InDataCount) / 2), 
				this->DataListRef.begin() + (this->DataListRef.size() / 2) + (InDataCount / 2) + 1
			};
			break;
		case EMethod::Bad:
			OutDataList = { this->DataListRef.end() - InDataCount, this->DataListRef.end() };
			break;
		case EMethod::Random:
			for (const int Idx : GenerateRandomIntList(0, this->DataListRef.size () - 1, InDataCount))
			{
				OutDataList.insert(OutDataList.begin(), *(this->DataListRef.begin() + Idx));
			}
			break;
		}

	}

	std::set<int> GenerateRandomIntList(const int InRangeLow, const int InRangeUpper, const int InDataCount) const
	{
		assert(InDataCount <= InRangeUpper - InRangeLow + 1);
		std::random_device r;
		std::random_device::result_type CurrentSeed = r ();
		std::default_random_engine e1 ((std::random_device())());
		std::uniform_int_distribution<int> uniform_dist (InRangeLow, InRangeUpper - 1);
		std::set<int> Result;

		while (Result.size() < InDataCount) {
			const int Idx = uniform_dist (e1);
			Result.insert(Result.begin(), Idx);
		}

		return Result;
	}

protected:
	const DataListType& DataListRef;
	const ConfigObjType& ConfigObj;

private:
};

#endif /* A4C7E1E5_F899_4914_A068_04DAF55229B1 */
