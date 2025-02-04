#ifndef C7D34643_55AE_4574_B043_5E580F558864
#define C7D34643_55AE_4574_B043_5E580F558864

#include <spdlog/spdlog.h>
#include <cstddef>
#include <memory>


namespace Utilities
{

// Custom hash function for CalcPointSetDataPtr. Use the index as the hash value.
template<typename CalcPointSetDataPtrType, int FingerCount>
struct TCalcPointSetDataPtrHash
{
	static constexpr int POINT_COUNT = 1000;
	inline static std::shared_ptr<spdlog::logger> Logger;

	std::size_t operator()(const CalcPointSetDataPtrType& key) const
	{
		std::size_t Hash = 0;
		for ( int i = key->PointIndexPair.size() - 1; i >= 0; --i )
		{
			Hash *= POINT_COUNT;
			Hash += key->PointIndexPair(i);
		}

		if ( Logger )
		{
			Logger->debug("Index list: {}, Hash value: {}", key->PointIndexPair.transpose(), Hash);
		}
		return Hash;
	}
};

// Custom hash function for CalcPointSetDataPtr Use the [point position] as the hash value.
template<typename CalcPointSetDataPtrType, int FingerCount>
struct TCalcPointSetDataPtrPointPosHash
{
	static constexpr int POINT_COUNT = 1000;
	inline static std::shared_ptr<spdlog::logger> Logger;

	std::size_t operator()(const CalcPointSetDataPtrType& key) const
	{
		std::size_t Hash = 0;
		for ( int i = key->PointIndexPair.size() - 1; i >= 0; --i )
		{
			// Hash using the point position.
			Hash *= POINT_COUNT;
			Hash += key->PositionPair.row(i).sum() * 1000000;
		}

		if ( Logger )
		{
			Logger->debug("Index list: {}, Hash value: {}", key->PointIndexPair.transpose(), Hash);
		}
		return Hash;
	}
};
// Custom equality function for CalcPointSetDataPtr
template<typename CalcPointSetDataPtr>
struct TCalcPointSetDataPtrEqual
{
	bool operator()(const CalcPointSetDataPtr& lhs, const CalcPointSetDataPtr& rhs) const
	{
		for ( int i = 0; i < lhs->PointIndexPair.size(); i++ )
		{
			if (lhs->PointIndexPair(i) != rhs->PointIndexPair(i))
			{
				return false;
			}
		}
		return true;
	}
};
}


#endif /* C7D34643_55AE_4574_B043_5E580F558864 */
