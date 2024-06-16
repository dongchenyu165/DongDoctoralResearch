#ifndef D1E50037_DB3D_42E4_A503_4B51417DDF9C
#define D1E50037_DB3D_42E4_A503_4B51417DDF9C

#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include <functional>

template<typename PointType>
void CustomPointRemover(PCL_Helper::PCPTR<PointType> InPC, PCL_Helper::PCPTR<PointType> OutRemovedPC, std::function<bool(const PointType&)> const& bShouldBeRemoved)
{
	for (int i = 0; i < InPC->size(); i++)
	{
		auto& PointInPC = InPC->points[i];

		if (bShouldBeRemoved(PointInPC))
		{
			continue;
		}
		OutRemovedPC->push_back(PointType(PointInPC));
	}
}

#endif /* D1E50037_DB3D_42E4_A503_4B51417DDF9C */
