#ifndef DBC44A57_C1A3_4E5D_885D_540DB7355CFB
#define DBC44A57_C1A3_4E5D_885D_540DB7355CFB

#include "GlobalTypes.hpp"

namespace PCL_Helper::App
{

class PolygonFillerImpl;

template<typename PointType>
class TPolygonFiller
{
	void Test();

private:
	PolygonFillerImpl* _impl;
};

template class TPolygonFiller<Types::CalcPoint>;

} // namespace PCL_Helper::App
#endif /* DBC44A57_C1A3_4E5D_885D_540DB7355CFB */
