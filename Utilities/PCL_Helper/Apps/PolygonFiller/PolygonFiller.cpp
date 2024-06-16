#include "impl/PolygonFillerImpl.hpp"
#include "PolygonFiller.hpp"


namespace PCL_Helper
{
namespace App
{

template<>
void TPolygonFiller<Types::CalcPoint>::Test()
{
	_impl = new PolygonFillerImpl();
}


} // namespace App
} // namespace PCL_Helper