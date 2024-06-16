#ifndef DE4C7CF3_BB3F_4C4D_ADE1_59F8A5352A0E
#define DE4C7CF3_BB3F_4C4D_ADE1_59F8A5352A0E

#include "Utilities/JSON_Helper/IJsonLoadable.hpp"
#include <Utilities/PCL_Helper/Apps/PolygonFiller/PolygonFillerTypes.hpp>

namespace PCL_Helper::App
{
class PolygonFillerImpl : public IJsonLoadable
{
	void LoadConfig(const std::string& InJsonFilePath, const std::vector<std::string>& InConfigKeyList) override;
};

} // namespace PCL_Helper::App
#endif /* DE4C7CF3_BB3F_4C4D_ADE1_59F8A5352A0E */
