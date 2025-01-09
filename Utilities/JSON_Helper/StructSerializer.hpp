#ifndef DA414443_36E5_4575_B477_400CDBBCC5AB
#define DA414443_36E5_4575_B477_400CDBBCC5AB

#include <fstream>

#include <nlohmann/json.hpp>


namespace JSON_Helper
{
using json = nlohmann::json;

/**
 * @brief Load a struct/class from a json file.
 * The key name in json dictionary should be the struct's member name.
 *
 * 		@ref: NLOHMANN_DEFINE_TYPE_INTRUSIVE macro: https://github.com/nlohmann/json#simplify-your-life-with-macros,


 * For [InKeyList] parameter, if we have a json file:,
 *
 * {
 *	 "MakeCuttingFace":{
 *	 	"SliceParam": {
 *	 		"PlaneNormal": [0, 0, 1],
 *	 		"PlanePosition": [0, 0, 0.04],
 *	 		"GrabbingThickness": 0.002,
 *	 		"Advance_PlanePoseSecondaryAxisRefDir": [1, 0, 0],
 *	 		"bProjectToPlane": true
 *	 	}
 *	 }
 * }
 * ```
 * @tparam StructType
 * @param InJsonPath
 * @param InKeyList A key list (or path) to navigate to the json dictionary of structure.
 * @param OutStruct
 */
template<typename StructType, typename JsonObjType = json>
void LoadStructure_ByPath(const std::string& InJsonPath, const std::vector<std::string>& InKeyList, StructType& OutStruct)
{
	std::ifstream f(InJsonPath);
	JsonObjType OrgData = JsonObjType::parse(f, nullptr, true, true);

	JsonObjType Data    = OrgData;
	for ( auto CurKey : InKeyList )
	{
		Data = Data[CurKey];
	}

	OutStruct = Data.template get<StructType>();
}

template<typename StructType, typename JsonObjType = json>
void LoadStructure_ByJsonStr(const std::string& InJsonStr, const std::vector<std::string>& InKeyList, StructType& OutStruct)
{
	std::stringstream ss(InJsonStr);
	JsonObjType OrgData = JsonObjType::parse(ss, nullptr, true, true);

	JsonObjType Data    = OrgData;
	for ( auto CurKey : InKeyList )
	{
		Data = Data[CurKey];
	}

	OutStruct = Data.template get<StructType>();
}

} // namespace JSON_Helper

#endif /* DA414443_36E5_4575_B477_400CDBBCC5AB */
