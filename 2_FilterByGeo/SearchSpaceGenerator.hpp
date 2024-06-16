#ifndef A119453D_5783_4C1C_8C21_B8F9E8862DBF
#define A119453D_5783_4C1C_8C21_B8F9E8862DBF

#include "Utilities/spdlog/LogConfig.hpp"
#include <GlobalTypes.hpp>

namespace PCL_Helper
{

class SearchSpaceGenerator
{
public:
	SearchSpaceGenerator(Types::CalcPCPTR InPC);

	size_t Generate(std::vector<Types::CalcPointSetData>& OutSearchSpace);

private:
	Types::CalcPCPTR OperatingPC;
	SPDLog::LoggerType Logger;
};

} // namespace PCL_Helper

#endif /* A119453D_5783_4C1C_8C21_B8F9E8862DBF */
