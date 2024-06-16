#ifndef D88DA615_0B49_436B_B2F1_857CC7AA3B34
#define D88DA615_0B49_436B_B2F1_857CC7AA3B34

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <Utilities/JSON_Helper/EigenSeiralizer.hpp>

namespace PCL_Helper
{
namespace App
{

struct PolygonFillerConfig
{
	Eigen::Vector3f FillingGridSize;
	Eigen::Vector3f FinalDownSampleSize;

	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PolygonFillerConfig, FillingGridSize, FillingGridSize);
};

} // namespace App
} // namespace PCL_Helper

#endif /* D88DA615_0B49_436B_B2F1_857CC7AA3B34 */
