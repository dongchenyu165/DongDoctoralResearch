#ifndef E5ADB3DD_40E0_424B_AEFB_D4DE134C1C30
#define E5ADB3DD_40E0_424B_AEFB_D4DE134C1C30

#include <Eigen/Core>
#include <nlohmann/json.hpp>

namespace nlohmann
{

template<typename Scalar, int Rows, int Cols>
struct adl_serializer<Eigen::Matrix<Scalar, Rows, Cols>>
{
	static void to_json(json& InJsonObj, const Eigen::Matrix<Scalar, Rows, Cols>& InMat)
	{
		for ( int i = 0; i < Rows * Cols; i++ )
		{
			InJsonObj.push_back(InMat(i));
		}
	}

	static void from_json(const json& InJsonObj, Eigen::Matrix<Scalar, Rows, Cols>& InMat)
	{
		for ( int i = 0; i < InJsonObj.size(); i++ )
		{
			InMat(i) = InJsonObj[i];
		}
	}
};
} // namespace nlohmann

#endif /* E5ADB3DD_40E0_424B_AEFB_D4DE134C1C30 */
