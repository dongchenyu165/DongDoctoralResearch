#ifndef B955690E_9085_448F_82A6_3A16847A66AC
#define B955690E_9085_448F_82A6_3A16847A66AC

#include <Eigen/Core>

#include <GlobalVars.hpp>
#include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

namespace Types
{

using CalcScalar = double;
using ForceTorqueType = Eigen::Matrix<CalcScalar, 6, 1>;

using Vec3        = Eigen::Matrix<CalcScalar, 3, 1>;
using ConstVec3   = const Eigen::Matrix<CalcScalar, 3, 1>;
using Mat4x4      = Eigen::Matrix<CalcScalar, 4, 4>;
using ConstMat4x4 = const Eigen::Matrix<CalcScalar, 4, 4>;
#define DEF_MATRIX_TYPES_ALIASES(SCALAR_TYPE) \
	using Vec3        = Eigen::Matrix<SCALAR_TYPE, 3, 1>; \
	using ConstVec3   = const Eigen::Matrix<SCALAR_TYPE, 3, 1>; \
	using Mat4x4      = Eigen::Matrix<SCALAR_TYPE, 4, 4>; \
	using ConstMat4x4 = const Eigen::Matrix<SCALAR_TYPE, 4, 4>;

using CalcPoint = PCL_Helper::PointXYZRGBN;
// Point cloud with a [PointXYZRGBN] point type for this program.
using CalcPC = PCL_Helper::PC<CalcPoint>;
// Point cloud pointer type of type [CalcPC].
using CalcPCPTR      = PCL_Helper::PCPTR<CalcPoint>;
using CalcPCPTR_List = std::vector<PCL_Helper::PCPTR<CalcPoint>>;

} // namespace Types

/**
 * @brief Make a PC smart-pointer variable.
 */
#define NEW_CALC_PC_PTR(VAR_NAME) Types::CalcPCPTR VAR_NAME = Types::CalcPCPTR(new Types::CalcPC)


#endif /* B955690E_9085_448F_82A6_3A16847A66AC */
