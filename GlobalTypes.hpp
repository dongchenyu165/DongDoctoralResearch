#ifndef CDB22300_C712_4BB5_9D8B_D6D84A929F34
#define CDB22300_C712_4BB5_9D8B_D6D84A929F34

#include <Eigen/Core>

#include <GlobalVars.hpp>
#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <DataTypes/PointSetData.hpp>
#include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>

namespace Types
{

using CalcScalar = double;

using TrajectoryNode = TKnifeTrajectoryNode<CalcScalar>;
using Trajectory     = std::vector<TrajectoryNode>;

using ForceTorque = Eigen::Matrix<CalcScalar, 6, 1>;

using CalcPointSetData = TSearchSpaceElement<bCONSIDER_GRAVITY ? FINGER_NUMBER + 1 : FINGER_NUMBER>;
using HoldingPointSet  = Eigen::Matrix<CalcScalar, FINGER_NUMBER, 3>;
using SearchSpace      = std::vector<CalcPointSetData>;

using Vec3        = Eigen::Matrix<CalcScalar, 3, 1>;
using ConstVec3   = const Eigen::Matrix<CalcScalar, 3, 1>;
using Mat4x4      = Eigen::Matrix<CalcScalar, 4, 4>;
using ConstMat4x4 = const Eigen::Matrix<CalcScalar, 4, 4>;

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

#endif /* CDB22300_C712_4BB5_9D8B_D6D84A929F34 */
