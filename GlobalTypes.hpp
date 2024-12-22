#ifndef CDB22300_C712_4BB5_9D8B_D6D84A929F34
#define CDB22300_C712_4BB5_9D8B_D6D84A929F34

#include <Eigen/Core>

#include <GlobalBaseTypes.hpp>
#include <GlobalVars.hpp>
#include <DataTypes/KnifeTrajectoryNode.hpp>
#include <DataTypes/PointSetData.hpp>

namespace Types
{

using TrajectoryNode = TKnifeTrajectoryNode<CalcScalar>;
using Trajectory     = std::vector<TrajectoryNode, Eigen::aligned_allocator<TrajectoryNode>>;

// 3D point list for holding(finger tip position) in (N, 3) format.
using HoldingPointSet = Eigen::Matrix<CalcScalar, FINGER_NUMBER, 3>;

// Type alias for the calculation related data (like [position] [point normal] [point index] etc...) for a point set.
using CalcPointSetData         = TSearchSpaceElement<bCONSIDER_GRAVITY ? FINGER_NUMBER + 1 : FINGER_NUMBER>;
// Type alias for the shared pointer of the [CalcPointSetData]. @see CalcPointSetData
using CalcPointSetDataPtr      = std::shared_ptr<CalcPointSetData>;
using CalcPointSetDataConstPtr = std::shared_ptr<const CalcPointSetData>;
using SearchSpace              = std::vector<CalcPointSetDataPtr>;

} // namespace Types

#endif /* CDB22300_C712_4BB5_9D8B_D6D84A929F34 */
