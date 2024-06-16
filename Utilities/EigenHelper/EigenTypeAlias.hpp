#ifndef CA7A36C9_5071_4568_B7B5_B6163A5F5C06
#define CA7A36C9_5071_4568_B7B5_B6163A5F5C06

#include <Eigen/Core>

namespace EigenHelper
{

namespace Types
{

typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix4d Mat4d;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix3d Mat3d;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3d;

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Vector6f Vec6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Vector7f Vec7f;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Vector6d Vec6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Vector7d Vec7d;

} // namespace Types
} // namespace EigenHelper

#endif /* CA7A36C9_5071_4568_B7B5_B6163A5F5C06 */
