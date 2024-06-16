#ifndef B55A87DF_08A1_4F3E_B3C2_D183A2E38E6F
#define B55A87DF_08A1_4F3E_B3C2_D183A2E38E6F


#include <Eigen/Core>

/**
 * @brief 
 * 



In this program,
Knife's cross section has a triangle shape.
knife pose means the middle plane pose of the knife.
The rotation matrix.
		1st-column is the blade direction.
		3rd-column is the middle plane's normal direction.
		2nd-column is calculated by cross_product with a right-hand coordinate.
 * @tparam Scalar 
 */
template<typename Scalar>
class TKnifeTrajectoryNode
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// In this program,
	// Knife's cross section has a triangle shape.
	// knife pose means the middle plane pose of the knife.
	// The rotation matrix.
	// 		1st-column is the blade direction.
	// 		3rd-column is the middle plane's normal direction.
	// 		2nd-column is calculated by cross_product with a right-hand coordinate.
	Eigen::Matrix<Scalar, 4, 4> Pose;
	Eigen::Matrix<Scalar, 3, 1> Velocity;
};

#endif /* B55A87DF_08A1_4F3E_B3C2_D183A2E38E6F */
