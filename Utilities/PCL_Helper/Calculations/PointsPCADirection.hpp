#ifndef A9AB32B3_1DBD_42F8_A8F8_7BF33DDCD297
#define A9AB32B3_1DBD_42F8_A8F8_7BF33DDCD297

#include <Eigen/Core>
#include <Eigen/Dense>


namespace PCL_Helper
{
	template<typename _Scalar, int _Rows, int _Options>
	static Eigen::Matrix<_Scalar, 3, 1> CalcPCADirection(Eigen::Matrix<_Scalar, _Rows, 3, _Options> InPoints)
	{
		const Eigen::Matrix<_Scalar, 3, 1> Mean = InPoints.colwise().mean();
		const decltype(InPoints) CenteredPoints = InPoints.rowwise() - Mean.transpose();
		const Eigen::Matrix3d Covariance = (CenteredPoints.transpose() * CenteredPoints) / double(InPoints.rows() - 1);

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> EigenCalcObj(Covariance);
		
		// The eigenvalues are sorted in increasing order.
		// col 2 means the eigenvector with largest eigenvalue
		return EigenCalcObj.eigenvectors().col(2);  
	}
}

#endif /* A9AB32B3_1DBD_42F8_A8F8_7BF33DDCD297 */
