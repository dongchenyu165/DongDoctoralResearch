#ifndef ADB09177_A442_443C_BB3E_49590F058A97
#define ADB09177_A442_443C_BB3E_49590F058A97

#include <Eigen/Core>

namespace EigenHelper
{

template<typename Scalar=float>
Eigen::Matrix<Scalar, 3, 3> MakeRotationMat(Eigen::Matrix<Scalar, 3, 1> MainDir, const char MainDirAxis, Eigen::Matrix<Scalar, 3, 1> SecondaryDir, const char SecondaryDirAxis)
{
	using AxisType = Eigen::Matrix<Scalar, 3, 1>;

	const char MainDirColumnIndex = MainDirAxis - 'x';
	const char SecondaryDirColumnIndex = SecondaryDirAxis - 'x';

	// Check if inputed [MainDirAxis] and [SecondaryDirAxis] is valid.
	assert(MainDirColumnIndex < 3 && MainDirColumnIndex >= 0);
	assert(SecondaryDirColumnIndex < 3 && SecondaryDirColumnIndex >= 0);
	assert(SecondaryDirColumnIndex != MainDirColumnIndex);

	const bool bIsSameVector = ((MainDir.cross(SecondaryDir)).isZero());
	if (bIsSameVector)
	{
		// print_ERR(__FUNCTION__, " === You input two same vectors!");
	}
	assert(!bIsSameVector);

	/** Get the column index of the third axis.
	 * Calculation as follows:
	 * 0 + 1 = 1; 3 - 1 = 2; 2
	 * 0 + 2 = 2; 3 - 2 = 1; 1
	 * 1 + 2 = 3; 3 - 3 = 0; 0
	 */
	const char ThirdDirColumnIndex = 3 - (SecondaryDirColumnIndex + MainDirColumnIndex);
	assert(ThirdDirColumnIndex < 3 && ThirdDirColumnIndex >= 0);

	AxisType NormalizedMainDir = MainDir / MainDir.norm();
	AxisType NormalizedOrgSecondaryDir = SecondaryDir / SecondaryDir.norm();

	// Make main axis
	Eigen::Matrix<Scalar, 3, 3> ResultMat;
	ResultMat.template block<3, 1>(0, MainDirColumnIndex) = NormalizedMainDir;

	// Make secondary axis
	// 1. Project to plane that perpendicular to MainDir's normal plane.
	AxisType ProjectedSenondaryDir = NormalizedOrgSecondaryDir - NormalizedOrgSecondaryDir.dot(NormalizedMainDir) * NormalizedMainDir;
	// 2. Normalized result.
	AxisType NormalizedSecondaryDir = ProjectedSenondaryDir / ProjectedSenondaryDir.norm();
	// 3. Insert result to the final result matrix.
	ResultMat.template block<3, 1>(0, SecondaryDirColumnIndex) = NormalizedSecondaryDir;

	// Calculate the third axis. Right hand coordinate.
	AxisType ThirdDir;
	if (ThirdDirColumnIndex == 1)
	{
		// z * x
		const AxisType& CrossOrgA = ResultMat.template block<3, 1>(0, 2);
		const AxisType& CrossOrgB = ResultMat.template block<3, 1>(0, 0);
		ThirdDir = (CrossOrgA).cross(CrossOrgB);
	}
	else if (ThirdDirColumnIndex == 2)
	{
		// x * y
		const AxisType& CrossOrgA = ResultMat.template block<3, 1>(0, 0);
		const AxisType& CrossOrgB = ResultMat.template block<3, 1>(0, 1);
		ThirdDir = (CrossOrgA).cross(CrossOrgB);
	}
	else if (ThirdDirColumnIndex == 0)
	{
		// y * z
		const AxisType& CrossOrgA = ResultMat.template block<3, 1>(0, 1);
		const AxisType& CrossOrgB = ResultMat.template block<3, 1>(0, 2);
		ThirdDir = (CrossOrgA).cross(CrossOrgB);
	}

	// Insert the third axis.
	ResultMat.block(0, ThirdDirColumnIndex, 3, 1) = ThirdDir;

	return ResultMat;
}

}

#endif /* ADB09177_A442_443C_BB3E_49590F058A97 */
