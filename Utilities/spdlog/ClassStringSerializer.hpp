#ifndef DAEEA81E_12F0_4234_BDF5_A08CA004F781
#define DAEEA81E_12F0_4234_BDF5_A08CA004F781

// #include "spdlog/fmt/ostr.h" // must be included

// #include <Eigen/Core>
// #include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>
// #include <Utilities/PCL_Helper/Basic/FieldChecker.hpp>

// namespace SPDLog
// {

// template<typename OStream, typename Scalar, int Rows, int Cols>
// OStream& operator<<(OStream& os, const Eigen::Matrix<Scalar, Rows, Cols>& InMat)
// {
// 	static std::stringstream ss;
// 	if constexpr ( Cols == 1 )
// 	{
// 		ss << "transposed: " << InMat.transpose();
// 	}
// 	else
// 	{
// 		ss << InMat;
// 	}
// 	fmt::format_to(std::ostream_iterator<char>(os), "{}", ss.str());
// 	return os;
// }

// template<typename OStream, typename PointType>
// OStream& operator<<(OStream& os, const PointType& InPoint)
// {
// 	constexpr bool bHAS_Z      = PCL_Helper::HAS_z_FIELD<PointType>();
// 	constexpr bool bHAS_COLOR  = PCL_Helper::HAS_RGB_FIELD<PointType>();
// 	constexpr bool bHAS_NORMAL = PCL_Helper::HAS_NORMAL_FIELD<PointType>();

// 	static std::stringstream ss;
// 	ss << "Pos: [" << InPoint.x << ", " << InPoint.y;
// 	if constexpr ( bHAS_Z )
// 	{
// 		ss << InPoint.z << "] ";
// 	}
// 	else
// 	{
// 		ss << "] ";
// 	}
// 	if constexpr ( bHAS_COLOR )
// 	{
// 		ss << "RGB: [" << InPoint.r << ", " << InPoint.g << ", " << InPoint.b << "] ";
// 	}
// 	if constexpr ( bHAS_NORMAL )
// 	{

// 		ss << "N: [" << InPoint.normal_x << ", " << InPoint.normal_y
// 		   << ", " << InPoint.normal_z << "] ";
// 	}

// 	fmt::format_to(std::ostream_iterator<char>(os), "{}", ss.str());
// 	return os;
// }

// } // namespace SPDLog

#endif /* DAEEA81E_12F0_4234_BDF5_A08CA004F781 */
