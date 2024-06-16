#ifndef A8053CAA_A240_41C9_BBD4_21D3F29AD183
#define A8053CAA_A240_41C9_BBD4_21D3F29AD183

#include "Utilities/PCL_Helper/Basic/FieldChecker.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "spdlog/pattern_formatter.h"
#include "spdlog/fmt/fmt.h"
#include <iostream>
#include <memory>

// namespace SPDLog
// {

// class PCL_PointFormatter : public spdlog::custom_flag_formatter
// {
// public:
// 	void format(
// 		const spdlog::details::log_msg& InMsg,
// 		const std::tm& InTime,
// 		spdlog::memory_buf_t& InDestBuff
// 	) override
// 	{
// 		std::string some_txt = "custom-flag";
// 		InDestBuff.append(some_txt.data(), some_txt.data() + some_txt.size());

// 		std::cout << (InMsg.payload.data()) << std::endl;
// 	}

// 	std::unique_ptr<custom_flag_formatter> clone() const override
// 	{
// 		return spdlog::details::make_unique<PCL_PointFormatter>();
// 	}
// };

// } // namespace SPDLog

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

namespace fmt
{
// inline namespace v10
// {

template<typename Scalar, int Rows, int Cols>
struct formatter<Eigen::Matrix<Scalar, Rows, Cols>> : formatter<string_view>
{
	using MatType = Eigen::Matrix<Scalar, Rows, Cols>;

	auto format(const MatType& InMat, format_context& ctx) const -> format_context::iterator
	{
		// static std::string str;
		std::stringstream ss;
		if constexpr ( Cols == 1 )
		{
			// str += fmt::format("Transposed: {}", InMat.transpose());
			ss << "transposed: " << InMat.transpose();
		}
		else
		{
			// str += fmt::format("{}", InMat);
			ss << InMat;
		}
		// formatter<string_view>::format(str, ctx);
		formatter<string_view>::format(ss.str(), ctx);
		return ctx.out();
	}
};

// template<int Rows, int Cols>
// struct formatter<Eigen::Map<Eigen::Matrix<double, Rows, Cols>>> : formatter<Eigen::Matrix<double, Rows, Cols>>
// {};

// template<int Rows, int Cols>
// struct formatter<Eigen::Map<Eigen::Matrix<float, Rows, Cols>>> : formatter<Eigen::Matrix<float, Rows, Cols>>
// {};

// template<int Rows, int Cols>
// struct formatter<Eigen::Map<Eigen::Matrix<int, Rows, Cols>>> : formatter<Eigen::Matrix<int, Rows, Cols>>
// {};

template<typename Scalar, int Rows, int Cols>
struct formatter<Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>>> : formatter<Eigen::Matrix<Scalar, Rows, Cols>>
{
	// using MatType = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>>;

	// auto format(const MatType& InMat, format_context& ctx) const -> format_context::iterator
	// {
	// 	static std::string str;
	// 	// std::stringstream ss;
	// 	if constexpr ( Cols == 1 )
	// 	{
	// 		// ss << "transposed: " << InMat.transpose();
	// 		str += fmt::format("Transposed: {}", InMat.transpose());
	// 	}
	// 	else
	// 	{
	// 		// ss << InMat;
	// 		str += fmt::format("{}", InMat);
	// 	}
	// 	formatter<string_view>::format(str, ctx);
	// 	return ctx.out();
	// }
};

// template<typename PointType>
template<>
struct formatter<PCL_Helper::PointXYZRGBN> : formatter<string_view>

{
	using PointType = PCL_Helper::PointXYZRGBN;

	// struct fmt::formatter<PCL_Helper::PointXYZ> {
	// struct fmt::formatter {
	// Parses format specifiers and stores them in the formatter.
	//
	// [ctx.begin(), ctx.end()) is a, possibly empty, character range that
	// contains a part of the format string starting from the format
	// specifications to be parsed, e.g. in
	//
	//   fmt::format("{:f} continued", ...);
	//
	// the range will contain "f} continued". The formatter should parse
	// specifiers until '}' or the end of the range. In this example the
	// formatter should parse the 'f' specifier and return an iterator
	// pointing to '}'.
	constexpr auto parse(format_parse_context& ctx) -> format_parse_context::iterator
	{
		// p-Point data  c-RGB Color  n-Normal  s-SIFT
		auto It = ctx.begin(), end = ctx.end();

		// Check if reached the end of the range:
		for ( ; *It != '}'; ++It )
		{
			switch ( *It )
			{
			case 'p':
				bPrintPoint = true;
				break;
			case 'c':
				bPrintColor = true;
				break;
			case 'n':
				bPrintNormal = true;
				break;
			default:
				break;
			}
		}
		return It;
	}

	bool bPrintPoint  = true;
	bool bPrintColor  = false;
	bool bPrintNormal = false;

	// Formats value using the parsed format specification stored in this
	// formatter and writes the output to ctx.out().
	auto format(const PointType& InPoint, format_context& ctx) const -> format_context::iterator
	{
		constexpr bool bHAS_Z      = PCL_Helper::HAS_z_FIELD<PointType>();
		constexpr bool bHAS_COLOR  = PCL_Helper::HAS_RGB_FIELD<PointType>();
		constexpr bool bHAS_NORMAL = PCL_Helper::HAS_NORMAL_FIELD<PointType>();

		std::stringstream ss;
		static std::string str;
		str.clear();
		if ( bPrintPoint )
		{
			str += fmt::format("Pos: [{}, {}", InPoint.x, InPoint.y);

			if constexpr ( bHAS_Z )
			{
				str += fmt::format(", {}]", InPoint.z); // Print [z] field
			}
			else
			{
				str += fmt::format("] "); // ] for [xy]
			}
		}
		if constexpr ( bHAS_COLOR )
		{
			if ( bPrintColor )
			{
				str += fmt::format("RGB: [{:d}, {:d}, {:d}] ", InPoint.r, InPoint.g, InPoint.b);
			}
		}
		if constexpr ( bHAS_NORMAL )
		{
			if ( bPrintNormal )
			{
				str += fmt::format("N: [{}, {}, {}] ", InPoint.normal_x, InPoint.normal_y, InPoint.normal_z);
			}
		}

		formatter<string_view>::format(str, ctx);
		return ctx.out();
	}
};

template<>
struct formatter<PCL_Helper::PointXY> : formatter<PCL_Helper::PointXYZRGBN>
{
};

template<>
struct formatter<PCL_Helper::PointXYZ> : formatter<PCL_Helper::PointXYZRGBN>
{
};

template<>
struct formatter<PCL_Helper::PointXYZN> : formatter<PCL_Helper::PointXYZRGBN>
{
};

template<>
struct formatter<PCL_Helper::PointXYZRGB> : formatter<PCL_Helper::PointXYZRGBN>
{
};

// } // namespace v10
} // namespace fmt

#endif /* A8053CAA_A240_41C9_BBD4_21D3F29AD183 */
