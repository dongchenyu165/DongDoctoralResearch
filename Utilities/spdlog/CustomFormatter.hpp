#ifndef A8053CAA_A240_41C9_BBD4_21D3F29AD183
#define A8053CAA_A240_41C9_BBD4_21D3F29AD183

#include "Utilities/PCL_Helper/Basic/FieldChecker.hpp"
#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "spdlog/fmt/fmt.h"
#include <Eigen/src/Core/Matrix.h>
#include <iomanip>
#include <iostream>
#include <memory>


namespace fmt
{


// Helper type trait to detect Eigen expressions
template<typename T, typename = void>
struct is_eigen_expression : std::false_type
{
};

template<typename T>
struct is_eigen_expression<T,
	std::void_t<decltype(std::declval<T>().rows()),
		decltype(std::declval<T>().cols()),
		decltype(std::declval<T>().eval())>> : std::true_type
{
};

template<typename T>
inline constexpr bool is_eigen_expression_v = is_eigen_expression<T>::value;

// Generic formatter for any Eigen expression
template<typename T>
struct formatter<T, char, std::enable_if_t<is_eigen_expression_v<T>>>
{
	char Presentation = 'f';
	int Precision     = 6;
	int Width         = 0;
	bool ForcePlus    = false;
	bool ZeroPad      = false;

	constexpr auto parse(format_parse_context& InCtx)
	{
		auto It = InCtx.begin();

		// Parse sign
		if ( It != InCtx.end() && *It == '+' )
		{
			ForcePlus = true;
			++It;
		}

		// Parse width and zero padding
		if ( It != InCtx.end() )
		{
			if ( *It == '0' )
			{
				ZeroPad = true;
				++It;
			}

			Width = 0;
			while ( It != InCtx.end() && std::isdigit(*It) )
			{
				Width = Width * 10 + (*It - '0');
				++It;
			}
		}

		// Parse precision
		if ( It != InCtx.end() && *It == '.' )
		{
			++It;
			Precision = 0;
			while ( It != InCtx.end() && std::isdigit(*It) )
			{
				Precision = Precision * 10 + (*It - '0');
				++It;
			}
		}

		// Parse presentation type
		if ( It != InCtx.end() && (*It == 'f' || *It == 'e' || *It == 'g') )
		{
			Presentation = *It++;
		}

		if ( It != InCtx.end() && *It != '}' )
		{
			throw format_error("invalid format");
		}

		return It;
	}

	template<typename FormatContext>
	auto format(const T& InExpr, FormatContext& InOutCtx)
	{
		auto Evaluated = InExpr.eval();
		auto OutIter   = InOutCtx.out();

		std::stringstream ss;
		// Set format flags
		ss.precision(Precision);
		if ( ForcePlus )
		{
			ss << std::showpos;
		}
		if ( ZeroPad )
		{
			// ss << std::setfill('0');
		}
		if ( Width > 0 )
		{
			// ss << std::setw(Width);
		}

		switch ( Presentation )
		{
		case 'f':
			ss << std::fixed;
			break;
		case 'e':
			ss << std::scientific;
			break;
		case 'g':
			ss << std::defaultfloat;
			break;
		}

		// Configure Eigen format
		Eigen::IOFormat fmt(Precision, 0, ", ", ";\n", "[", "]");
		if (Evaluated.cols() == 1)
		{
			ss << "col-vector Transposed: " << Evaluated.transpose().format(fmt);
		}
		else
		{
			ss << Evaluated.format(fmt);
		}

		return format_to(OutIter, "{}", ss.str());
	}
};

template<>
struct formatter<PCL_Helper::PointXYZRGBN> : formatter<string_view>

{
	using PointType = PCL_Helper::PointXYZRGBN;

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
