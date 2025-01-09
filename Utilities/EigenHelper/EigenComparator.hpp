#pragma once
#include <Eigen/Core>

namespace Utilities
{
template<typename Scalar, int Rows, int Cols, int Options = Eigen::ColMajor>
struct MatrixLess
{
	bool operator()(const Eigen::Matrix<Scalar, Rows, Cols, Options>& lhs,
		const Eigen::Matrix<Scalar, Rows, Cols, Options>& rhs) const
	{
		// Lexicographical comparison
		for ( int i = 0; i < Rows; ++i )
		{
			for ( int j = 0; j < Cols; ++j )
			{
				if ( lhs(i, j) != rhs(i, j) )
				{
					return lhs(i, j) < rhs(i, j);
				}
			}
		}
		return false;
	}
};
} // namespace Utilities