#pragma once
#include <algorithm>
#include <initializer_list>
#include <memory>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>


namespace EMST
{

template<typename Scalar, size_t Dim>
class Point
{
  public:
	using PointType = Eigen::Matrix<Scalar, Dim, 1>;

	Point(const PointType& InRef) : PointBuffer(InRef) { }

	Point(std::initializer_list<Scalar> InInitializer)
	{
		int i = 0;
		for ( auto PointData : InInitializer )
		{
			PointBuffer(i) = PointData;
			i++;
		}
	}

	Point() { PointBuffer = PointType::Zero(); }

	const Scalar& operator[](size_t i) const { return PointBuffer(i); }

	Scalar& operator[](size_t i) { return PointBuffer(i); }

	const PointType& GetPointBuffer() const { return PointBuffer; }

  protected:
	PointType PointBuffer;
};

template<typename Scalar, size_t Dim>
class PointWithID : Point<Scalar, Dim>
{
	using PointType = typename Point<Scalar, Dim>::PointType;

  public:
	PointWithID(const PointType& InRef, size_t InID = 0) : ID(InID) { this->PointBuffer = InRef; }

	PointWithID(size_t InID = 0) : ID(InID) { }

	size_t GetID() const { return ID; }

  private:
	size_t ID = 0;
};

/**
 * Bounding box in DIM-dimensional space
 */
template<typename Scalar, size_t Dim>
class AABB
{
	using PointType = Point<Scalar, Dim>;
	using EigenPointType = typename PointType::PointType;
	using PointListType = std::vector<PointType>;
	using PtListConstIterType = typename PointListType::const_iterator;

	using PointWithIDType = PointWithID<Scalar, Dim>;
	using PointWithIDListType = std::vector<PointWithIDType>;
	using PtWithIDListConstIterType = typename PointWithIDListType::const_iterator;

	using AABB_Type = AABB<Scalar, Dim>;
 
public:
	AABB() {}
	/**
	 * Construct bounding box from vector iterator
	 */
	AABB(PtWithIDListConstIterType iter, PtWithIDListConstIterType end)
	{
		Min = Max = *iter;
		for ( ; iter != end; ++iter )
		{
			for ( size_t k = 0; k < Dim; k++ )
			{
				Min[k] = std::min(Min[k], (*iter)[k]);
				Max[k] = std::max(Max[k], (*iter)[k]);
			}
		}
	}

	/**
	 * Size of the k-th dimension
	 */
	double GetDimensionSize(size_t InDimentionID) const
	{
		return Max[InDimentionID] - Min[InDimentionID];
	}

	/**
	 * Get the index of a dimension with the largest size
	 */
	size_t get_largest_dim() const
	{
		typename EigenPointType::Index Index;
		(Max.GetPointBuffer() - Min.GetPointBuffer()).maxCoeff(&Index);
		return (size_t)Index;
	}

	/**
	 * Distance between two bounding boxes
	 */
	template<size_t DIM_>
	friend Scalar GetDistanceToOtherAABB(const AABB_Type& InA, const AABB_Type& InB)
	{
		Scalar SquaredDistance = 0;
		for ( size_t k = 0; k < Dim; k++ )
		{
			Scalar Diff = std::max(InA.Min[k], InB.Min[k]) - std::min(InA.Max[k], InB.Max[k]);
			if ( Diff > 0 )
			{
				SquaredDistance += Diff * Diff;
			}
		}
		return sqrt(SquaredDistance);
	}

  private:
	PointType Min;
	PointType Max;
};

// Implementation


// template<size_t DIM>
// size_t AABB<DIM>::get_largest_dim() const
// {
// 	size_t biggest_dim = 0;
// 	for ( size_t k = 1; k < DIM; k++ )
// 	{
// 		if ( size(k) > size(biggest_dim) )
// 		{
// 			biggest_dim = k;
// 		}
// 	}
// 	return biggest_dim;
// }


} // namespace EMST