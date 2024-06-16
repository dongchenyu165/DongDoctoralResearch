#pragma once
#include <algorithm>
#include <vector>
#include <limits>
#include <memory>
#include "DataTypes.hpp"

// using namespace EMST;
namespace EMST
{

/**
 * K-d tree data structure
 */

template<typename Scalar, size_t Dim>
struct KdTree
{

	using PointType					= Point<Scalar, Dim>;
	using EigenPointType			= typename PointType::PointType;
	using PointListType				= std::vector<PointType>;
	using PtListConstIterType		= typename PointListType::const_iterator;

	using PointWithIDType			= PointWithID<Scalar, Dim>;
	using PointWithIDListType		= std::vector<PointWithIDType>;
	using PtWithIDListConstIterType = typename PointWithIDListType::const_iterator;

	using AABB_Type					= AABB<Scalar, Dim>;

  public:
	KdTree() { }

	/**
	 * Construct K-d tree with height at most max_height from a given set of points
	 */
	KdTree(const PointListType& points, size_t max_height)
	{
		this->points.reserve(points.size());
		for ( auto& p : points )
		{
			this->points.push_back(PointWithIDType(p, this->points.size()));
		}
		nodes.resize(points.size() * 4);
		build(1, 0, points.size(), max_height);
	}

	size_t get_root_id() const { return 1; }

	size_t get_maximal_id() const { return (nodes.empty() ? 0 : nodes.size() - 1); }

	size_t get_left_child_id(size_t id) const { return id * 2; }

	size_t get_right_child_id(size_t id) const { return id * 2 + 1; }

	bool is_leaf(size_t id) const { return nodes[id].is_leaf; }

	const AABB_Type& get_bounding_box(size_t id) const { return nodes[id].aabb; }

	PtWithIDListConstIterType points_begin(size_t node_id) const { return points.begin() + nodes[node_id].start; }

	PtWithIDListConstIterType points_end(size_t node_id) const { return points.begin() + nodes[node_id].start + nodes[node_id].size; }

  private:
	/**
	 * K-d tree node. Represents part of space with points from (start) to (start+size-1) position in points array
	 */
	struct KdNode
	{
		KdNode() : start(0), is_leaf(false), size(0) { }

		AABB_Type aabb;
		bool is_leaf; // is this node a leaf node and has no childs
		size_t start;
		size_t size;
	};

	/**
	 * Recursively build tree for current node node_id, for points from position (start) to position (start+size-1)
	 */
	void build(size_t node_id, size_t start, size_t size, size_t remaining_height)
	{
		nodes[node_id].aabb	 = AABB_Type(points.cbegin() + start, points.cbegin() + start + size);
		nodes[node_id].start = start;
		nodes[node_id].size	 = size;

		if ( size == 1 || remaining_height == 0 )
		{
			nodes[node_id].is_leaf = true;
			return;
		}

		size_t biggest = nodes[node_id].aabb.get_largest_dim();

		std::nth_element(
			points.begin() + start,
			points.begin() + start + size / 2,
			points.begin() + start + size,
			[&biggest](const PointType& a, const PointType& b) { return a[biggest] < b[biggest]; }
		);

		// recursively call build for two parts
		build(node_id * 2, start, size / 2, remaining_height - 1);
		build(node_id * 2 + 1, start + size / 2, (size + 1) / 2, remaining_height - 1);
	}

	PointWithIDListType points;
	std::vector<KdNode> nodes;
};

// Implementation

} // namespace EMST