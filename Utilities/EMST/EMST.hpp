#pragma once
#include <cstddef>
#include <vector>
#include "KDTree.hpp"
#include "DisjointSetUnion.hpp"

namespace EMST
{

template<typename Scalar, size_t Dim>
class EMSTSolver
{

	using PointType					= Point<Scalar, Dim>;
	using EigenPointType			= typename PointType::PointType;
	using PointListType				= std::vector<PointType>;
	using PtListConstIterType		= typename PointListType::const_iterator;

	using PointWithIDType			= PointWithID<Scalar, Dim>;
	using PointWithIDListType		= std::vector<PointWithIDType>;
	using PtWithIDListConstIterType = typename PointWithIDListType::const_iterator;

	using AABB_Type					= AABB<Scalar, Dim>;
	using EMSTSolverType			= EMSTSolver<Scalar, Dim>;
	using KdTreeType				= KdTree<Scalar, Dim>;

	typedef std::pair<size_t, size_t> Edge;

  public:
	explicit EMSTSolver(const PointListType& points) : num_points(points.size())
	{
		dsu.Reset(num_points);
		tree = KdTreeType(points, static_cast<size_t>(floor(log2(num_points)) - 1));
		is_fully_connected.assign(tree.get_maximal_id() + 1, false);
		solve();
		// todo: clear containers
	}

	const std::vector<Edge>& Solve() const { return solution; }

	const double& get_total_length() const { return total_length; }

  protected:
	std::vector<Edge> solution;
	double total_length = 0.0;

  private:
	void solve()
	{
		auto& solution	   = EMSTSolverType::solution;
		auto& total_length = EMSTSolverType::total_length;

		while ( solution.size() + 1 < num_points )
		{
			node_approximation.assign(tree.get_maximal_id() + 1, std::numeric_limits<double>::max());
			nearest_set.assign(num_points, { std::numeric_limits<double>::max(), Edge(0, 0) });

			check_fully_connected(tree.get_root_id());

			find_component_neighbors(tree.get_root_id(), tree.get_root_id());

			for ( size_t i = 0; i < num_points; i++ )
			{
				if ( i == dsu.GetRootParent(i) )
				{
					Edge e = nearest_set[i].second;
					if ( dsu.Merge(e.first, e.second) )
					{
						solution.push_back(e);
						total_length += nearest_set[i].first;
					}
				}
			}
		}
	}

	void find_component_neighbors(size_t q, size_t r, size_t depth = 0)
	{
		if ( is_fully_connected[q] && is_fully_connected[r]
			 && dsu.bIsInSameRootParent(tree.points_begin(q)->get_id(), tree.points_begin(r)->get_id()) )
		{
			return;
		}
		if ( distance(tree.get_bounding_box(q), tree.get_bounding_box(r)) > node_approximation[q] )
		{
			return;
		}
		if ( tree.is_leaf(q) && tree.is_leaf(r) )
		{
			node_approximation[q] = 0.0;
			for ( auto i = tree.points_begin(q); i != tree.points_end(q); i++ )
			{
				for ( auto j = tree.points_begin(r); j != tree.points_end(r); j++ )
				{
					if ( !dsu.bIsInSameRootParent(i->get_id(), j->get_id()) )
					{
						double dist = distance(*i, *j);
						if ( dist < nearest_set[dsu.GetRootParent(i->get_id())].first )
						{
							nearest_set[dsu.GetRootParent(i->get_id())] = {
								dist,
								{i->get_id(), j->get_id()}
							};
						}
					}
				}
				node_approximation[q] = std::max(node_approximation[q], nearest_set[dsu.GetRootParent(i->get_id())].first);
			}
		}
		else
		{
			size_t qleft  = tree.get_left_child_id(q);
			size_t qright = tree.get_right_child_id(q);
			size_t rleft  = tree.get_left_child_id(r);
			size_t rright = tree.get_right_child_id(r);
			if ( tree.is_leaf(q) )
			{
				find_component_neighbors(q, rleft, depth);
				find_component_neighbors(q, rright, depth);
				return;
			}
			if ( tree.is_leaf(r) )
			{
				find_component_neighbors(qleft, r, depth);
				find_component_neighbors(qright, r, depth);
				node_approximation[q] = std::max(node_approximation[qleft], node_approximation[qright]);
				return;
			}
			find_component_neighbors(qleft, rleft, depth + 1);
			find_component_neighbors(qleft, rright, depth + 1);
			find_component_neighbors(qright, rright, depth + 1);
			find_component_neighbors(qright, rleft, depth + 1);
			node_approximation[q] = std::max(node_approximation[qleft], node_approximation[qright]);
		}
	}

	void check_fully_connected(size_t node_id)
	{
		if ( is_fully_connected[node_id] )
		{
			return;
		}
		if ( tree.is_leaf(node_id) )
		{
			bool fully_connected = true;
			for ( auto iter = tree.points_begin(node_id); iter + 1 != tree.points_end(node_id); ++iter )
			{
				fully_connected &= dsu.bIsInSameRootParent(iter->get_id(), (iter + 1)->get_id());
			}
			is_fully_connected[node_id] = fully_connected;
			return;
		}
		size_t left	 = tree.get_left_child_id(node_id);
		size_t right = tree.get_right_child_id(node_id);
		check_fully_connected(left);
		check_fully_connected(right);
		if ( is_fully_connected[left] && is_fully_connected[right]
			 && dsu.bIsInSameRootParent(tree.points_begin(left)->get_id(), tree.points_begin(right)->get_id()) )
		{
			is_fully_connected[node_id] = true;
		}
	}

	size_t num_points;
	DisjointSetUnion dsu;
	KdTreeType tree;
	std::vector<bool> is_fully_connected;
	std::vector<double> node_approximation;
	std::vector<std::pair<double, Edge>> nearest_set;
};

} // namespace EMST