#ifndef B376567C_A3E4_46E0_BF1E_31068C71C010
#define B376567C_A3E4_46E0_BF1E_31068C71C010

#include <memory>
#include <string>
#include <sstream>
#include <cassert>
#include <algorithm>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/kruskal_min_spanning_tree.h>

#include <spdlog/logger.h>
#include <nlohmann/json.hpp>

#include <Utilities/spdlog/LogConfig.hpp>
#include <GlobalVars.hpp>
#include "../Basic/PCL_TypeAlias.hpp"
#include "GlobalTypes.hpp"
#include "GlobalVars.hpp"

#include "Utilities/PCL_Helper/Visualizer/DebugViewerManager.hpp"

// #include "Utilities/PCL_Helper/Visualizer/DebugViewer.hpp"
// #include "Utilities/PCL_Helper/Visualizer/PointArrangementViewer.hpp"

namespace PCL_Helper
{
namespace App
{

struct PointsArrangerParam
{
	Eigen::Matrix4f Tlocal2world;
};

template<typename PointType = PointXYZ>
class TPointsArranger
{
	PCPTR<PointType> OperatingPC;
	std::shared_ptr<spdlog::logger> Logger;

public:
	TPointsArranger()
	{
		Logger =
			SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PointArranger", LogConfigJsonPath, { "PointArranger" });
	}

	void LoadConfig(const std::string& InJsonPath);

	PCPTR<PointType> GetArrangedPC() { return OperatingPC; }

	void Clear()
	{
		for ( int i = 0; i < PointChildIndexList.size(); i++ )
		{
			PointChildIndexList[i].clear();
		}
		PointChildIndexList.clear();
	}

	void ArrangePoints(PCPTR<PointType> InPC)
	{
		using K                   = CGAL::Exact_predicates_inexact_constructions_kernel;
		using CGAL_Point2D        = K::Point_2;
		using Triangulation       = CGAL::Delaunay_triangulation_2<K>;
		using vertex_descriptor   = boost::graph_traits<Triangulation>::vertex_descriptor;
		using vertex_iterator     = boost::graph_traits<Triangulation>::vertex_iterator;
		using edge_descriptor     = boost::graph_traits<Triangulation>::edge_descriptor;
		using VertexIndexMap      = std::map<vertex_descriptor, int>;
		using VertexIdPropertyMap = boost::associative_property_map<VertexIndexMap>;

		using ChildIndexListType = std::set<int>;
		using TreeDictType       = std::map<int, ChildIndexListType>;

		LOG_FUNC_ENTER(Logger, debug, 0);
		Logger->info("Start arrange edge points");

		PCIDX_Ptr Index      = PCIDX_Ptr(new PCIDX);
		PCIDX_Ptr IndexEd    = PCIDX_Ptr(new PCIDX);
		PCIDX_Ptr FinalIndex = PCIDX_Ptr(new PCIDX);
		OperatingPC          = InPC;

		Clear();

		Logger->debug("Input cloud has {} points", OperatingPC->size());

		Logger->debug("Insert points to CGAL [Triangulation] types");
		Triangulation tr;
		CGAL_Point2D CGAL_Pt;
		for ( PointType PC_Point : *InPC )
		{
			tr.insert(CGAL_Point2D(PC_Point.x, PC_Point.y));
		}
		Logger->trace("FINISH... Insert {} points to CGAL [Triangulation] types.\n", tr.number_of_vertices());

		// Associate indices to the vertices
		VertexIndexMap vertex_id_map;
		VertexIdPropertyMap vertex_index_pmap(vertex_id_map);
		int index = 0;
		for ( vertex_descriptor vd : vertices(tr) )
		{
			vertex_id_map[vd] = index++;
		}

		// We use the default edge weight which is the squared length of the edge
		// This property map is defined in graph_traits_Triangulation_2.h
		// In the function call you can see a named parameter: vertex_index_map
		Logger->debug("Construct Euclidean mimimum spanning tree");
		std::list<edge_descriptor> mst;
		boost::kruskal_minimum_spanning_tree(tr, std::back_inserter(mst), vertex_index_map(vertex_index_pmap));

		TreeDictType Tree;
		std::map<int, int> ScanedDict;
		std::set<int> HasChildIndexSet;  // A set contains indexs of point that has child.
		std::set<int> HasParentIndexSet; // A set contains indexs of point that has parent.

		PointChildIndexList.clear();
		PointChildIndexList.assign(InPC->size(), std::vector<int>());

		// TPointArrangementViewer<PointType> Viewer;
		// Viewer.addPointCloud(InPC, "MainPC");

		Logger->debug("Loop edges of constructed euclidean mimimum spanning tree");
		for ( edge_descriptor ed : mst )
		{
			const int& StartIndex = vertex_index_pmap[CGAL::source(ed, tr)];
			const int& EndIndex   = vertex_index_pmap[CGAL::target(ed, tr)];

			PointChildIndexList[StartIndex].push_back(EndIndex);
			PointChildIndexList[EndIndex].push_back(StartIndex);

			Logger->trace("\t Edge: {} -> {}", StartIndex, EndIndex);
			// Viewer.AddSingleEdge(InPC, StartIndex, EndIndex);
		}

		std::set<int> FullIndexSet;
		for ( int i = 0; i < InPC->size(); i++ )
		{
			FullIndexSet.insert(i);
		}

		//
		std::set<int> NewStartIndexSet; // A set contains indexs of MST start point (root).
		for ( int i = 0; i < InPC->size(); i++ )
		{
			const auto& ChildList = PointChildIndexList[i];

			if ( ChildList.size() == 1 )
			{
				NewStartIndexSet.insert(i);
			}
		}

		// for ( int StartIndex : StartIndexSet )
		// {
		// 	std::cout << StartIndex << " ";
		// 	Viewer.addText3D("ST_" + std::to_string(StartIndex), (*InPC)[StartIndex], 0.0003);
		// 	// auto aaa = pcl::ModelCoefficients();
		// 	// aaa.values = {(*InPC)[StartIndex].x, (*InPC)[StartIndex].y, (*InPC)[StartIndex].z, 0.0005};
		// 	// Viewer.addSphere(PointType{(*InPC)[StartIndex].x, (*InPC)[StartIndex].y, (*InPC)[StartIndex].z}, 0.0005);
		// 	Viewer.addSphere((*InPC)[StartIndex], 0.0005, "ST_SPHERE_" + std::to_string(StartIndex));
		// }
		// std::cout << std::endl;
		// for ( int EndIndex : EndIndexSet )
		// {
		// 	std::cout << EndIndex << " ";
		// 	Viewer.addText3D("ED_" + std::to_string(EndIndex), (*InPC)[EndIndex], 0.0003);
		// 	// auto aaa = pcl::ModelCoefficients();
		// 	// aaa.values = {(*InPC)[EndIndex].x, (*InPC)[EndIndex].y, (*InPC)[EndIndex].z, 0.0005};
		// 	Viewer.addSphere((*InPC)[EndIndex], 0.0005, "ED_SPHERE_" + std::to_string(EndIndex));
		// }
		// std::cout << std::endl;
		// Viewer.resetCameraViewpoint("MainPC");
		// Viewer.spin();
		FindAllLinks(NewStartIndexSet);
		// MergeLinks(StartIndexSet);

		// TPointArrangementViewer<PointType> Viewer;
		// Viewer.AddPointCloud(InPC, Index);
		// Viewer.spin();



		LOG_FUNC_EXIT(Logger, debug, 0);
	}

private:
	void FindAllLinks(const std::set<int>& InStartIndexSet)
	{
		using TreeType = std::vector<std::vector<int>>; // Storage child index list of each node.
		std::vector<std::vector<int>> LinkList;
		std::vector<int> LongestLink;

		// Find all link by link start index.
		for ( int StartIndex : InStartIndexSet )
		{

			std::vector<int> CurrentLink         = { StartIndex };
			std::vector<int> SearchingIndexStack = PointChildIndexList[StartIndex];
			std::vector<int> CurrentLinkRollBack;
			CurrentLinkRollBack.assign(SearchingIndexStack.size(), 0);
			while ( !SearchingIndexStack.empty() )
			{
				const int CurrentIndex = SearchingIndexStack[0];
				SearchingIndexStack.erase(SearchingIndexStack.begin());
				CurrentLinkRollBack.erase(CurrentLinkRollBack.begin());

				if ( std::find(CurrentLink.begin(), CurrentLink.end(), CurrentIndex) != CurrentLink.end() )
				{
					continue;
				}

				CurrentLink.push_back(CurrentIndex);

				const std::vector<int>& NextSearchList = PointChildIndexList[CurrentIndex];
				if ( NextSearchList.size() == 1 )
				{
					PrintLink(CurrentLink, "Cur  : ");
					PrintLink(SearchingIndexStack, "Stack: ");
					PrintLink(CurrentLinkRollBack, "Rollb: ");

					if ( CurrentLink.size() > LongestLink.size() )
					{
						LongestLink = CurrentLink;
					}

					// Rollback
					const int& RollbackIndex = CurrentLinkRollBack.front();
					CurrentLink.erase(CurrentLink.begin() + RollbackIndex + 1, CurrentLink.end());

					continue;
				}

				int i = 0;
				for ( int PendingCheckingIndex : NextSearchList )
				{
					if ( std::find(CurrentLink.begin(), CurrentLink.end(), PendingCheckingIndex) != CurrentLink.end() )
					{
						continue;
					}
					SearchingIndexStack.insert(SearchingIndexStack.begin() + i, PendingCheckingIndex);
					CurrentLinkRollBack.insert(CurrentLinkRollBack.begin(), CurrentLink.size() - 1);
					i++;
				}
			}
		}

		std::cout << "Longest link : ";
		for ( int idx : LongestLink )
		{
			std::cout << idx << " - ";
		}
		std::cout << std::endl;

		PCIDX_Ptr PointIndexs = PCIDX_Ptr(new PCIDX);
		PointIndexs->indices  = std::move(LongestLink);

		// TPointArrangementViewer<PointType> Viewer;
		// Viewer.AddPointCloud(OperatingPC, PointIndexs);
		// Viewer.spin();

		DEBUG_SHOW_POLYGON_CUSTOM_IDX("ArrangeResult", "CuttingFace", OperatingPC, PointIndexs);
		
		PCPTR<PointType> ResultPC = PCPTR<PointType>(new PC<PointType>);
		for (int i = 0; i < PointIndexs->indices.size(); i++)
		{
			ResultPC->push_back(OperatingPC->points[PointIndexs->indices[i]]);
		}
		OperatingPC = ResultPC;
		DEBUG_SHOW_PC_LIST("DebugViewer", "Debug", OperatingPC);

	}

	void PrintLink(const std::vector<int>& InLink, const std::string& InPromptMsg)
	{
		std::stringstream stream;
		stream << InPromptMsg;
		for ( int idx : InLink )
		{
			stream << idx << " - ";
		}
		// std::cout << "??? " << std::endl;
		Logger->debug("{}: {}", InPromptMsg, stream.str());
	}

	// template<typename SetDataType>
	// std::set<SetDataType> SetIntersection(const std::set<SetDataType>& InSetA, const std::set<SetDataType>& InSetB)
	// {
	// 	std::set<SetDataType> ReturnSet;
	// 	std::set_intersection(InSetA.begin(), InSetA.end(), InSetB.begin(), InSetB.end(),
	// 		std::inserter(ReturnSet, ReturnSet.begin()));
	// 	return ReturnSet;
	// }

	// template<typename SetDataType>
	// std::set<SetDataType> SetDiff(const std::set<SetDataType>& InSetA, const std::set<SetDataType>& InSetB)
	// {
	// 	std::set<SetDataType> ReturnSet;
	// 	std::set_difference(InSetA.begin(), InSetA.end(), InSetB.begin(), InSetB.end(),
	// 		std::inserter(ReturnSet, ReturnSet.begin()));
	// 	return ReturnSet;
	// }

	// template<typename SetDataType>
	// std::set<SetDataType> SetSubstrate(const std::set<SetDataType>& InSetA, const std::set<SetDataType>& InSetB)
	// {
	// 	std::set<SetDataType> ReturnSet, IntermediaSet;
	// 	std::set_intersection(InSetA.begin(), InSetA.end(), InSetB.begin(), InSetB.end(),
	// 		std::inserter(IntermediaSet, IntermediaSet.begin()));
	// 	std::set_difference(InSetA.begin(), InSetA.end(), IntermediaSet.begin(), IntermediaSet.end(),
	// 		std::inserter(ReturnSet, ReturnSet.begin()));
	// 	return ReturnSet;
	// }

	std::vector<std::vector<int>> PointChildIndexList;
};

} // namespace App
} // namespace PCL_Helper

#endif /* B376567C_A3E4_46E0_BF1E_31068C71C010 */
