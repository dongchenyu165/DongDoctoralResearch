#ifndef AB60891A_D5E1_49F2_9CB6_53920C038FBA
#define AB60891A_D5E1_49F2_9CB6_53920C038FBA
#ifndef B376567C_A3E4_46E0_BF1E_31068C71C010
#define B376567C_A3E4_46E0_BF1E_31068C71C010

#include <algorithm>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/kruskal_min_spanning_tree.h>

#include <cassert>
#include <nlohmann/json.hpp>
#include <string>

#include "../Basic/PCL_TypeAlias.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Utilities/PCL_Helper/Visualizer/PointArrangementViewer.hpp"

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

	// TPointsArranger() { }

  public:
	void LoadConfig(const std::string& InJsonPath);

	void ArrangePoints(PCPTR<PointType> InPC)
	{
		using K					  = CGAL::Exact_predicates_inexact_constructions_kernel;
		using CGAL_Point2D		  = K::Point_2;
		using Triangulation		  = CGAL::Delaunay_triangulation_2<K>;
		using vertex_descriptor	  = boost::graph_traits<Triangulation>::vertex_descriptor;
		using vertex_iterator	  = boost::graph_traits<Triangulation>::vertex_iterator;
		using edge_descriptor	  = boost::graph_traits<Triangulation>::edge_descriptor;
		using VertexIndexMap	  = std::map<vertex_descriptor, int>;
		using VertexIdPropertyMap = boost::associative_property_map<VertexIndexMap>;

		using ChildIndexListType  = std::set<int>;
		using TreeDictType		  = std::map<int, ChildIndexListType>;

		PCIDX_Ptr Index			  = PCIDX_Ptr(new PCIDX);
		PCIDX_Ptr IndexEd		  = PCIDX_Ptr(new PCIDX);
		PCIDX_Ptr FinalIndex	  = PCIDX_Ptr(new PCIDX);
		OperatingPC				  = InPC;

		Triangulation tr;
		CGAL_Point2D CGAL_Pt;
		for ( PointType PC_Point : *InPC )
		{
			tr.insert(CGAL_Point2D(PC_Point.x, PC_Point.y));
		}
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
		std::list<edge_descriptor> mst;
		boost::kruskal_minimum_spanning_tree(tr, std::back_inserter(mst), vertex_index_map(vertex_index_pmap));

		TreeDictType Tree;
		std::map<int, int> ScanedDict;
		std::set<int> HasChildIndexSet;	 // A set contains indexs of point that has child.
		std::set<int> HasParentIndexSet; // A set contains indexs of point that has parent.

		PointChildIndexList.clear();
		PointChildIndexList.assign(InPC->size(), std::vector<int>());

		TPointArrangementViewer<PointType> Viewer;
		Viewer.addPointCloud(InPC, "MainPC");

		// std::cout << "The edges of the Euclidean mimimum spanning tree:" << std::endl;
		for ( edge_descriptor ed : mst )
		{
			// vertex_descriptor svd			= CGAL::source(ed, tr);
			// vertex_descriptor tvd			= CGAL::target(ed, tr);
			// Triangulation::Vertex_handle sv = svd;
			// Triangulation::Vertex_handle tv = tvd;

			const int& StartIndex = vertex_index_pmap[CGAL::source(ed, tr)];
			const int& EndIndex	  = vertex_index_pmap[CGAL::target(ed, tr)];
			// const bool bNewStartIndex		= Tree.find(StartIndex) == Tree.end();

			PointChildIndexList[StartIndex].push_back(EndIndex);
			PointChildIndexList[EndIndex].push_back(StartIndex);

			HasChildIndexSet.insert(HasChildIndexSet.end(), StartIndex);
			HasParentIndexSet.insert(HasParentIndexSet.end(), EndIndex);

			std::cout // << "[ " << sv->point() << "  |  " << tv->point() << " ] "
				<< "  ==== [ " << StartIndex << "  ->  " << EndIndex << " ] " << std::endl;
			Viewer.AddSingleEdge(InPC, StartIndex, EndIndex);

			// if ( bNewStartIndex )
			// {
			// 	ChildIndexListType _tmp = { EndIndex };
			// 	Tree.insert(std::pair<int, ChildIndexListType>(StartIndex, _tmp));
			// }
			// else
			// {
			// 	Tree[StartIndex].insert(EndIndex);
			// }

			// const bool bNewEndIndex = Tree.find(EndIndex) == Tree.end();
			// if ( bNewEndIndex )
			// {
			// 	ChildIndexListType _tmp = { StartIndex };
			// 	Tree.insert(std::pair<int, ChildIndexListType>(EndIndex, _tmp));
			// }
			// else
			// {
			// 	Tree[EndIndex].insert(StartIndex);
			// }
			// // 给ScanedDict赋初值
			// ScanedDict.insert_or_assign(vertex_index_pmap[sv], 0);
			// ScanedDict.insert_or_assign(vertex_index_pmap[tv], 0);

			// Index->indices.push_back(vertex_index_pmap[sv]);
			// IndexEd->indices.push_back(vertex_index_pmap[tv]);
		}

		std::set<int> FullIndexSet;
		for ( int i = 0; i < InPC->size(); i++ )
		{
			FullIndexSet.insert(i);
		}

		std::set<int> LinkMiddleIndexSet;
		std::set<int> StartIndexSet;	// A set contains indexs of MST start point (root).
		std::set<int> NewStartIndexSet; // A set contains indexs of MST start point (root).
		std::set<int> EndIndexSet;		// A set contains indexs of MST end point (leaf).
		// StartIndexSet = SetSubstrate(HasChildIndexSet, HasParentIndexSet);
		// EndIndexSet	  = SetSubstrate(HasParentIndexSet, HasChildIndexSet);

		StartIndexSet = SetSubstrate(FullIndexSet, HasParentIndexSet);
		EndIndexSet	  = SetSubstrate(FullIndexSet, HasChildIndexSet);

		//
		int i = 0;
		for ( auto ChildList : PointChildIndexList )
		{
			if ( ChildList.size() == 1 )
			{
				NewStartIndexSet.insert(i);
			}
			i++;
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
		std::cout << std::endl;
		Viewer.resetCameraViewpoint("MainPC");
		// Viewer.spin();
		FindAllLinks(NewStartIndexSet);
		// MergeLinks(StartIndexSet);

		// TPointArrangementViewer<PointType> Viewer;
		// Viewer.AddPointCloud(InPC, Index);
		// Viewer.spin();
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

			std::vector<int> CurrentLink		 = { StartIndex };
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
				// if ( NextSearchList.empty() )
				if ( NextSearchList.size() == 1 )
				{
					PrintLink(CurrentLink, "Cur  : ");
					PrintLink(SearchingIndexStack, "Stack: ");
					PrintLink(CurrentLinkRollBack, "Rollb: ");
					std::cout << std::endl;

					if ( CurrentLink.size() > LongestLink.size() )
					{
						LongestLink = CurrentLink;
					}

					{
						// PCIDX_Ptr PointIndexs = PCIDX_Ptr(new PCIDX);
						// PointIndexs->indices  = CurrentLink;

						// TPointArrangementViewer<PointType> Viewer;
						// Viewer.AddPointCloud(OperatingPC, PointIndexs);
						// Viewer.setWindowName("Cur");
						// Viewer.resetCamera();
						// Viewer.close();
						// Viewer.spin();
					}

					// Rollback
					const int& RollbackIndex = CurrentLinkRollBack.front();
					CurrentLink.erase(CurrentLink.begin() + RollbackIndex + 1, CurrentLink.end());
					// std::cout << "   >>>> Rolling back to link [" << CurrentLink.front() << " -> ... -> " << CurrentLink.back() << ": ";
					// for ( int idx : CurrentLink )
					// {
					// 	std::cout << idx << " - ";
					// }
					// std::cout << std::endl;
					// std::cout << std::endl;
					{
						// PCIDX_Ptr PointIndexs = PCIDX_Ptr(new PCIDX);
						// PointIndexs->indices  = CurrentLink;

						// TPointArrangementViewer<PointType> Viewer;
						// Viewer.AddPointCloud(OperatingPC, PointIndexs);
						// Viewer.setWindowName("Rollback");
						// Viewer.resetCamera();
						// Viewer.close();
						// Viewer.spin();
					}
					continue;
				}

				int i = 0;
				for (int PendingCheckingIndex : NextSearchList)
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
			// std::cout << "=============================================" << std::endl;
			// std::cout << std::endl;
			// std::cout << std::endl;
			// std::cout << std::endl;
		}

		std::cout << "Longest link : ";
		for ( int idx : LongestLink )
		{
			std::cout << idx << " - ";
		}
		std::cout << std::endl;

		PCIDX_Ptr PointIndexs = PCIDX_Ptr(new PCIDX);
		PointIndexs->indices  = LongestLink;

		TPointArrangementViewer<PointType> Viewer;
		Viewer.AddPointCloud(OperatingPC, PointIndexs);
		Viewer.spin();
	}

	void PrintLink(const std::vector<int>& InLink, const std::string& InPromptMsg)
	{
		std::cout << InPromptMsg;
		// std::cout << "Find link [" << CurrentLink.front() << " -> ... -> " << CurrentLink.back() << ": ";
		for ( int idx : InLink )
		{
			std::cout << idx << " - ";
		}
		std::cout << std::endl;
	}

	template<typename SetDataType>
	std::set<SetDataType> SetIntersection(const std::set<SetDataType>& InSetA, const std::set<SetDataType>& InSetB)
	{
		std::set<SetDataType> ReturnSet;
		std::set_intersection(InSetA.begin(), InSetA.end(), InSetB.begin(), InSetB.end(), std::inserter(ReturnSet, ReturnSet.begin()));
		return ReturnSet;
	}

	template<typename SetDataType>
	std::set<SetDataType> SetDiff(const std::set<SetDataType>& InSetA, const std::set<SetDataType>& InSetB)
	{
		std::set<SetDataType> ReturnSet;
		std::set_difference(InSetA.begin(), InSetA.end(), InSetB.begin(), InSetB.end(), std::inserter(ReturnSet, ReturnSet.begin()));
		return ReturnSet;
	}

	template<typename SetDataType>
	std::set<SetDataType> SetSubstrate(const std::set<SetDataType>& InSetA, const std::set<SetDataType>& InSetB)
	{
		std::set<SetDataType> ReturnSet, IntermediaSet;
		std::set_intersection(InSetA.begin(), InSetA.end(), InSetB.begin(), InSetB.end(), std::inserter(IntermediaSet, IntermediaSet.begin()));
		std::set_difference(InSetA.begin(), InSetA.end(), IntermediaSet.begin(), IntermediaSet.end(), std::inserter(ReturnSet, ReturnSet.begin()));
		return ReturnSet;
	}

	int MergeLinks(const std::set<int>& InStartIndexSet)
	{
		std::vector<int> CurrentLink;
		std::vector<std::vector<int>> LinkList;

		// Find all link by link start index.
		for ( int StartIndex : InStartIndexSet )
		{
			if ( CurrentLink.size() > 0 )
			{
				LinkList.push_back(CurrentLink);
			}
			// Clear for new searching.
			CurrentLink.clear();

			bool bStartIndexInSearchedLinkList = false;
			for ( auto Link : LinkList )
			{
				if ( std::find(Link.begin(), Link.end(), StartIndex) != CurrentLink.end() )
				{
					bStartIndexInSearchedLinkList = true;
					break;
				}
			}
			if ( bStartIndexInSearchedLinkList )
			{
				continue;
			}

			//
			CurrentLink.push_back(StartIndex);
			// Use stack structure to find a link.
			// Push the child index list into stack
			std::vector<int> SearchingIndexStack = PointChildIndexList[StartIndex];

			while ( !SearchingIndexStack.empty() )
			{
				const int CurrentIndex = SearchingIndexStack[0];
				SearchingIndexStack.erase(SearchingIndexStack.begin());

				// SearchingIndexStack is empty means [] is other child index link to searching root node.
				// Set [CurrentLink] reverse to use push_back() to append other index from searching root node.
				/**
				 * @brief
				 * [0 - 1 - 2 - 3]
				 * [0 - 4 - 5 - 6]
				 * [3 - 2 - 1 - 0 - 4 - 5 - 6]
				 *
				 */
				if ( SearchingIndexStack.empty() )
				{
					std::reverse(CurrentLink.begin(), CurrentLink.end());
					// SearchingIndexStack
				}

				if ( std::find(CurrentLink.begin(), CurrentLink.end(), CurrentIndex) != CurrentLink.end() )
				{
					continue;
				}

				CurrentLink.push_back(CurrentIndex);

				const std::vector<int>& NextSearchList = PointChildIndexList[CurrentIndex];
				// if ( NextSearchList.empty() )
				// if ( NextSearchList.size() == 1 )
				// {
				// 	LinkList.push_back(CurrentLink);
				// 	CurrentLink.clear();
				// 	CurrentLink.push_back(StartIndex);
				// 	continue;
				// }
				SearchingIndexStack.insert(SearchingIndexStack.begin(), NextSearchList.begin(), NextSearchList.end());
			}
		}

		std::cout << std::endl << std::endl << "Exist Links: " << std::endl;
		int i = 0;
		for ( auto Link : LinkList )
		{
			std::cout << "Link " << i++ << " : ";
			for ( int idx : Link )
			{
				std::cout << idx << " - ";
			}
			std::cout << std::endl;
		}
		return 0;

		// Merge each link to one long link.
		// std::vector<int> EachLinkStEd;
		std::map<int, std::vector<int>> MapStEdIndexToLinkIndex; // Map a Start or End index to it's link index

		i = 0;
		for ( auto Link : LinkList )
		{
			const bool bStExist = MapStEdIndexToLinkIndex.find(Link.front()) != MapStEdIndexToLinkIndex.end();
			const bool bEdExist = MapStEdIndexToLinkIndex.find(Link.back()) != MapStEdIndexToLinkIndex.end();
			if ( bStExist )
			{
				MapStEdIndexToLinkIndex.at(Link.front()).push_back(i);
			}
			else
			{
				MapStEdIndexToLinkIndex.insert_or_assign(Link.front(), std::vector<int>{ i });
			}
			if ( bEdExist )
			{
				MapStEdIndexToLinkIndex.at(Link.back()).push_back(i);
			}
			else
			{
				MapStEdIndexToLinkIndex.insert_or_assign(Link.back(), std::vector<int>{ i });
			}

			i++;
		}

		std::vector<int> LongestLink = LinkList[0];
		std::vector<bool> CheckedLinkResultList;
		CheckedLinkResultList.assign(LinkList.size(), false);
		CheckedLinkResultList[0] = true;

		do
		{
			const int& EndIndex								= LongestLink.back();
			const std::vector<int>& EdCheckingLinkIndexList = MapStEdIndexToLinkIndex[EndIndex];
			for ( int CheckingIndex : EdCheckingLinkIndexList )
			{
				if ( CheckedLinkResultList[CheckingIndex] )
				{
					continue;
				}

				const std::vector<int>& MergeingLink = LinkList[CheckingIndex];
				if ( EndIndex == MergeingLink.front() )
				{
					LongestLink.insert(LongestLink.end(), MergeingLink.begin() + 1, MergeingLink.end());
				}
				else
				{
					LongestLink.insert(LongestLink.end(), MergeingLink.rbegin() + 1, MergeingLink.rend());
				}
				CheckedLinkResultList[CheckingIndex] = true;
				break;
			}

			if ( (EdCheckingLinkIndexList.size() == 1) )
			{
				break;
			}

		} while ( true );

		do
		{
			const int& StartIndex							= LongestLink.front();
			const std::vector<int>& StCheckingLinkIndexList = MapStEdIndexToLinkIndex[StartIndex];
			for ( int CheckingIndex : StCheckingLinkIndexList )
			{
				if ( CheckedLinkResultList[CheckingIndex] )
				{
					continue;
				}

				const std::vector<int>& MergeingLink = LinkList[CheckingIndex];
				if ( StartIndex == MergeingLink.front() )
				{
					LongestLink.insert(LongestLink.begin(), MergeingLink.rbegin(), MergeingLink.rend() - 1);
				}
				else
				{
					LongestLink.insert(LongestLink.begin(), MergeingLink.begin(), MergeingLink.end() - 1);
				}
				CheckedLinkResultList[CheckingIndex] = true;
				break;
			}

			if ( (StCheckingLinkIndexList.size() == 1) )
			{
				break;
			}
		} while ( true );

		std::cout << std::endl
				  << std::endl
				  << "LongestLink "
				  << " : ";
		for ( int idx : LongestLink )
		{
			std::cout << idx << " - ";
		}
		std::cout << std::endl;

		return 0;
	}

	std::vector<std::vector<int>> PointChildIndexList;
};

} // namespace App
} // namespace PCL_Helper

#endif /* B376567C_A3E4_46E0_BF1E_31068C71C010 */


#endif /* AB60891A_D5E1_49F2_9CB6_53920C038FBA */
