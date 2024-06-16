#include "DisjointSetUnion.hpp"

DisjointSetUnion::DisjointSetUnion(std::size_t InSize) { Reset(InSize); }

std::size_t DisjointSetUnion::GetRootParent(std::size_t InElement) const
{
	return Parent[InElement] == InElement ? InElement : Parent[InElement] = GetRootParent(Parent[InElement]);
}

bool DisjointSetUnion::bIsInSameRootParent(std::size_t InA, std::size_t InB) const { return GetRootParent(InA) == GetRootParent(InB); }

void DisjointSetUnion::Reset(std::size_t InNewSize)
{
	Parent.resize(InNewSize);
	rank.assign(InNewSize, 0);
	for ( std::size_t i = 0; i < InNewSize; i++ )
	{
		Parent[i] = i;
	}
}

bool DisjointSetUnion::Merge(std::size_t InA, std::size_t InB)
{
	std::size_t ParentA = GetRootParent(InA);
	std::size_t ParentB = GetRootParent(InB);
	if ( ParentA == ParentB )
	{
		return false;
	}
	if ( rank[ParentA] > rank[ParentB] )
	{
		std::swap(ParentA, ParentB);
	}
	if ( rank[ParentA] == rank[ParentB] )
	{
		rank[ParentB]++;
	}
	Parent[ParentA] = ParentB;
	return true;
}
