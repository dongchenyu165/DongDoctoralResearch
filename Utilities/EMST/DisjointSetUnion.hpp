#pragma once
#include <vector>

/**
 * Disjoint-set-union data structure
 */
class DisjointSetUnion
{

public:

	explicit DisjointSetUnion(std::size_t InSize = 0);

	std::size_t GetRootParent(std::size_t InElement) const;
	bool bIsInSameRootParent(std::size_t InA, std::size_t InB) const;
	bool Merge(std::size_t InA, std::size_t InB);

	void Reset(std::size_t InNewSize);

private:

	// Storage parent indexes of each element
	mutable std::vector<std::size_t> Parent;
	std::vector<std::size_t> rank;
};