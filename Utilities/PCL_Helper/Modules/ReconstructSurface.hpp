#ifndef E2C633F2_4AF3_42D9_A020_8BBFA0895FB6
#define E2C633F2_4AF3_42D9_A020_8BBFA0895FB6

#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include "Utilities/PCL_Helper/Calculations/FlipNormal.hpp"


namespace PCL_Helper {

template<typename PointType>
void ReconstructSurface (PCPTR<PointType> InPC, float InSearchRadius)
{
	if ( (*InPC).size () == 0 )
	{
		return;
	}

	// Create a KD-Tree
	typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

	// Init reconstruction object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<PointType, PointType> mls;

	// Set parameters
	mls.setInputCloud (InPC->makeShared ());
	mls.setComputeNormals (true);
	mls.setPolynomialOrder (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (InSearchRadius);

	// Reconstruct
	mls.process (*InPC);

	FlipNormal_ByViewpoint(InPC);
}

}

#endif /* E2C633F2_4AF3_42D9_A020_8BBFA0895FB6 */
