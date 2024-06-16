#ifndef AD059A6E_C55A_48E7_BA35_B7B8AA5D5793
#define AD059A6E_C55A_48E7_BA35_B7B8AA5D5793

#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/surface/gp3.h>

#include "PCL_TypeAlias.hpp"

namespace PCL_Helper {

	// PlyMesh_Ptr CreatePlyMesh(PCXYZ_Ptr Data)
	// {
	// 	PlyMesh_Ptr Mesh(new PlyMesh);

	// 	// Normal estimation*
	// 	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	// 	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	// 	tree->setInputCloud (Data);
	// 	n.setInputCloud (Data);
	// 	n.setSearchMethod (tree);
	// 	n.setKSearch (20);
	// 	n.compute (*normals);
	// 	//* normals should not contain the point normals + surface curvatures

	// 	// Concatenate the XYZ and normal fields*
	// 	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	// 	pcl::concatenateFields (*Data, *normals, *cloud_with_normals);
	// 	//* cloud_with_normals = cloud + normals

	// 	// Create search tree*
	// 	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	// 	tree2->setInputCloud (cloud_with_normals);

	// 	// Initialize objects
	// 	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// 	PlyMesh triangles;

	// 	// Set the maximum distance between connected points (maximum edge length)
	// 	gp3.setSearchRadius (0.025);

	// 	// Set typical values for the parameters
	// 	gp3.setMu (2.5);
	// 	gp3.setMaximumNearestNeighbors (100);
	// 	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	// 	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	// 	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	// 	gp3.setNormalConsistency(false);

	// 	// Get result
	// 	gp3.setInputCloud (cloud_with_normals);
	// 	gp3.setSearchMethod (tree2);
	// 	gp3.reconstruct (*Mesh);

	// 	return Mesh;
	// }

}

#endif /* AD059A6E_C55A_48E7_BA35_B7B8AA5D5793 */
