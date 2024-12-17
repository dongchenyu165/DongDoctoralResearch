#ifndef AD059A6E_C55A_48E7_BA35_B7B8AA5D5793
#define AD059A6E_C55A_48E7_BA35_B7B8AA5D5793

#include <Eigen/Core>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/surface/gp3.h>

#include "ToEigenMap.hpp"
#include "PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Basic/PointCloudConverter.hpp"

/**
 * @namespace PCL_Helper
 * @brief Helper namespace containing functions for Point Cloud Library operations
 */


namespace PCL_Helper {

	
	/**
	* @brief Creates a polygon mesh from point cloud data using Json configuration parameters
				The interface for operating JSON object is [nlohmann::json] library.
	* @tparam PointType The point type of the input cloud (typically pcl::PointXYZ)
	* @tparam JSONType The JSON configuration type
	* @param InData Input point cloud data
	* @param InJsonConfig JSON configuration containing mesh generation parameters:
	*        {
				"normal_estimation": {
					"k_search": 20,
					"description1": "Number of nearby points used in normal estimation",
					"effect1": "Larger values result in smoother normal estimation but longer computation time",
					"recommended_range1": "10-50"
				},
				"triangulation": {
					"search_radius": 0.025,
					"description1": "Maximum distance between connected points during triangulation",
					"effect1": "Larger values connect more distant points, may cause inaccurate connections",
					"recommended_range1": "0.01-0.05",

					"mu": 2.5,
					"description2": "Ratio of maximum to minimum edge length",
					"effect2": "Controls triangle shape uniformity",
					"recommended_range2": "2.0-3.0",

					"max_nearest_neighbors": 100,
					"description3": "Maximum number of nearest neighbors to search",
					"effect3": "Affects reconstruction accuracy and computation time",
					"recommended_range3": "50-200",

					"normal_consistency": false,
					"description4": "Whether to check normal consistency",
					"effect4": "true can avoid incorrect triangle connections but may lead to holes",
					"recommended_range4": "true/false"
				},
				"angle_constraints": {
					"max_surface_angle": 45.0,
					"description1": "Maximum angle between adjacent triangle surface normals (degrees)",
					"effect1": "Controls surface smoothness, smaller values produce smoother surfaces",
					"recommended_range1": "30-60",

					"min_angle": 10.0,
					"description2": "Minimum value for triangle interior angles (degrees)",
					"effect2": "Prevents generation of overly elongated triangles",
					"recommended_range2": "5-15",

					"max_angle": 120.0,
					"description3": "Maximum value for triangle interior angles (degrees)",
					"effect3": "Limits triangle shape, prevents very obtuse angles",
					"recommended_range3": "110-130"
				}
			}
	*        
	* @return PlyMesh_Ptr Pointer to the generated polygon mesh
	*/
	template<typename PointType, typename JSONType, typename LoggerPtrType>
	PlyMesh_Ptr CreatePlyMesh(PCPTR<PointType> InData, const JSONType& InJsonConfig, LoggerPtrType Logger)
	{
		PCL_Helper::PCXYZN_Ptr CloudWithNormals;
		PlyMesh_Ptr OutMesh(new PlyMesh);

		// Check if the input data has normals
		constexpr bool bHasNormalField = pcl::traits::has_normal<PointType>::value;
		bool bHasNormalData            = false;
		if constexpr ( bHasNormalField )
		{
			const auto& DataNormalMat = TPCL_EigenMapper<PointType>::GetNormalEigenMatrixMap(InData);
			// Check if point cloud has valid normal data:
			// 1. Verify length of normal vectors are not zero
			// 2. Ensure no NaN values exist in any normal vector component
			bHasNormalData = !(DataNormalMat.rowwise().norm().isApproxToConstant(0, 1e-4) ||
							   DataNormalMat.array().isNaN().any());
		}

		if ( bHasNormalData )
		{
			CloudWithNormals = PCL_Helper::ConvertPointCloud<PCL_Helper::PointXYZN, EConvertRGBField::Const,
				EConvertNormalField::Copy>(InData);
		}
		else
		{
			CloudWithNormals = PCL_Helper::ConvertPointCloud<PCL_Helper::PointXYZN, EConvertRGBField::Const,
				EConvertNormalField::Estimate>(InData, { 255, 0, 0 }, { 1, 0, 0 },
				InJsonConfig["normal_estimation"]["k_search"]);
		}

		// Create search tree
		pcl::search::KdTree<pcl::PointNormal>::Ptr SearchTree2(new pcl::search::KdTree<pcl::PointNormal>);
		SearchTree2->setInputCloud(CloudWithNormals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> Triangulator;
		PlyMesh Triangles;

		// Set the maximum distance between connected points (maximum edge length)
		Triangulator.setSearchRadius(InJsonConfig["triangulation"]["search_radius"]);

		// Set typical values for the parameters
		Triangulator.setMu(InJsonConfig["triangulation"]["mu"]);
		Triangulator.setMaximumNearestNeighbors(InJsonConfig["triangulation"]["max_nearest_neighbors"]);
		Triangulator.setMaximumSurfaceAngle(InJsonConfig["angle_constraints"]["max_surface_angle"] * M_PI/180); // 45 degrees
		Triangulator.setMinimumAngle(InJsonConfig["angle_constraints"]["min_angle"] * M_PI/180); // 10 degrees
		Triangulator.setMaximumAngle(InJsonConfig["angle_constraints"]["max_angle"] * M_PI/180); // 120 degrees
		Triangulator.setNormalConsistency(InJsonConfig["triangulation"]["normal_consistency"]);

		// Get result
		Triangulator.setInputCloud(CloudWithNormals);
		Triangulator.setSearchMethod(SearchTree2);
		Triangulator.reconstruct(*OutMesh);

		return OutMesh;
	}


	/**
	* @brief Creates a polygon mesh from point cloud data using default parameters
	* @tparam PointType The point type of the input cloud (typically pcl::PointXYZ)
	* @param InData Input point cloud data
	* @return PlyMesh_Ptr Pointer to the generated polygon mesh
	* @note Default parameters:
	*       - k_search: 20
	*       - search_radius: 0.025
	*       - mu: 2.5
	*       - max_nearest_neighbors: 100
	*       - max_surface_angle: 45 degrees
	*       - min_angle: 10 degrees
	*       - max_angle: 120 degrees
	*       - normal_consistency: false
	*/
	template<typename PointType>
	PlyMesh_Ptr CreatePlyMesh(PCPTR<PointType> InData)
	{
		PlyMesh_Ptr OutMesh(new PlyMesh);

		// Normal estimation
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> NormalEstimator;
		pcl::PointCloud<pcl::Normal>::Ptr Normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr SearchTree(new pcl::search::KdTree<pcl::PointXYZ>);
		SearchTree->setInputCloud(InData);
		NormalEstimator.setInputCloud(InData);
		NormalEstimator.setSearchMethod(SearchTree);
		NormalEstimator.setKSearch(20);
		NormalEstimator.compute(*Normals);

		// Concatenate the XYZ and normal fields
		pcl::PointCloud<pcl::PointNormal>::Ptr CloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*InData, *Normals, *CloudWithNormals);

		// Create search tree
		pcl::search::KdTree<pcl::PointNormal>::Ptr SearchTree2(new pcl::search::KdTree<pcl::PointNormal>);
		SearchTree2->setInputCloud(CloudWithNormals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> Triangulator;
		PlyMesh Triangles;

		// Set the maximum distance between connected points (maximum edge length)
		Triangulator.setSearchRadius(0.025);

		// Set typical values for the parameters
		Triangulator.setMu(2.5);
		Triangulator.setMaximumNearestNeighbors(100);
		Triangulator.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		Triangulator.setMinimumAngle(M_PI/18); // 10 degrees
		Triangulator.setMaximumAngle(2*M_PI/3); // 120 degrees
		Triangulator.setNormalConsistency(false);

		// Get result
		Triangulator.setInputCloud(CloudWithNormals);
		Triangulator.setSearchMethod(SearchTree2);
		Triangulator.reconstruct(*OutMesh);

		return OutMesh;
	}

}

#endif /* AD059A6E_C55A_48E7_BA35_B7B8AA5D5793 */
