#ifndef F65C95A4_1C20_4F0F_A817_E8AD32B316FE
#define F65C95A4_1C20_4F0F_A817_E8AD32B316FE

#include <string>
#include <cstdint>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <nlohmann/json.hpp>

#include <Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp>
#include <Utilities/spdlog/LogConfig.hpp>
#include <GlobalVars.hpp>

namespace PCL_Helper {

/**
 * @brief The settings for Euclidean Clustering.
 */
typedef struct EuclideanClusterSettingStruct
{
	float ClusterTolerance	= 0.01f;  // The distance threashold between two clusters.
	uint32_t MinClusterSize = 1;	  // Min number of points in result clusters.
	uint32_t MaxClusterSize = 4e9;	  // Max number of points in result clusters. The number of points$p$ in final result clusters is [MinClusterSize
									  // \f$<p<\f$ MaxClusterSize].
	bool ClusterWithoutZAxis = false; // 
	
	// Json serilizer def:
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(EuclideanClusterSettingStruct, ClusterTolerance, MinClusterSize, MaxClusterSize, ClusterWithoutZAxis);

	// to string
	std::string ToString()
	{
		return 	"Cluster Tolerance: " + std::to_string(ClusterTolerance) + "; " + 
				"Cluster Size Min: " + std::to_string(MinClusterSize) + "; " + 
				"Cluster Size Max: " + std::to_string(MaxClusterSize) ;
	}
} EC_Setting;


/**
 * @brief Clustering using euclidean distance.
 *
 * @param Source Oringin point cloud.
 * @param ClusterList The std::vector of result of clustering.
 * @param Setting Settings of euclidean cluster.
 * @param ClusterWithoutZAxis If TRUE, Project points to XoY plane when clustering.
 * @note We can not get complete point cloud of object on the table. If we just use 3D clustering, it may be separate the object to many pieces. To
 * avoid this issue, you need to use [ClusterWithoutZAxis] with [true].
 * @return int Number of clusters.
 */
template<typename PointType>
int ExtractEuclideanCluster (PCPTR<PointType> InSource, std::vector<PCPTR<PointType>>& OutClusterList, EC_Setting InSetting)
{
	static SPDLog::LoggerType Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_GlobalLogger", LogConfigJsonPath, {"PCL_GlobalLogger"});
	LOG_FUNC_ENTER(Logger, debug, 0);

	Logger->debug("Euclidean Clustering with param: {}; Cluster Without Z-Axis: {}", InSetting.ToString(), (InSetting.ClusterWithoutZAxis ? "True" : "False"));
	Logger->debug("Input point cloud has [{}] points.", InSource->size());

	PCPTR<PointType> NoZ;
	PCPTR<PointType> CalculationInputPC = InSource;
	if ( InSetting.ClusterWithoutZAxis )
	{
		NoZ = InSource->makeShared ();					// Copy input point cloud.
		InSource->getMatrixXfMap ().row (2).setZero (); // Set Z field to 0.
		CalculationInputPC = NoZ;						// set z-zeroed point cloud to calculation cluster.
	}

	// Creating the KdTree object for the search method of the extraction
	typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	tree->setInputCloud (CalculationInputPC);

	std::vector<PCIDX> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance (InSetting.ClusterTolerance);
	ec.setMinClusterSize (InSetting.MinClusterSize);
	ec.setMaxClusterSize (InSetting.MaxClusterSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (CalculationInputPC);
	ec.extract (cluster_indices);

	for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it )
	{
		PCPTR<PointType> CloudCluster (new PC<PointType>);
		for ( std::vector<int>::const_iterator PointID_It = it->indices.begin (); PointID_It != it->indices.end (); ++PointID_It )
		{
			CloudCluster->push_back (InSource->points[*PointID_It]); //*
		}
		OutClusterList.push_back (CloudCluster);
		Logger->trace("\t Find a cluster with [{}] points.", CloudCluster->size());
	}
	Logger->debug ("Total clustors [{}] point.", cluster_indices.size ());

	LOG_FUNC_EXIT(Logger, debug, 0);
	return cluster_indices.size ();
}

}

#endif /* F65C95A4_1C20_4F0F_A817_E8AD32B316FE */
