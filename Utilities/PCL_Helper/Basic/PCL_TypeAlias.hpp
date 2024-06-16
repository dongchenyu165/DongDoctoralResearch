#ifndef DC460ABE_F876_424C_B2D7_D749EDACC18D
#define DC460ABE_F876_424C_B2D7_D749EDACC18D

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h> // For type: [pcl::PointIndices]
#include <pcl/PolygonMesh.h>

namespace PCL_Helper
{

template<typename PointType>
using PC = pcl::PointCloud<PointType>;
template<typename PointType>
using PC_List = std::vector<pcl::PointCloud<PointType>>;
template<typename PointType>
using PCPTR = std::shared_ptr<pcl::PointCloud<PointType>>;
template<typename PointType>
using PCPTR_List = std::vector<std::shared_ptr<pcl::PointCloud<PointType>>>;

using PointXY      = pcl::PointXY;
using PointXYZ     = pcl::PointXYZ;
using PointXYZN    = pcl::PointNormal;
using PointXYZRGB  = pcl::PointXYZRGB;
using PointXYZRGBN = pcl::PointXYZRGBNormal;

using PCXY     = PC<pcl::PointXY>;     // Point cloud of type: XY (x, y).
using PCXYZ    = PC<pcl::PointXYZ>;    // Point cloud of type: XYZ (x, y, z).
using PCXYZN   = PC<pcl::PointNormal>; // Point cloud of type: Normal (x, y, z, nx, ny, nz, curvature).
using PCXYZRGB = PC<pcl::PointXYZRGB>; // Point cloud of type: XYZRGB (x, y, z, r, g, b).
using PCXYZRGBN =
	PC<pcl::PointXYZRGBNormal>; // Point cloud of type: XYZRGBNormal (x, y, z, r, g, b, nx, ny, nz, curvature).

using PCXY_Ptr      = PCPTR<pcl::PointXY>;
using PCXYZ_Ptr     = PCPTR<pcl::PointXYZ>;
using PCXYZN_Ptr    = PCPTR<pcl::PointNormal>;
using PCXYZRGB_Ptr  = PCPTR<pcl::PointXYZRGB>;
using PCXYZRGBN_Ptr = PCPTR<pcl::PointXYZRGBNormal>;

using PCIDX     = pcl::PointIndices; // A set of point index of a point cloud.
using PCIDX_Ptr = pcl::PointIndices::Ptr;

using PCN     = pcl::PointCloud<pcl::Normal>; // Only the normal info of point cloud. (nx, ny, nz, curvature).
using PCN_Ptr = pcl::PointCloud<pcl::Normal>::Ptr;

using PlyMesh     = pcl::PolygonMesh;
using PlyMesh_Ptr = pcl::PolygonMesh::Ptr;

/**
 * @brief Make a PC smart-pointer variable.
 */
#define NEW_PC_PTR(VAR_NAME, POINT_TYPE) \
	PCL_Helper::PCPTR<POINT_TYPE> VAR_NAME = PCL_Helper::PCPTR<POINT_TYPE>(new PCL_Helper::PC<POINT_TYPE>)

/**
 * @brief Make a PC smart-pointer variable.
 */
#define NEW_PC_IDX_PTR(VAR_NAME) PCL_Helper::PCIDX_Ptr VAR_NAME = PCL_Helper::PCIDX_Ptr(new PCL_Helper::PCIDX)
} // namespace PCL_Helper

#endif /* DC460ABE_F876_424C_B2D7_D749EDACC18D */
