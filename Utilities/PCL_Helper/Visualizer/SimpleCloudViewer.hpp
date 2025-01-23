#ifndef B16ED593_C249_4151_BFA1_7D5EFA540194
#define B16ED593_C249_4151_BFA1_7D5EFA540194

// #include <cstddef>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "../Basic/PCL_TypeAlias.hpp"
// #include "GlobalVars.hpp"
#include "pcl/common/io.h"
// #include <Utilities/spdlog/LogConfig.hpp>

namespace PCL_Helper
{

// enum class ViewerColorPolicy
// {
// 	AutoSegmentColor,  // Rainbow color apply to each point cloud.
// 	SelfColor,  // Use each point cloud self's color (if exists RGB field, else change to [AutoSegmentColor]).
// };

inline pcl::PointXYZ HSV2RGB(
	double H,
	double S,
	double V,
	double RangeH = 360,
	double RangeS = 255,
	double RangeV = 255,
	double RangeR = 255,
	double RangeG = 255,
	double RangeB = 255
)
{
	pcl::PointXYZ Res;
	int Hi = 0.0;
	if ( S == 0 )
	{
		Res.x = V;
		Res.y = V;
		Res.z = V;
	}
	else
	{
		H  /= 60.0;
		Hi = (int)H % 6;
	}

	double f = H - Hi;
	double a = V / RangeV * (1 - S / RangeS);
	double b = V / RangeV * (1 - S / RangeS * f);
	double c = V / RangeV * (1 - S / RangeS * (1 - f));

	switch ( Hi )
	{
	case 0:
		Res.x = V;
		Res.y = c * RangeG;
		Res.z = a * RangeB;
		break;
	case 1:
		Res.x = b * RangeR;
		Res.y = V;
		Res.z = a * RangeB;
		break;
	case 2:
		Res.x = a * RangeR;
		Res.y = V;
		Res.z = c * RangeB;
		break;
	case 3:
		Res.x = a * RangeR;
		Res.y = b * RangeG;
		Res.z = V;
		break;
	case 4:
		Res.x = c * RangeR;
		Res.y = a * RangeG;
		Res.z = V;
		break;
	case 5:
		Res.x = V;
		Res.y = a * RangeG;
		Res.z = b * RangeB;
		break;
	}
	return Res;
}

inline PointXYZ CreateRainbowColor(int InColorIndex, int FullCount, double S = 255, double V = 255)
{
	const double HueStep = 360.0 / static_cast<double>(FullCount);
	PointXYZ ColorInfo((float)0, (float)0, (float)0);
	ColorInfo = HSV2RGB(InColorIndex * HueStep, S, V);
	return ColorInfo;
}

inline std::vector<PointXYZ> CreateRainbowColorList(int InColorCount, double S = 255, double V = 255)
{
	std::vector<PointXYZ> ResColorList;
	for ( int i = 0; i < InColorCount; ++i )
	{
		PointXYZ ColorInfo((float)0, (float)0, (float)0);
		ColorInfo = HSV2RGB(i * (360.0 / InColorCount), S, V);
		ResColorList.push_back(ColorInfo);
	}
	return ResColorList;
}

inline std::vector<PointXYZ> CreateRainbowColorList_Offset(int InColorCount, double InH_Offset, double S = 255, double V = 255)
{
	std::vector<PointXYZ> ResColorList;
	for ( int i = 0; i < InColorCount; ++i )
	{
		PointXYZ ColorInfo((float)0, (float)0, (float)0);
		ColorInfo = HSV2RGB(((int)(i * (360 / InColorCount) + InH_Offset)) % 360, S, V);
		ResColorList.push_back(ColorInfo);
	}
	return ResColorList;
}

template<typename PointType>
class TSimpleCloudViewer : public pcl::visualization::PCLVisualizer
{
	using ColorHandlerType = pcl::visualization::PointCloudColorHandlerCustom<PointType>;

public:
	TSimpleCloudViewer(const std::string& InWindowName) : pcl::visualization::PCLVisualizer(InWindowName) { }


	void AddPointCloudList(const std::vector<PCPTR<PointType>>& InPointList, const std::string& InCloudName = "")
	{
		// auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_Vis", LogConfigJsonPath);

		auto ColorList = CreateRainbowColorList(InPointList.size());
		for ( int i = 0; i < InPointList.size(); i++ )
		{
			ColorHandlerType CH(InPointList[i], ColorList[i].x, ColorList[i].y, ColorList[i].z);
			// TODO: Fix template error
			// Logger->trace("SIMPLE VIEWER: ADD Point cloud with size: [{}]; RGB: [{}]", InPointList[i]->size(), ColorList[i].getVector3fMap());
			addPointCloud(InPointList[i], CH, InCloudName + "cloud_" + std::to_string(i));
		}
	}

	void AddPointCloudByIndexList(PCPTR<PointType> InPC, std::vector<PCIDX_Ptr> InIndexList, const std::string& InCloudName = "")
	{
		// auto Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_Vis", LogConfigJsonPath);
		PCPTR<PointType> AddingPC(new PC<PointType>);
		auto ColorList = CreateRainbowColorList(InIndexList.size());

		for ( int i = 0; i < InIndexList.size(); i++ )
		{
			pcl::copyPointCloud(*InPC, *(InIndexList[i]), *AddingPC);
			// Logger->trace("SIMPLE VIEWER: COPY Point cloud with size: [{}]; RGB: [{}]", AddingPC->size(), ColorList[i].getVector3fMap());

			ColorHandlerType CH(AddingPC, ColorList[i].x, ColorList[i].y, ColorList[i].z);
			addPointCloud(AddingPC, CH, InCloudName + "cloud_" + std::to_string(i));
		}
	}

	void AddNormal(PCPTR<PointType> InPC, float InShowingPercent = 0.3, float InNormalLength = 0.01)
	{
		int NormalShowingStep = InPC->size() / (InShowingPercent * InPC->size());
		addPointCloudNormals<PointType>(InPC, NormalShowingStep == 0 ? 1 : NormalShowingStep, InNormalLength, "Normal");
	}

	void AddNormal(PCPTR<PointType> InPC, PCN_Ptr InNormalData, float InShowingPercent = 0.3, float InNormalLength = 0.01)
	{
		int NormalShowingStep = InPC->size() / (InShowingPercent * InPC->size());
		addPointCloudNormals<PointType, pcl::PointNormal>(InPC, NormalShowingStep == 0 ? 1 : NormalShowingStep, InNormalLength, "Normal");
	}

	void Clear()
	{
		this->removeAllPointClouds();
		this->removeAllShapes();
	}

protected:
	
	// /** \brief Add a PointXYZ Point Cloud to screen.
	// * \param[in] cloud the input point cloud dataset
	// * \param[in] id the point cloud object id (default: cloud)
	// * \param[in] viewport the view port where the Point Cloud should be added (default: all)
	// */
	// inline bool
	// addPointCloud (const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud,
	// 				const std::string &id = "cloud", int viewport = 0)
	// {
	// 	return (addPointCloud<pcl::PointXYZRGBNormal> (cloud, id, viewport));
	// }
};

} // namespace PCL_Helper

#endif /* B16ED593_C249_4151_BFA1_7D5EFA540194 */
