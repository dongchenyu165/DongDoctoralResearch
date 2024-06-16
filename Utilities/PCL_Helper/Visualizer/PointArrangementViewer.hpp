#ifndef EE772636_5E95_4F83_A3A2_C9E78A4C585B
#define EE772636_5E95_4F83_A3A2_C9E78A4C585B

// #include <cstddef>
#include <pcl/visualization/pcl_visualizer.h>

#include "../Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Visualizer/DebugViewer.hpp"

namespace PCL_Helper
{

template<typename PointType>
class TPointArrangementViewer : public TDebugViewer<PointType>
{
	using Super            = TDebugViewer<PointType>;
	using ColorHandlerType = pcl::visualization::PointCloudColorHandlerCustom<PointType>;

public:
	TPointArrangementViewer() = delete;

	TPointArrangementViewer(const std::string& InID,
		const std::string& InCategoryName,
		const std::string& InWindowName,
		// const typename Super::ELevel& InNewLevel = Super::ELevel::DEBUG,
		const bool bInEnable = true)
		: Super(InID, InCategoryName, InWindowName, bInEnable)
	{
	}

	void AddPointCloud(const PCPTR<PointType>& InPC)
	{
		static ColorHandlerType CH(InPC, 255, 0, 0);
		PointType TempPt;
		const int PointNum = InPC->size();
		for ( int i = 0; i < PointNum; i++ )
		{
			const int& Index     = (i);
			const int& NextIndex = ((i + 1) % PointNum);

			this->addLine((*InPC)[Index], (*InPC)[NextIndex], 0.5, .7, .1, "Test_Line_" + std::to_string(i));
			TempPt.getVector3fMap() = ((*InPC)[Index].getVector3fMap() + (*InPC)[NextIndex].getVector3fMap()) / 2;
			this->addText3D(std::to_string(i + 1), TempPt, 0.001);

			std::cout << "i: " << i << " " << std::endl;
		}
		this->setBackgroundColor(0.1, 0.1, 0.1);
		this->addPointCloud(InPC, CH, "PointCloud_0");
	}

	void AddPointCloud(const PCPTR<PointType>& InPC, PCIDX_Ptr InPCIndex, bool bInIsOpenCurve = true)
	{
		static ColorHandlerType CH(InPC, 255, 0, 0);
		PointType TempPt;
		const int POINT_NUM = InPCIndex->indices.size();

		this->addSphere((*InPC)[0], 0.0005, "ST_SPHERE_" + std::to_string(0));

		for ( int i = 0; i < POINT_NUM - bInIsOpenCurve; i++ )
		{
			const int& Index     = (InPCIndex->indices[i]);
			const int& NextIndex = (InPCIndex->indices[(i + 1) % POINT_NUM]);

			this->addLine((*InPC)[Index], (*InPC)[NextIndex], 0.5, .7, .1, "Test_Line_" + std::to_string(i));

			TempPt.getVector3fMap() = ((*InPC)[Index].getVector3fMap() + (*InPC)[NextIndex].getVector3fMap()) / 2;
			this->addText3D(std::to_string(i + 1), TempPt, 0.0001);
		}
		this->setBackgroundColor(0.1, 0.1, 0.1);
		this->addPointCloud(InPC, CH, "PointCloud_0");
	}

	void AddSingleEdge(const PCPTR<PointType>& InRefPC, const size_t& InA, const size_t& InB)
	{
		this->addText3D(std::to_string(InA), (*InRefPC)[InA], 0.0001);
		this->addText3D(std::to_string(InB), (*InRefPC)[InB], 0.0001);
		this->addLine((*InRefPC)[InA], (*InRefPC)[InB], 0.5, .7, .1,
			"AddSingleEdge_Line_" + std::to_string(InA) + "_" + std::to_string(InB));
	}
};

} // namespace PCL_Helper

#define DEBUG_SHOW_POLYGON(VIEWER_NAME, CATEGORY_NAME, POLYGON_PC)                                               \
	{                                                                                                            \
		constexpr const char CODE_PATH[] = __FILE__ ":{}";                                                       \
		const auto CodePath              = spdlog ::fmt_lib ::format(CODE_PATH, __LINE__);                       \
		if ( auto ViewerPtr =                                                                                    \
				 TDebugViewerManager<Types ::CalcPoint>::GetOrMakeViewer(VIEWER_NAME, CATEGORY_NAME, CodePath) ) \
		{                                                                                                        \
			auto CastViewerPtr = std::dynamic_pointer_cast<TPointArrangementViewer<PointType>>(ViewerPtr);       \
			CastViewerPtr->Clear();                                                                              \
			CastViewerPtr->addCoordinateSystem(0.1);                                                             \
			CastViewerPtr->AddPointCloud(POLYGON_PC);                                                            \
			if ( ViewerPtr->wasStopped() )                                                                       \
			{                                                                                                    \
				ViewerPtr->createInteractor();                                                                   \
			}                                                                                                    \
			ViewerPtr->spin();                                                                                   \
		}                                                                                                        \
	}
#define DEBUG_SHOW_POLYGON_CUSTOM_IDX(VIEWER_NAME, CATEGORY_NAME, POLYGON_PC, INDEX)                             \
	{                                                                                                            \
		constexpr const char CODE_PATH[] = __FILE__ ":{}";                                                       \
		const auto CodePath              = spdlog ::fmt_lib ::format(CODE_PATH, __LINE__);                       \
		if ( auto ViewerPtr =                                                                                    \
				 TDebugViewerManager<Types ::CalcPoint>::GetOrMakeViewer(VIEWER_NAME, CATEGORY_NAME, CodePath) ) \
		{                                                                                                        \
			auto CastViewerPtr = std::dynamic_pointer_cast<TPointArrangementViewer<PointType>>(ViewerPtr);       \
			CastViewerPtr->Clear();                                                                              \
			CastViewerPtr->addCoordinateSystem(0.1);                                                             \
			CastViewerPtr->AddPointCloud(POLYGON_PC, INDEX);                                                     \
			if ( ViewerPtr->wasStopped() )                                                                       \
			{                                                                                                    \
				ViewerPtr->createInteractor();                                                                   \
			}                                                                                                    \
			ViewerPtr->spin();                                                                                   \
		}                                                                                                        \
	}

#endif /* EE772636_5E95_4F83_A3A2_C9E78A4C585B */
