#ifndef CD78FBA2_C76D_480F_B630_10BBFBC8B8C5
#define CD78FBA2_C76D_480F_B630_10BBFBC8B8C5

#include "GlobalVars.hpp"
// #include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
// #include "Utilities/PCL_Helper/Visualizer/SimpleCloudViewer.hpp"
// #include "nlohmann/detail/macro_scope.hpp"
// #include "spdlog/common.h"
// #include <algorithm>
// #include <cstdint>
// #include <fstream>
#include <string>
#include <memory>
// #include <string_view>
#include <Utilities/spdlog/LogConfig.hpp>
#include <nlohmann/json.hpp>

#include "DebugViewer.hpp"
#include "PointArrangementViewer.hpp"


namespace PCL_Helper
{

struct DebugViewerConfig
{
	std::string ViewerCategory;
	std::string ViewerID;

	std::string WindowTitle;
	std::string Level;
	bool Enable;
	std::string CodePath;
	std::string Description;
	std::string ViewerClassStr;

	NLOHMANN_DEFINE_TYPE_INTRUSIVE(DebugViewerConfig,
		Enable,
		WindowTitle,
		Level,
		CodePath,
		Description,
		ViewerClassStr);
};

template<typename PointType>
class TDebugViewerManager
{
	using json = nlohmann::json;
public:
	/**
	 * @brief The priority is [Larger is lower].
	 *
	 */
	enum ELevel : int
	{
		TRACE = spdlog::level::trace,
		DEBUG = spdlog::level::debug,
		INFO  = spdlog::level::info,
		WARN  = spdlog::level::warn,
		ERROR = spdlog::level::err,
		FATAL = spdlog::level::critical,
		OFF   = spdlog::level::off,
	};

	// static const std::map<const std::string, ELevel> StringToLevelEnum;
	inline static const std::map<const std::string, ELevel> StringToLevelEnum = {
		{"TRACE",     ELevel::TRACE},
        { "DEBUG",    ELevel::DEBUG},
        { "INFO",     ELevel::INFO },
        { "WARN",     ELevel::WARN },
		{ "ERROR",    ELevel::ERROR},
        { "CRITICAL", ELevel::FATAL},
        { "OFF",      ELevel::OFF  }
	};

	using Super         = PCL_Helper::TSimpleCloudViewer<PointType>;
	using ViewerType    = TDebugViewer<PointType>;
	using ViewerPtrType = std::shared_ptr<ViewerType>;

/* -------------------------------------------------------------------------- */
/*                              Static Functions                              */
/* -------------------------------------------------------------------------- */
#pragma region Static Functions

	static ViewerPtrType GetOrMakeViewer(const std::string& InViewerID,
		const std::string& InViewerCategory,
		const std::string& InCodePath)
	{
		if ( !bIsInit )
		{
			bIsInit = true;
			__Initialize__();
		}

		LOG_FUNC_ENTER(Logger, debug, 1);
		SPDLog::Log_I(Logger, 1, "Start viewer: [{}].", InViewerID);
		SPDLog::Log_D(Logger, 1, "DEBUG VIEWER: Create or get viewer object by name: [{}].", InViewerID);

		json& ViewerConfigJsonObj    = AllViewerConfigJsonObj[InViewerCategory][InViewerID];
		DebugViewerConfig ConfigObj  = ViewerConfigJsonObj.get<DebugViewerConfig>();
		const bool& bIsViewerEnabled = ViewerConfigJsonObj["Enable"].template get_ref<const bool&>();
		const ELevel& Level          = StringToLevelEnum.at(ConfigObj.Level);
		const bool& bShowViewer      = bShouldViewerShow(InViewerID, Level, InViewerCategory, bIsViewerEnabled);
		if ( !bShowViewer )
		{
			SPDLog::Log_D(Logger, 1, "DEBUG VIEWER: Viewer: [{}] WILL HIDE.", InViewerID);
			LOG_FUNC_EXIT(Logger, debug, 1);
			return nullptr;
		}

		// ViewerPtrType ViewerPtr = Get(InViewerID);
		// if ( ViewerPtr )
		// {
		// 	SPDLog::Log_D(Logger, 1, "DEBUG VIEWER: [{}] is in [ViewerDict]", InViewerID);
		// 	LOG_FUNC_EXIT(Logger, debug, 1);
		// 	return ViewerPtr;
		// }

		// Make a new viewer.
		if ( ConfigObj.ViewerClassStr.empty() )
		{
			ViewerDict[InViewerID] = std::make_shared<TDebugViewer<PointType>>(InViewerID, InViewerCategory,
				ConfigObj.WindowTitle, bShowViewer);
			// ConfigObj.WindowTitle, Level, bShowViewer);
		}
		else if ( ConfigObj.ViewerClassStr == "PointArrangementViewer" )
		{
			ViewerDict[InViewerID] = std::static_pointer_cast<TDebugViewer<PointType>>(
				std::make_shared<TPointArrangementViewer<PointType>>(InViewerID, InViewerCategory,
					ConfigObj.WindowTitle, ConfigObj.Enable));
			// ConfigObj.WindowTitle, Level, ConfigObj.Enable));
		}

		// Storage viewer ID to json file.
		UpdateCodePath(InViewerID, InViewerCategory, InCodePath);

		SPDLog::Log_D(Logger, 1, "DEBUG VIEWER: Return created viewer [{}]; Addr: [{:#x}]", InViewerID,
			(uint64_t)(ViewerDict[InViewerID].get()));
		LOG_FUNC_EXIT(Logger, debug, 1);
		return ViewerDict[InViewerID];
	}

	/**
	 * @brief Set the global viewer's Level
	 *
	 * @param InNewLevel
	 */
	static void SetGlobalLevel(const ELevel InNewLevel) { GlobalLevel = InNewLevel; };

private:
	static bool IsInEnabledCategoryList(const std::string& InCategoryName)
	{
		return std::find(EnabledViewerCategoryList.begin(), EnabledViewerCategoryList.end(), InCategoryName) !=
		       EnabledViewerCategoryList.end();
	}

	static ViewerPtrType Get(const std::string& InViewerID)
	{
		auto FoundViewerPtr = ViewerDict.find(InViewerID);
		return (FoundViewerPtr == ViewerDict.end()) ? nullptr : FoundViewerPtr->second;
	}

	static void __Initialize__()
	{
		Logger = SPDLog::LoggerManager::GetOrMakeLoggerFromJsonPath("PCL_Vis", LogConfigJsonPath);
		__UpdateConfigJson_FromFile__(ViewerConfigJsonPath);
	}

	static json __UpdateConfigJson_FromFile__(const std::string& InJsonFilePath = ViewerConfigJsonPath)
	{
		LOG_FUNC_ENTER(Logger, trace, 2);
		std::ifstream f(InJsonFilePath);
		json LoadedJsonObj = json::parse(f, nullptr, true, true);

		// Load [GlobalLevel]
		std::string LevelStr = LoadedJsonObj["GlobalLevel"];
		GlobalLevel          = StringToLevelEnum.at(LevelStr);
		SPDLog::Log_T(Logger, 2, "DEBUG_VIEWER: Global Level str:[{}]   enum number:[{}]", LevelStr, (int)GlobalLevel);

		// Load [ViewerConfig]
		AllViewerConfigJsonObj = LoadedJsonObj["ViewerConfig"];
		if ( Logger->should_log(spdlog::level::debug) )
		{
			size_t ViewerCount = 0;
			for ( auto JsonIt : AllViewerConfigJsonObj )
			{
				for ( auto ViewerIt : JsonIt )
				{
					ViewerCount++;
				}
			}
			SPDLog::Log_T(Logger, 2, "DEBUG_VIEWER: Load [{}] categories, and [{}] viewers",
				AllViewerConfigJsonObj.size(), ViewerCount);
		}

		// Load
		EnabledViewerCategoryList = LoadedJsonObj["EnabledCategoryNameList"];

		f.close();
		LOG_FUNC_EXIT(Logger, trace, 2);
		return LoadedJsonObj;
	}

	static void UpdateCodePath(const std::string& InViewerID,
		const std::string& InViewerCategory,
		const std::string& InCodePath)
	{
		if ( InCodePath.size() == 0 )
		{
			return;
		}
		json LoadedJsonObj = __UpdateConfigJson_FromFile__(ViewerConfigJsonPath);

		LoadedJsonObj["ViewerConfig"][InViewerCategory][InViewerID]["CodePath"] = InCodePath;

		// Open json file in write mode. And output to file.
		std::ofstream OutputFileHandler;
		OutputFileHandler.open(ViewerConfigJsonPath);
		OutputFileHandler << LoadedJsonObj.dump(1, '\t');
		OutputFileHandler.close();
	}

	static bool bShouldViewerShow(const std::string& InViewerName,
		const ELevel& InLevel,
		const std::string& InCategoryName,
		const bool& bInEnable)
	{
		LOG_FUNC_ENTER(Logger, trace, 3);

		const bool bEnable_ByLogLevel = InLevel >= GlobalLevel;
		const bool bEnable_ByCategory = IsInEnabledCategoryList(InCategoryName);
		// const bool& bEnable_ByCategory = bCategoryEnable;
		const bool& bEnable_Internal = bInEnable;

		SPDLog::Log_T(Logger, 3, "DEBUG VIEWER [{}]: SelfLevel: [{}]   GlobalLevel: [{}]", InViewerName, (int)InLevel,
			(int)GlobalLevel);
		SPDLog::Log_T(Logger, 3, "DEBUG VIEWER [{}]: Internal: [{}] Category: [{}] LogLevel: [{}]", InViewerName,
			bEnable_Internal, bEnable_ByCategory, bEnable_ByLogLevel);

		LOG_FUNC_EXIT(Logger, trace, 3);
		return bEnable_Internal && bEnable_ByCategory && bEnable_ByLogLevel;
	}

#pragma endregion

private:
	/// 2-dimension json object.
	/// First level-key is Viewer Category.
	/// Second level-key is Viewer ID;
	/// The value is the config of the viewer.
	/// Example to get a viewer config obj:
	/// 	```ViewerConfigObj = ViewerConfigJsonObj[Category][ID];```
	inline static json AllViewerConfigJsonObj;
	inline static std::vector<std::string> EnabledViewerCategoryList;
	inline static std::map<std::string, ViewerPtrType> ViewerDict;
	inline static ELevel GlobalLevel = ELevel::INFO;

	inline static bool bIsInit = false;

	inline static SPDLog::LoggerType Logger;
};

}; // namespace PCL_Helper

#define DEBUG_SHOW_PC_LIST(VIEWER_NAME, CATEGORY_NAME, ...)                                                   \
	{                                                                                                         \
		constexpr const char CODE_PATH[] = __FILE__ ":{}";                                                    \
		const auto CodePath              = spdlog::fmt_lib::format(CODE_PATH, __LINE__);                      \
		if ( auto ViewerPtr = PCL_Helper::TDebugViewerManager<Types::CalcPoint>::GetOrMakeViewer(VIEWER_NAME, \
				 CATEGORY_NAME, CodePath) )                                                                   \
		{                                                                                                     \
			ViewerPtr->Clear();                                                                               \
			ViewerPtr->addCoordinateSystem(0.1);                                                              \
			ViewerPtr->AddPointCloudList({ __VA_ARGS__ }, CodePath);                                          \
			if ( ViewerPtr->wasStopped() )                                                                    \
			{                                                                                                 \
				ViewerPtr->createInteractor();                                                                \
			}                                                                                                 \
			ViewerPtr->spin();                                                                                \
		}                                                                                                     \
	}

#define DEBUG_SHOW_PCIDX_LIST(VIEWER_NAME, CATEGORY_NAME, REF_PC, ...)                                        \
	{                                                                                                         \
		constexpr const char CODE_PATH[] = __FILE__ ":{}";                                                    \
		const auto CodePath              = spdlog::fmt_lib::format(CODE_PATH, __LINE__);                      \
		if ( auto ViewerPtr = PCL_Helper::TDebugViewerManager<Types::CalcPoint>::GetOrMakeViewer(VIEWER_NAME, \
				 CATEGORY_NAME, CodePath) )                                                                   \
		{                                                                                                     \
			ViewerPtr->Clear();                                                                               \
			ViewerPtr->addCoordinateSystem(0.1);                                                              \
			ViewerPtr->AddPointCloudByIndexList(REF_PC, { __VA_ARGS__ }, CodePath);                           \
			if ( ViewerPtr->wasStopped() )                                                                    \
			{                                                                                                 \
				ViewerPtr->createInteractor();                                                                \
			}                                                                                                 \
			ViewerPtr->spin();                                                                                \
		}                                                                                                     \
	}

#define BEGIN_DEBUG_SHOW(VIEWER_NAME, CATEGORY_NAME, POINT_TYPE)                                                      \
	{                                                                                                                 \
		constexpr const char CODE_PATH[] = __FILE__ ":{}";                                                            \
		const auto CodePath              = spdlog::fmt_lib::format(CODE_PATH, __LINE__);                              \
		if ( auto ViewerPtr =                                                                                         \
				 PCL_Helper::TDebugViewerManager<POINT_TYPE>::GetOrMakeViewer(VIEWER_NAME, CATEGORY_NAME, CodePath) ) \
		{                                                                                                             \
			ViewerPtr->Clear();

#define END_DEBUG_SHOW()               \
	if ( ViewerPtr->wasStopped() )     \
	{                                  \
		ViewerPtr->createInteractor(); \
	}                                  \
	ViewerPtr->spin();                 \
	}                                  \
	}

#endif /* CD78FBA2_C76D_480F_B630_10BBFBC8B8C5 */
