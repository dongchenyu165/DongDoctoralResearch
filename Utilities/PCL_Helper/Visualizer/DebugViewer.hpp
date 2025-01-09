#ifndef D9072965_8748_4CA8_880F_9AB1534CE2A4
#define D9072965_8748_4CA8_880F_9AB1534CE2A4

#include "Utilities/PCL_Helper/Basic/PCL_TypeAlias.hpp"
#include "Utilities/PCL_Helper/Visualizer/SimpleCloudViewer.hpp"
#include <memory>
#include <Utilities/spdlog/LogConfig.hpp>
#include <nlohmann/json.hpp>

#if false

template<typename PointType>
class TPointArrangementViewer;

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
#endif

namespace PCL_Helper
{
// 
template<typename PointType>
class TDebugViewer : public PCL_Helper::TSimpleCloudViewer<PointType>
{
	using json = nlohmann::json;
public:
	/**
	 * @brief The priority is [Larger is lower].
	 *
	 */
	// enum ELevel : int
	// {
	// 	TRACE = spdlog::level::trace,
	// 	DEBUG = spdlog::level::debug,
	// 	INFO  = spdlog::level::info,
	// 	WARN  = spdlog::level::warn,
	// 	ERROR = spdlog::level::err,
	// 	FATAL = spdlog::level::critical,
	// 	OFF   = spdlog::level::off,
	// };

	// inline static const std::map<const std::string, ELevel> StringToLevelEnum = {
	// 	{"TRACE",     ELevel::TRACE},
    //     { "DEBUG",    ELevel::DEBUG},
    //     { "INFO",     ELevel::INFO },
    //     { "WARN",     ELevel::WARN },
	// 	{ "ERROR",    ELevel::ERROR},
    //     { "CRITICAL", ELevel::FATAL},
    //     { "OFF",      ELevel::OFF  }
	// };

	using Super         = PCL_Helper::TSimpleCloudViewer<PointType>;
	using ViewerType    = TDebugViewer<PointType>;
	using ViewerPtrType = std::shared_ptr<ViewerType>;

/* -------------------------------------------------------------------------- */
/*                              Static Functions                              */
/* -------------------------------------------------------------------------- */
#if false
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
		if ( !ConfigObj.Enable || !IsInEnabledCategoryList(InViewerCategory) )
		{
			SPDLog::Log_D(Logger, 1, "DEBUG VIEWER: Category enable [{}];   Viewer enable: [{}]",
				IsInEnabledCategoryList(InViewerCategory), bIsViewerEnabled);
			LOG_FUNC_EXIT(Logger, debug, 1);
			return nullptr;
		}

		ViewerPtrType ViewerPtr = Get(InViewerID);
		if ( ViewerPtr )
		{
			SPDLog::Log_D(Logger, 1, "DEBUG VIEWER: [{}] is in [ViewerDict]", InViewerID);
			LOG_FUNC_EXIT(Logger, debug, 1);
			return ViewerPtr;
		}

		// Make a new viewer.
		const ELevel& Level = StringToLevelEnum.at(ConfigObj.Level);
		// TPointArrangementViewer
		if ( ConfigObj.ViewerClassStr.empty() )
		{
			ViewerDict[InViewerID] = std::make_shared<TDebugViewer<PointType>>(InViewerID, InViewerCategory,
				ConfigObj.WindowTitle, Level, ConfigObj.Enable);
		}
		else if ( ConfigObj.ViewerClassStr == "PointArrangementViewer" )
		{
			ViewerDict[InViewerID] = std::static_pointer_cast<TDebugViewer<PointType>>( std::make_shared<TPointArrangementViewer<PointType>>(InViewerID, InViewerCategory,
				ConfigObj.WindowTitle, Level, ConfigObj.Enable));
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
		json LoadedJsonObj = __UpdateConfigJson_FromFile__(ViewerConfigJsonPath);

		LoadedJsonObj["ViewerConfig"][InViewerCategory][InViewerID]["CodePath"] = InCodePath;

		// Open json file in write mode. And output to file.
		std::ofstream OutputFileHandler;
		OutputFileHandler.open(ViewerConfigJsonPath);
		OutputFileHandler << LoadedJsonObj.dump(1, '\t');
		OutputFileHandler.close();
	}

	#pragma endregion
#endif
private:
	/// 2-dimension json object.
	/// First level-key is Viewer Category.
	/// Second level-key is Viewer ID;
	/// The value is the config of the viewer.
	/// Example to get a viewer config obj:
	/// 	```ViewerConfigObj = ViewerConfigJsonObj[Category][ID];```
	// inline static json AllViewerConfigJsonObj;
	// inline static std::vector<std::string> EnabledViewerCategoryList;
	// inline static std::map<std::string, ViewerPtrType> ViewerDict;
	// inline static ELevel GlobalLevel = ELevel::INFO;

	// inline static bool bIsInit = false;

	inline static SPDLog::LoggerType Logger;

	/* -------------------------------------------------------------------------- */
	/*                              Member functions                              */
	/* -------------------------------------------------------------------------- */
public:
	/**
	 * @brief Use [TDebugViewer::GetOrMakeViewer] instead.
	 *
	 */
	TDebugViewer() = delete;

	TDebugViewer(const std::string& InID,
		const std::string& InCategoryName,
		const std::string& InWindowName,
		// const TDebugViewerManager::ELevel& InNewLevel = TDebugViewerManager::ELevel::DEBUG,
		const bool bInEnable     = true)
		: ID(InID), CategoryName(InCategoryName), Super(InWindowName), bEnable(bInEnable)
	{
		// __UpdateEnableCache__();
	}

	// void addPointCloud();

	/**
	 * @brief Override the parent spin() function.
	 The parent [spin()] function is NOT a [virtual] function.
	 *
	 */
	void spin() /* override */
	{
		if ( !ShouldShowWindow() )
		{
			return;
		}

		// Call the parent spin() function.
		Super::spin();
	}

	void AddPointCloudList(const std::vector<PCL_Helper::PCPTR<PointType>>& InPointList,
		const std::string& InName = "")
	{
		if ( !ShouldShowWindow() )
		{
			return;
		}

		Super::AddPointCloudList(InPointList, ID + InName);
	}

	void AddPointCloudByIndexList(PCL_Helper::PCPTR<PointType> InPC,
		std::vector<PCL_Helper::PCIDX_Ptr> InIndexList,
		const std::string& InName = "")
	{
		if ( !ShouldShowWindow() )
		{
			return;
		}

		Super::AddPointCloudByIndexList(InPC, InIndexList, InName);
	}

	void SetEnable(bool bInNewEnable) { bEnable = bInNewEnable; }

	bool ShouldShowWindow() { return bEnable; }

private:
	// void __UpdateEnableCache__()
	// {
	// 	LOG_FUNC_ENTER(Logger, trace, 3);

	// 	const bool bEnable_ByLogLevel = Level >= GlobalLevel;
	// 	// const bool bEnable_ByCategory = IsInEnabledCategoryList(CategoryName);
	// 	const bool& bEnable_ByCategory = bCategoryEnable;
	// 	const bool& bEnable_Internal  = bEnable;

	// 	SPDLog::Log_T(Logger, 3, "DEBUG VIEWER [{}]: SelfLevel: [{}]   GlobalLevel: [{}]", ID, (int)Level,
	// 		(int)GlobalLevel);
	// 	SPDLog::Log_T(Logger, 3, "DEBUG VIEWER [{}]: Internal: [{}] Category: [{}] LogLevel: [{}]", ID,
	// 		bEnable_Internal, bEnable_ByCategory, bEnable_ByLogLevel);

	// 	__CachedEnable__ = bEnable_Internal && bEnable_ByCategory && bEnable_ByLogLevel;
	// 	LOG_FUNC_EXIT(Logger, trace, 3);
	// }

	/* -------------------------------------------------------------------------- */
	/*                              Member Variables                              */
	/* -------------------------------------------------------------------------- */
protected:
	// ELevel Level;
	bool bEnable         = true;
	bool bCategoryEnable = true;
	std::string ID;
	std::string CategoryName;

	bool __CachedEnable__ = false;
};

} // namespace PCL_Helper

#endif /* D9072965_8748_4CA8_880F_9AB1534CE2A4 */
