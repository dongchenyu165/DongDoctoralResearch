#ifndef DF113F32_2DE4_4CD6_A814_B39EDB1B7496
#define DF113F32_2DE4_4CD6_A814_B39EDB1B7496

#include "spdlog/logger.h"
#include <string>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <Utilities/spdlog/CustomFormatter.hpp>

namespace SPDLog
{
using LoggerType = std::shared_ptr<spdlog::logger>;

inline const std::vector<std::string> __INDENT_STR_LIST__ = {
	"",
	"\t",
	"\t\t",
	"\t\t\t",
	"\t\t\t\t",
	"\t\t\t\t\t",
	"\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t\t\t",
};

class LoggerManager
{
public:
	/**
	 * @brief Static method to load a logger config json and make a logger by that config.
	 *
	 * @param InJsonPath The json file path.
	 * @param InConfigKeyList The cascade key name list of the logger config.
	 * @return LoggerType
	 */
	static LoggerType GetOrMakeLoggerFromJsonPath(const std::string& InLoggerName,
		const std::string& InJsonFilePath,
		const std::vector<std::string>& InConfigKeyList);
	static LoggerType GetOrMakeLoggerFromJsonPath(const std::string& InLoggerName, const std::string& InJsonFilePath);
	static LoggerType MakeLoggerFromJsonPath(const std::string& InJsonPath,
		const std::vector<std::string>& InConfigKeyList);
	static LoggerType GetLogger(const std::string& InLoggerName);
	static LoggerType GetSubLogger(const std::string& InLoggerName, const std::string& InSubLoggerName);
	static void RemoveLogger(const std::string& InRemovingLoggerName);
};

/**
 * @brief Config class for the spdlog lib.
 Also provide [MakeLoggerFromThis] function to create a logger from this config.\n
 The logger contain two outputs [std] and [file].
 So the log pattern has two vars to setup.
 *
 */
class LoggerConfig
{
	friend class LoggerManager;

public:
	LoggerType MakeLoggerFromThis();

protected:
	// Config
	/* -------------------------------------------------------------------------- */
	/*                                 Config Vars                                */
	/* -------------------------------------------------------------------------- */
	std::string LoggerName;
	std::string LogLevel;
	bool LogEnable = true;

	/// Console output pattern: (Logger Name) HH:MM:SS : Log message.
	/// Format flags reference in the [Utilities/spdlog/FormatStr.md] file.
	std::string LogPattern_Std = "(%n) %e : %v";

	/// The path pattern of the output logger file.
	/// Support datetime format flags: https://en.cppreference.com/w/cpp/chrono/c/strftime.
	/// Also support format flag [%ms] to insert milliseconds part.
	std::string LogFileOutputPathPattern = "./Logs/%Y_%m_%d_%H_%M_%S__%ms.log";

	/// File output pattern: (Logger Name) (Log Level) YYYY-mm-dd_HH:MM:SS(Time Zone) : Log message
	/// Format flags reference in the [Utilities/spdlog/FormatStr.md] file.
	std::string LogPattern_File = "(%n) (%l) %Y-%m-%d_%e(%z) : %v";

	/* -------------------------------------------------------------------------- */
	/*                                Runtime Vars                                */
	/* -------------------------------------------------------------------------- */
	std::string LogFilePath = ""; // Formated file path.

public:
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(LoggerConfig,
		LoggerName,
		LogLevel,
		LogEnable,
		LogPattern_Std,
		LogFileOutputPathPattern,
		LogPattern_File);
};

class LogSection
{
	LoggerType Logger;
	spdlog::level::level_enum Level;
	int Indent;
	std::string Msg;

public:
	LogSection(LoggerType InLogger, spdlog::level::level_enum InLogLevel, std::string InAdditionalMsg)
		: Logger(InLogger), Level(InLogLevel), Msg(InAdditionalMsg)
	{

		if ( Logger )
		{
			Logger->log(Level, ">>> Start: {}", Msg);
		}
	}

	~LogSection()
	{
		if ( Logger )
		{
			Logger->log(Level, "<<< End: {}", Msg);
		}
	}
};

/* -------------------------------------------------------------------------- */
/*                                 Macro Area                                 */
/* -------------------------------------------------------------------------- */


template<typename... Args>
void Log_I(LoggerType& InLogger, const int& InIndentSize, const std::string& InMsg, Args&&... InArgs)
{
	InLogger->info("{}" + InMsg, __INDENT_STR_LIST__[InIndentSize], std::forward<Args>(InArgs)...);
}

template<typename... Args>
void Log_D(LoggerType& InLogger, const int& InIndentSize, const std::string& InMsg, Args&&... InArgs)
{
	InLogger->debug("{}" + InMsg, __INDENT_STR_LIST__[InIndentSize], std::forward<Args>(InArgs)...);
}

template<typename... Args>
void Log_T(LoggerType& InLogger, const int& InIndentSize, const std::string& InMsg, Args&&... InArgs)
{
	InLogger->trace("{}" + InMsg, __INDENT_STR_LIST__[InIndentSize], std::forward<Args>(InArgs)...);
}

#define LOG_FUNC_ENTER(LOGGER, LEVEL, INDENT)                                           \
	LOGGER->LEVEL("");                                                                  \
	LOGGER->LEVEL("{}==========================", SPDLog::__INDENT_STR_LIST__[INDENT]); \
	LOGGER->LEVEL("{}Entering {} {}:{}", SPDLog::__INDENT_STR_LIST__[INDENT], __FUNCTION__, __FILE__, __LINE__)

#define LOG_FUNC_EXIT(LOGGER, LEVEL, INDENT)                                            \
	LOGGER->LEVEL("{}Exiting {}", SPDLog::__INDENT_STR_LIST__[INDENT], __FUNCTION__);   \
	LOGGER->LEVEL("{}==========================", SPDLog::__INDENT_STR_LIST__[INDENT]); \
	LOGGER->LEVEL("")

#define LOG_SECTION(LOGGER, LEVEL, SECTION_TITLE) LOGGER->LEVEL("<<<< {} >>>>\n", SECTION_TITLE)

#define LOG_PROBE(LOGGER) \
	LOGGER->debug("Debug probe: Function: {} ; Line: {} ; File: {}", __FUNCTION__, __LINE__, __FILE__)
#define LOG_PROBE_TRACE(LOGGER) \
	LOGGER->trace("Debug probe: Function: {} ; Line: {} ; File: {}", __FUNCTION__, __LINE__, __FILE__)

} // namespace SPDLog

#endif /* DF113F32_2DE4_4CD6_A814_B39EDB1B7496 */
