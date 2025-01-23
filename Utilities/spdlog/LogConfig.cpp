#include "LogConfig.hpp"
#include "GlobalVars.hpp"
#include "Utilities/spdlog/CustomFormatter.hpp"
#include "spdlog/pattern_formatter.h"

#include <bits/chrono.h>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <Utilities/JSON_Helper/StructSerializer.hpp>

namespace SPDLog
{

inline std::string __GetStartTimeRestMS_String__()
{
	using namespace std::chrono;
	auto MillisecondsFromEpoch = duration_cast<milliseconds>(ProgramStartTime.time_since_epoch()).count();
	auto RestMilliseconds      = MillisecondsFromEpoch % 1000;
	return std::to_string(RestMilliseconds);
}

std::string __FormatProgramExecStartTimeStr__(const std::string& InFormatString)
{
	// Define format flag for Milliseconds.
	static const std::string MillisecondsFormatFlag = "%ms";

	// Find milliseconds format flag position.
	const int MillisecondsFormatFlagIdx = InFormatString.find(MillisecondsFormatFlag);

	// Generate milliseconds string and replaced to [InFormatString].
	std::string MS_Str             = __GetStartTimeRestMS_String__();
	std::string FormatStringWithMS = InFormatString;
	FormatStringWithMS.replace(MillisecondsFormatFlagIdx, MillisecondsFormatFlag.size(), MS_Str);

	// Convert the rest time format flag to string.
	std::time_t TimeObj = std::chrono::system_clock::to_time_t(ProgramStartTime);
	char Buffer[1024];
	strftime(Buffer, sizeof(Buffer), FormatStringWithMS.c_str(), localtime(&TimeObj));

	return std::string(Buffer);
}

/* -------------------------------------------------------------------------- */
/*                      class LoggerManager methods impl                      */
/* -------------------------------------------------------------------------- */
// Static method
LoggerType LoggerManager::GetOrMakeLoggerFromJsonPath(const std::string& InLoggerName,
	const std::string& InJsonFilePath,
	const std::vector<std::string>& InConfigKeyList)
{
	auto Logger = LoggerManager::GetLogger(InLoggerName);
	if ( Logger )
	{
		spdlog::trace("Create logger with name: [{}] Addr: [{:08x}]", InLoggerName, (uint64_t)(Logger.get()));
		return Logger;
	}

	Logger = LoggerManager::MakeLoggerFromJsonPath(InJsonFilePath, InConfigKeyList);
	if ( Logger->name() != InLoggerName )
	{
		spdlog::error("Inputed name [{}] is NOT equal to config json name [{}].", InLoggerName, Logger->name());
		assert(Logger->name() == InLoggerName);
	}
	return Logger;
}

LoggerType LoggerManager::GetOrMakeLoggerFromJsonPath(const std::string& InLoggerName,
	const std::string& InJsonFilePath)
{
	return GetOrMakeLoggerFromJsonPath(InLoggerName, InJsonFilePath, { InLoggerName });
}

LoggerType LoggerManager::MakeLoggerFromJsonPath(const std::string& InJsonPath,
	const std::vector<std::string>& InConfigKeyList)
{
	LoggerConfig LogConfig;
	JSON_Helper::LoadStructure_ByPath(InJsonPath, InConfigKeyList, LogConfig);

	LogConfig.LogFilePath = __FormatProgramExecStartTimeStr__(LogConfig.LogFileOutputPathPattern);
	spdlog::info("Logger [{}]'ll output log to file [{}]", LogConfig.LoggerName, LogConfig.LogFilePath);

	return LogConfig.MakeLoggerFromThis();
}

LoggerType LoggerManager::GetLogger(const std::string& InLoggerName) { return spdlog::get(InLoggerName); }

LoggerType LoggerManager::GetSubLogger(const std::string& InLoggerName, const std::string& InSubLoggerName)
{
	auto Logger = spdlog::get(InLoggerName);
	if ( Logger == nullptr )
	{
		spdlog::error("Parent logger [{}] is not exist!", InLoggerName);
		return nullptr;
	}

	return Logger->clone(InSubLoggerName);
}

void RemoveLogger(const std::string& InRemovingLoggerName)
{
	auto PendingRemovingLogger = spdlog::get(InRemovingLoggerName);
	if ( !PendingRemovingLogger )
	{
		spdlog::error("Logger [{}] is not exist!", InRemovingLoggerName);;
	}
	spdlog::drop(InRemovingLoggerName);
}

/* -------------------------------------------------------------------------- */
/*                       class LoggerConfig methods impl                      */
/* -------------------------------------------------------------------------- */
const std::map<std::string, spdlog::level::level_enum> StringToLevelEnum = {
	{"TRACE",     spdlog::level::trace   },
    { "DEBUG",    spdlog::level::debug   },
    { "INFO",     spdlog::level::info    },
	{ "WARN",     spdlog::level::warn    },
    { "ERROR",    spdlog::level::err     },
    { "CRITICAL", spdlog::level::critical},
	{ "OFF",      spdlog::level::off     }
};

LoggerType LoggerConfig::MakeLoggerFromThis()
{

	if ( spdlog::get(LoggerName) != nullptr )
	{
		spdlog::get(LoggerName)->warn("This logger has been created! ");
		return spdlog::get(LoggerName);
	}

	spdlog::trace("Create logger with name: [{}]", LoggerName);

	auto OrgStdSink  = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
	spdlog::sink_ptr StdSink  = OrgStdSink;
	spdlog::sink_ptr FileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(LogFilePath, true);
	OrgStdSink->set_color_mode(spdlog::color_mode::automatic);
	StdSink->set_pattern(LogPattern_Std);
	FileSink->set_pattern(LogPattern_File);
	StdSink->set_level(LogEnable ? StringToLevelEnum.at(LogLevel) : spdlog::level::off);
	FileSink->set_level(LogEnable ? StringToLevelEnum.at(LogLevel) : spdlog::level::off);

	LoggerType Logger = std::make_shared<spdlog::logger>(spdlog::logger(LoggerName, { StdSink, FileSink }));
	spdlog::register_logger(Logger);  // Insert logger to spdlog logger dictionary.

	Logger->set_level(LogEnable ? StringToLevelEnum.at(LogLevel) : spdlog::level::off);
	Logger->flush_on(StringToLevelEnum.at(LogLevel));

	spdlog::trace(" >>>> Create logger with Addr: [{:08x}]", (uint64_t)(Logger.get()));
	return spdlog::get(LoggerName);
}

} // namespace SPDLog