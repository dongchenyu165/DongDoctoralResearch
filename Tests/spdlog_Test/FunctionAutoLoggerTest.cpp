#include "spdlog/spdlog.h"
#include <Utilities/spdlog/FunctionAutoLogger.hpp>
#include <Utilities/spdlog/LogConfig.hpp>


SPDLog::LoggerType gLogger;

void TestFunction(int InNum)
{
	// FUNC_LOGGER_ENTER;
	FUNC_LOGGER_ENTER_CUSTOM_LOGGER(gLogger);
	if ( InNum == 0 )
	{
		FUNC_LOGGER_RET;
	}
	else if ( InNum == 1 )
	{
		FUNC_LOGGER_RET;
	}
	else if ( InNum == 2 )
	{
		FUNC_LOGGER_RET;
	}
	else if ( InNum == 3 )
	{
		FUNC_LOGGER_RET;
	}
	else
	{
		FUNC_LOGGER_RET;
	}
}

void TestOuter1()
{
	FUNC_LOGGER_ENTER_CUSTOM_LOGGER(gLogger);
	TestFunction(1);
}

void TestOuter2()
{
	FUNC_LOGGER_ENTER_CUSTOM_LOGGER(gLogger);
	TestOuter1();
	LOG_INDENT(gLogger, debug, "TestFunction 1");
	TestFunction(1);
}


int main(int argc, char** argv)
{
	SPDLog::LoggerType Logger = spdlog::get(std::string("FunctionLogger"));
	if ( !Logger )
	{
		Logger = spdlog::stdout_color_mt("FunctionLogger");
	}
	Logger->set_level(spdlog::level::debug);
	gLogger                   = Logger;

	TestFunction(3);

	TestOuter1();

	TestOuter2();
}