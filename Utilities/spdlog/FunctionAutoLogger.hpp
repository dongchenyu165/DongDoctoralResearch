#ifndef B44CFA3D_E46C_4B38_AD37_DBA551D7F50D
#define B44CFA3D_E46C_4B38_AD37_DBA551D7F50D

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <execinfo.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <fstream>
#include <dlfcn.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <dwarf.h>
#include <elfutils/libdwfl.h>
#include <vector>

#include "spdlog/fmt/bundled/core.h"
#include "spdlog/logger.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

/**
 * @brief A class to automatically log function entry and exit, also can print custom log
 * 			messages with a properate indent depending on the call stack depth.
 *		  The core idea is to use the backtrace() function to get the call stack(an array of function pointers),
 *		    and then use the dwfl library to convert the function pointer to the source file name and line number.
 * 		  Core idea related functions:
 * 		    - backtrace related:
 * 		    	- backtrace()
 * 		    	- backtrace_symbols()
 * 		    - dwfl libraries related:
 * 		    	- InitializeDwfl()
 * 		    	- GetFileAndLine()
 * # Usage:
 *    1. Add [FUNC_LOGGER_ENTER] macro to the begining of the function you want to log.
 *       Note: [FUNC_LOGGER_ENTER_CUSTOM_LOGGER(SPD_LOGGER_OBJ)] can be used to use a custom logger.
 *    			Both of the macroes will create a [FunctionAutoLogger] local object with the name [__AUTO_GEN__Logger]
 *    2. Replace the return statement by [FUNC_LOGGER_RET] macro. For examplr: 
 *				FUNC_LOGGER_RET; 
 *					or 
 *				FUNC_LOGGER_RET [YOUR_RETURN_VAL]; 
 * 			Note: Without this macro, the program can run normally, but the printed return position maybe not
 *accurate enough.
 */
class FunctionAutoLogger
{
public:
	FunctionAutoLogger(const std::string& InFunctionName, const char* InFileName, int InLineNumber)
		: FunctionName_(InFunctionName), FileName_(InFileName), LineNumber_(InLineNumber)
	{
		StackDepth_ = FindStackDepth(StackBuffer_, StackFrameCount_);

		Logger_ = spdlog::get(std::string("FunctionLogger"));
		if ( !Logger_ )
		{
			Logger_ = spdlog::stdout_color_mt("FunctionLogger");
		}
		Logger_->set_level(spdlog::level::debug);

		PrintEnteringMsg();
	}

	FunctionAutoLogger(std::shared_ptr<spdlog::logger> InLogger,
		const std::string& InFunctionName,
		const char* InFileName,
		int InLineNumber)
		: Logger_(InLogger), FunctionName_(InFunctionName), FileName_(InFileName), LineNumber_(InLineNumber)
	{
		if ( !InLogger )
		{
			Logger_ = spdlog::get(std::string("FunctionLogger"));
			if ( !Logger_ )
			{
				Logger_ = spdlog::stdout_color_mt("FunctionLogger");
			}
			Logger_->set_level(spdlog::level::debug);
		}

		StackDepth_ = FindStackDepth(StackBuffer_, StackFrameCount_);

		PrintEnteringMsg();
	}

	~FunctionAutoLogger()
	{
		StackDepth_ = FindStackDepth(StackBuffer_, StackFrameCount_);
		// No [FUNC_LOGGER_RET] macro added, use [GetFileAndLine] to get the return position (maybe not accurate).
		if ( ReturnLineNumber_ == -1 )
		{
			GetFileAndLine((uintptr_t)StackBuffer_[2], ReturnFileName_, ReturnLineNumber_);
		}
		PrintReturnMsg();
	}

	void RecordReturnLocation(const char* InFileName, int InLineNumber)
	{
		ReturnFileName_   = InFileName;
		ReturnLineNumber_ = InLineNumber;
	}

	int GetStackDepth() const { return StackDepth_; }

	template<typename... Args>
	void PrintMsg(const Args&... args)
	{
		std::cout << GetIndentString(StackDepth_);
		// 使用逗号表达式展开参数包
		((std::cout << args << " "), ...);
		std::cout << std::endl; // 输出换行符
	}

	std::string GetIndentString()
	{
		return std::string(__INDENT_STR_LIST[StackDepth_ > 0 ? StackDepth_ : 0].data());
	}

	static std::string_view GetIndentString(int InStackDepth)
	{
		return std::string_view(__INDENT_STR_LIST[InStackDepth > 0 ? InStackDepth : 0].data());
	}

private:
	static constexpr std::size_t MaxIndentLevel = 256;
	using IndentStringType                      = std::array<char, MaxIndentLevel + 1>; // +1 For '\0' char.
	using IndentStringListType = std::array<IndentStringType, MaxIndentLevel + 1>; // 256个缩进字符串 + 1个空字符串

	// 编译时生成包含指定个数的制表符的字符串
	static constexpr IndentStringType GenerateIndent(std::size_t level)
	{
		IndentStringType indent{};
		for ( std::size_t i = 0; i < level; ++i )
		{
			indent[i] = '\t';
		}
		indent[level] = '\0'; // 添加字符串结束符
		return indent;
	}

	static constexpr IndentStringListType GenerateIndentList()
	{
		IndentStringListType indentList{};
		for ( std::size_t i = 0; i <= MaxIndentLevel; ++i )
		{
			indentList[i] = GenerateIndent(i);
		}
		return indentList;
	}

	static inline const IndentStringListType __INDENT_STR_LIST = GenerateIndentList();



private:
	void PrintEnteringMsg()
	{
		if ( !ShouldPrint() )
		{
			return;
		}

		std::string SourceFileName;
		int SourceLineNum;
		// At this time, the [StackBuffer_] contains: 0:FindStackDepth, 1:FunctionAutoLogger, 2:CALLED_FUNCTION,
		// 3:CALLER_FUNCTION, and so on. We need to get the caller of [CALLED_FUNCTION], which is [CALLER_FUNCTION]. And
		// we finally print the information (like where it is called from) of [CALLER_FUNCTION]. So we need to get the
		// 4th element (the index is 3) of [StackBuffer_].
		const uintptr_t& CallerFuncPtr = (uintptr_t)StackBuffer_[3];
		GetFileAndLine(CallerFuncPtr, SourceFileName, SourceLineNum);

		auto EnteringMsg   = fmt::format("{IndentStr}>>> Entering {FunctionName} Function, Stack Depth:{StackDepth}",
			  fmt::arg("IndentStr", GetIndentString(StackDepth_ - 1)), fmt::arg("FunctionName", FunctionName_),
			  fmt::arg("StackDepth", StackDepth_));
		auto CalledFromMsg = fmt::format("{IndentStr}Called from: {FileName}:{LineNumber}",
			fmt::arg("IndentStr", GetIndentString(StackDepth_ - 1)), fmt::arg("FileName", SourceFileName),
			fmt::arg("LineNumber", SourceLineNum));

		Logger_->debug("");
		Logger_->debug(EnteringMsg);
		Logger_->debug(CalledFromMsg);
	}

	void PrintReturnMsg()
	{
		if ( !ShouldPrint() )
		{
			return;
		}

		auto ReturnMsg = fmt::format(
			"{IndentStr}<<< RETURN from {FunctionName} Function, return pos: {SourceFileName}:{SourceLineNum}",
			fmt::arg("IndentStr", GetIndentString(StackDepth_ - 1)), fmt::arg("FunctionName", FunctionName_),
			fmt::arg("SourceFileName", ReturnFileName_), fmt::arg("SourceLineNum", ReturnLineNumber_));
		Logger_->debug(ReturnMsg);
		Logger_->debug("");
	}

	bool ShouldPrint()
	{
		if ( !bEnableLogging )
		{
			return false;
		}
		if ( Logger_ == nullptr || !(Logger_->should_log(spdlog::level::debug)) )
		{
			return false;
		}
		return true;
	}

public:
	static inline bool bEnableLogging = true;
	static inline void* MainFuncPtr   = nullptr;
	static inline Dwfl* DwflObjPtr    = nullptr; // Use to get the source file and line number info.

private:
	std::string FunctionName_;
	const char* FileName_;
	int LineNumber_;
	std::string ReturnFileName_ = "";
	int ReturnLineNumber_       = -1;
	int StackDepth_;
	std::shared_ptr<spdlog::logger> Logger_;

	static constexpr int STACK_BUFFER_SIZE = 32;
	void* StackBuffer_[STACK_BUFFER_SIZE];
	int StackFrameCount_ = -1;

	static int FindStackDepth(void* InBuffer[], int& InFrames)
	{
		InFrames = backtrace(InBuffer, STACK_BUFFER_SIZE);

		int MainFuncStackDepth = GetMainFuncStackDepth(InBuffer, InFrames);
		return MainFuncStackDepth -
		       2; // -2 means: Ignore the [FindStackDepth] and [FunctionLogger construct] functions' stack.
	}

	// 初始化 libdwfl
	static void InitializeDwfl()
	{
		static const Dwfl_Callbacks callbacks = { .find_elf = dwfl_linux_proc_find_elf,
			.find_debuginfo                                 = dwfl_standard_find_debuginfo,
			.section_address                                = dwfl_offline_section_address };

		DwflObjPtr = dwfl_begin(&callbacks);
		if ( !DwflObjPtr )
		{
			std::cerr << "无法初始化 Dwfl: " << dwfl_errmsg(-1) << std::endl;
			return;
		}

		if ( dwfl_linux_proc_report(DwflObjPtr, getpid()) != 0 )
		{
			std::cerr << "无法报告进程: " << dwfl_errmsg(-1) << std::endl;
			dwfl_end(DwflObjPtr);
			return;
		}

		if ( dwfl_report_end(DwflObjPtr, nullptr, nullptr) != 0 )
		{
			std::cerr << "无法结束报告: " << dwfl_errmsg(-1) << std::endl;
			dwfl_end(DwflObjPtr);
			return;
		}
	}

	// 使用 libdwfl 获取行号信息
	/**
	 * @brief Retrieves the file name and line number for a given function return address.
	 *
	 * This static function uses DWARF debugging information to map a memory address
	 * back to its corresponding source file and line number. It's particularly useful
	 * for debugging and logging purposes, allowing you to pinpoint the exact location
	 * in the source code that corresponds to a particular point of execution.
	 *
	 * @param InFunctionReturnAddress The return address of the function call to be located.
	 * @param[out] OutFileName A reference to a string that will be filled with the source file name.
	 * @param[out] OutLineNumber A reference to an integer that will be filled with the line number.
	 *
	 * @return bool Returns true if the file and line information was successfully retrieved,
	 *              false otherwise.
	 *
	 * @note This function relies on DWARF debugging information being available.
	 *       It initializes the DWARF debugging object if it hasn't been initialized yet.
	 *
	 * @warning If the function fails to retrieve the information, it will print an error
	 *          message to std::cerr.
	 */
	static bool GetFileAndLine(uintptr_t InFunctionReturnAddress, std::string& OutFileName, int& OutLineNumber)
	{
		if ( !DwflObjPtr )
		{
			InitializeDwfl();
		}
		constexpr uint8_t CALL_INSTRUCTION_LENGTH = 5;
		Dwarf_Addr ExecAddr                       = InFunctionReturnAddress -
		                      CALL_INSTRUCTION_LENGTH; // Subtract the call instruction length, to get the actual
		                                               // calling instruction address instead of the next instruction.
		Dwfl_Line* LineInfoObjPtr = dwfl_getsrc(DwflObjPtr, ExecAddr);

		if ( !LineInfoObjPtr )
		{
			std::cerr << "dwfl_getsrc() got error: " << dwfl_errmsg(-1) << std::endl;
			return false;
		}

		const char* FileNameStr = dwfl_lineinfo(LineInfoObjPtr, &ExecAddr, &OutLineNumber, nullptr, nullptr, nullptr);
		if ( FileNameStr )
		{
			OutFileName = FileNameStr;
			return true;
		}

		return false;
	}

public:
	static int GetMainFuncStackDepth(void* InBuffer[] = nullptr, int InFrames = -1)
	{
		const bool bInBufferIsNull = InBuffer == nullptr;
		if ( bInBufferIsNull )
		{
			InBuffer = new void*[STACK_BUFFER_SIZE];
			InFrames = backtrace(InBuffer, STACK_BUFFER_SIZE);
		}

		char** Symbols = backtrace_symbols(InBuffer, InFrames);
		int MainIndex  = -1;

		// 查找 main 函数在调用栈中的位置
		if ( MainFuncPtr == nullptr )
		{
			char** Symbols = backtrace_symbols(InBuffer, InFrames);
			for ( int i = 0; i < InFrames; ++i )
			{
				if ( Symbols[i] && strstr(Symbols[i], "__libc_start_main") )
				{
					MainIndex   = i;                   // Found index of [__libc_start_main].
					MainFuncPtr = InBuffer[MainIndex]; // Record the addr of [__libc_start_main].
					break;
				}
			}
			free(Symbols);
		}
		else
		{
			for ( int i = 0; i < InFrames; ++i )
			{
				if ( InBuffer[i] == MainFuncPtr )
				{
					MainIndex = i;
					break;
				}
			}
		}
		if ( MainIndex == -1 )
		{
#ifdef _OPENMP
			if ( omp_get_thread_num() == 0 )
			{
				std::cerr << "Failed to find main function in the stack trace" << std::endl;
			}
#else
			std::cerr << "Failed to find main function in the stack trace" << std::endl;
#endif
			return -1;
		}

		if ( bInBufferIsNull )
		{
			delete[] InBuffer;
		}
		// 返回 main 的深度, instead of [__libc_start_main].
		return MainIndex - 1;
	}

	static void Init()
	{
		InitializeDwfl();
		GetMainFuncStackDepth();
	}
};

#ifdef BUILD_RELEASE
	#define FUNC_LOGGER_ENTER
	#define FUNC_LOGGER_ENTER_CUSTOM_LOGGER(LOGGER)
	#define FUNC_LOGGER_RET return
#elif defined(BUILD_DEBUG)
	#define FUNC_LOGGER_ENTER     FunctionAutoLogger __AUTO_GEN__Logger(__func__, __FILE__, __LINE__)
	#define FUNC_LOGGER_ENTER_CUSTOM_LOGGER(LOGGER) \
		FunctionAutoLogger __AUTO_GEN__Logger(LOGGER, __func__, __FILE__, __LINE__)
	#define FUNC_LOGGER_RET                                          \
		__AUTO_GEN__Logger.RecordReturnLocation(__FILE__, __LINE__); \
		return
	#define LOG_INDENT(LOGGER, LEVEL, ...) \
		if (LOGGER)\
		{\
			LOGGER->LEVEL(__AUTO_GEN__Logger.GetIndentString() + __VA_ARGS__);\
		}
	#define LOG_INDENT_CHECK_SHOULD_LOG(LOGGER, LEVEL, ...) \
		if (LOGGER)\
		{\
			if (LOGGER->should_log(spdlog::level::LEVEL))\
			{\
				LOGGER->LEVEL(__AUTO_GEN__Logger.GetIndentString() + __VA_ARGS__);\
			}\
		}
#else
	#define FUNC_LOGGER_ENTER
	#define FUNC_LOGGER_ENTER_CUSTOM_LOGGER(LOGGER)
	#define FUNC_LOGGER_RET return
#endif


#endif /* B44CFA3D_E46C_4B38_AD37_DBA551D7F50D */
