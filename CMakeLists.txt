# cmake_minimum_required(VERSION 3.4...3.18)
# project(pybind11_test)

# add_subdirectory(pybind11)
# pybind11_add_module(pybind11_test src/main.cpp)

# # EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# # define (VERSION_INFO) here.
# target_compile_definitions(pybind11_test
#                            PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})






# # set(PYTHON_MODULE_EXTENSION "")
# # message("====\t Python3_LINK_OPTIONS: ${PYTHON_MODULE_EXTENSION}") 
# cmake_minimum_required(VERSION 3.15...3.22)
# project(pybind11_test LANGUAGES CXX)

# set(PYBIND11_FINDPYTHON on) 
# set(Python_ROOT_DIR "~/.pyenv/versions/cookteam-dong-env/bin")
# set(Python_DIR "~/.pyenv/versions/cookteam-dong-env/bin")
# include_directories("/home/cookteam/.pyenv/versions/3.10.9/include/python3.10")
# # find_package(Python 3.10 COMPONENTS Interpreter Development REQUIRED PATHS /home/cookteam/.pyenv/versions/cookteam-dong-env NO_DEFAULT_PATH)
# find_package (Python 3.10 COMPONENTS Interpreter Development)
# # message("====\t Python3_LINK_OPTIONS: ${PYTHON_MODULE_EXTENSION}") 


# if(Python_FOUND)
# 	# message("====\t Python3_VERSION: ${Python3_VERSION}")
# 	# message("====\t Python3_EXECUTABLE: ${Python3_EXECUTABLE}")
# 	# message("====\t Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
# 	# message("====\t Python3_LIBRARIES: ${Python3_LIBRARIES}")
# 	# message("====\t Python3_NumPy_VERSION: ${Python3_NumPy_VERSION}")
# 	# message("====\t Python3_LINK_OPTIONS: ${Python3_LINK_OPTIONS}") 
# 	# message("====\t Python3_LINK_OPTIONS: ${PYTHON_MODULE_EXTENSION}") 
	
# 	# message("====\t Python_VERSION: ${Python_VERSION}")
# 	# message("====\t Python_INCLUDE_DIRS: ${Python_INCLUDE_DIRS}")
# 	# message("====\t Python_EXECUTABLE: ${PYTHON_EXECUTABLE}")
# 	# message("====\t Python_LIBRARIES: ${Python_LIBRARIES}")
# 	# message("====\t Python_NumPy_VERSION: ${Python_NumPy_VERSION}")
# 	# message("====\t Python_LINK_OPTIONS: ${Python_LINK_OPTIONS}") 
# 	# message("====\t Python_LINK_OPTIONS: ${PYTHON_MODULE_EXTENSION}") 
	
	
# 	# find_package(pybind11 CONFIG REQUIRED) 
# 	# set(PYBIND11_PYTHON_VERSION 3.10)
# 	add_subdirectory(pybind11)
	
# 	# set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
# 	# set(PYTHON_LIBRARY "${Python3_LIBRARIES}")
# 	message("====\t Python_LINK_OPTIONS: ${PYTHON_MODULE_EXTENSION}") 
# 	pybind11_add_module(pybind11_test SHARED NO_EXTRAS main.cpp)
# endif(Python_FOUND)




cmake_minimum_required(VERSION 3.15...3.22)
project(GraspingPointCalculator LANGUAGES CXX)


# Enable CUDA support
option(ENABLE_CUDA_SUPPORT "Using CUDA?" ON)
if(ENABLE_CUDA_SUPPORT)
	# Find and Enable CUDA support
	set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.2)
	find_package(CUDA 12.2 REQUIRED)
	set(CMAKE_CUDA_COMPILER ${CUDA_NVCC_EXECUTABLE})  # Must set the CUDA compiler before enable_language(CUDA).
	enable_language(CUDA)

	# Set CUDA related variable.
	set(CMAKE_CUDA_STANDARD 17)
	set(CMAKE_CUDA_STANDARD_REQUIRED ON)
	set(CMAKE_CUDA_ARCHITECTURES 75)  # Set CUDA architecture（[75] for [RTX 2080]）
	message("CMAKE_CUDA_COMPILER: ${CMAKE_CUDA_COMPILER}")

	include_directories(${CUDA_INCLUDE_DIRS})
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL related packages
find_package(VTK 9.2 QUIET)
if (VTK_QT_VERSION)
	message("Not found VTK for some reason, try find QT before the VTK")
	find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets PATHS /home/cookteam/Qt/6.4.1/gcc_64 NO_DEFAULT_PATH)
	find_package(VTK 9.2 REQUIRED)
endif (VTK_QT_VERSION)
include_directories(${VTK_PREFIX_PATH}/include/vtk-9.2)
message("=== PCL ===")
find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${VTK_USE_FILE})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES}  ${PCL_LIBRARIES} )

# CGAL
find_package(CGAL REQUIRED COMPONENTS Core)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
set(CGAL_LINK_LIBRARIES CGAL::CGAL)
set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${CGAL_LINK_LIBRARIES})


# JSON related
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)
# set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES}  nlohmann_json::nlohmann_json )
set(JSON_LIBRARIES nlohmann_json::nlohmann_json)


# SPDLOG
# FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/releases/download/v1.14.0/spdlog.tar.gz)
# FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.14.0.tar.gz)
FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/v1.14.0.tar.gz)
FetchContent_MakeAvailable(spdlog)
add_subdirectory("Utilities/spdlog")
set(SPDLOG_LIBRARIES spdlog::spdlog)


# Google Test
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()


# cppitertools
FetchContent_Declare(cppitertools 
	GIT_REPOSITORY https://github.com/ryanhaining/cppitertools.git
	# GIT_REPOSITORY git@github.com:ryanhaining/cppitertools.git
	GIT_TAG 8c6fed182c30e7ccae8d09699cfb03862f24c28c
	GIT_SHALLOW ON
	)
FetchContent_MakeAvailable(cppitertools)
set(ITERTOOLS_LIBRARIES cppitertools::cppitertools)


# Pybind11 related
set(PYBIND11_FINDPYTHON on)  # <== Necessary for PYBIND11 finding Python from a custom path
set(YOUR_PYTHON_INTERPRETER_PATH "~/.pyenv/versions/cookteam-dong-env/bin" CACHE FILEPATH "Path to the Python interpreter.Like:[~/.pyenv/versions/cookteam-dong-env/bin]")
set(Python_ROOT_DIR "${YOUR_PYTHON_INTERPRETER_PATH}")
# message("====\t Python_LINK_OPTIONS: ${YOUR_PYTHON_INTERPRETER_PATH}") 
add_subdirectory(pybind11)


# For OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


# LIBDW: sudo apt install libdw-dev
# Get the backtrack info.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBDW REQUIRED libdw)
include_directories(${LIBDW_INCLUDE_DIRS})
set(ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${LIBDW_LIBRARIES})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Enable ccache if found
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}") 
endif()

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

# Define the build type macro for program.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(BUILD_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(BUILD_RELEASE)
endif()


# Add subdirectories.
add_subdirectory(1_PrepareData)
add_subdirectory(2_FilterByGeo)
add_subdirectory(SearchSpaceGenerator)

# Test targets related.
add_subdirectory(Utilities/EMST)
option(TEST_EMST "Build and execute the test" OFF)
if(TEST_EMST)
	message("Build EMST Test")
	add_subdirectory(Tests/EMST)
endif()


option(TEST_PCL_FITTING "Build and execute the test" OFF)
if(TEST_PCL_FITTING)
	message("Build PCL_FITTING Test")
	add_subdirectory(Tests/PCL_Fitting)
endif()


option(TEST_EIGEN "Build and execute the test" ON)
if(TEST_EIGEN)
	message("Build Eigen Test")
	add_subdirectory(Tests/EigenTest)
endif()


option(TEST_CUDA_EIGEN "Build and execute the CUDA EIGEN test" ON)
if(TEST_CUDA_EIGEN)
	message("Build CUDA EIGEN Test")
	add_subdirectory(Tests/cuda_eigen)
endif()


option(TEST_PYBIND11 "Build and execute the test [PYBIND11]" OFF)
if(TEST_PYBIND11)
	message("Build Pybind11 Test")
	add_subdirectory(Tests/PybindTest)
endif()


option(BUILD_MAIN "Build and execute the [BUILD_MAIN]" ON)
if(BUILD_MAIN)
	message("Build BUILD_MAIN")
	add_executable(main main.cpp ${SPD_LOG_HELPER_SRC} ${PREPARE_DATA_SRC} ${SEARCH_SPACE_GEN_SRC})
	target_link_libraries(main ${PCL_LIBRARIES} nlohmann_json::nlohmann_json ${SPDLOG_LIBRARIES} ${PREPARE_DATA_LIB} ${ITERTOOLS_LIBRARIES} ${ADDITIONAL_LIBRARIES})
endif()


option(TEST_1_PrepareData "Build and execute the [TEST_1_PrepareData]" OFF)
if(TEST_1_PrepareData)
	message("Build TEST_1_PrepareData")
	add_subdirectory(Tests/1_PrepareData)
endif()

option(TEST_2_FilterByGeo "Build and execute the [TEST_2_FilterByGeo]" ON)
if(TEST_2_FilterByGeo)
	message("Build TEST_2_FilterByGeo")
	add_subdirectory(Tests/2_FilterByGeo)
endif()

option(TEST_SearchSpaceTest "Build and execute the [TEST_SearchSpaceTest]" ON)
if(TEST_SearchSpaceTest)
	message("Build SearchSpaceTest")
	add_subdirectory(Tests/SearchSpaceTest)
endif()

option(TEST_ScoreCalculator "Build and execute the [TEST_ScoreCalculator]" ON)
if(TEST_ScoreCalculator)
	message("Build TEST_ScoreCalculator")
	add_subdirectory(Tests/ScoreCalculator)
endif()

option(TEST_FunctionAutoLogger "Build and execute the [TEST_FunctionAutoLogger]" ON)
if(TEST_FunctionAutoLogger)
	message("Build TEST_FunctionAutoLogger")
	add_subdirectory(Tests/spdlog_Test)
endif()

option(TEST_ForceBalancer "Build and execute the [TEST_ForceBalancer]" ON)
if(TEST_ForceBalancer)
	message("Build TEST_ForceBalancer")
	add_subdirectory(Tests/4_ForceScore)
endif()

