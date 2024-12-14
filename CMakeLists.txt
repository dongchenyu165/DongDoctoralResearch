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



find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL related packages
find_package(VTK 9.2 QUIET)
if (VTK_QT_VERSION)
	message("Not found VTK for some reason, try find QT before the VTK")
	find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets PATHS /home/cookteam/Qt/6.4.1/gcc_64 NO_DEFAULT_PATH)
	find_package(VTK 9.2 REQUIRED)
endif (VTK_QT_VERSION)
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


include_directories(${CMAKE_CURRENT_SOURCE_DIR})


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Og -Wno-deprecated")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)



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


option(TEST_PYBIND11 "Build and execute the test [PYBIND11]" OFF)
if(TEST_PYBIND11)
	message("Build Pybind11 Test")
	add_subdirectory(Tests/PybindTest)
endif()


option(BUILD_MAIN "Build and execute the [BUILD_MAIN]" OFF)
if(BUILD_MAIN)
	message("Build BUILD_MAIN")
	add_executable(main main.cpp ${SPD_LOG_HELPER_SRC})
	target_link_libraries(main ${PCL_LIBRARIES} nlohmann_json::nlohmann_json ${SPDLOG_LIBRARIES})
endif()


option(TEST_1_PrepareData "Build and execute the [TEST_1_PrepareData]" OFF)
if(TEST_1_PrepareData)
	message("Build TEST_1_PrepareData")
	add_subdirectory(1_PrepareData)
	add_subdirectory(Tests/1_PrepareData)
endif()

option(TEST_2_FilterByGeo "Build and execute the [TEST_2_FilterByGeo]" ON)
if(TEST_2_FilterByGeo)
	message("Build TEST_2_FilterByGeo")
	add_subdirectory(2_FilterByGeo)
	add_subdirectory(Tests/2_FilterByGeo)
endif()

option(TEST_ScoreCalculator "Build and execute the [TEST_ScoreCalculator]" ON)
if(TEST_ScoreCalculator)
	message("Build TEST_ScoreCalculator")
	add_subdirectory(Tests/ScoreCalculator)
endif()

