add_executable(Eigen_test ./EigenTest.cpp)
target_link_libraries(Eigen_test nlohmann_json::nlohmann_json)