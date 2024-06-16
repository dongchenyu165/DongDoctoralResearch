
add_executable(Fitting PCL_Fitting.cpp )
target_link_libraries(Fitting ${PCL_LIBRARIES})

add_executable(EigenFitting EigenSplineFitting.cpp )
target_link_libraries(EigenFitting ${PCL_LIBRARIES} nlohmann_json::nlohmann_json )