add_executable(CuttingFaceTest CuttingFaceTest.cpp ${SPD_LOG_HELPER_SRC} ${PREPARE_DATA_SRC})
target_link_libraries(CuttingFaceTest ${PCL_LIBRARIES} nlohmann_json::nlohmann_json ${SPDLOG_LIBRARIES} ${PREPARE_DATA_LIB})
