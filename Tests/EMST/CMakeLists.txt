
include_directories(${EMST_INCLUDE_DIR})
# add_executable(EMST_test ./EMST_Test.cpp ${EMST_SOURCE_FILES} ${JSON_LIBRARIES})
add_executable(EMST_test ./EMST_Test.cpp ${EMST_SOURCE_FILES})
target_link_libraries(EMST_test nlohmann_json::nlohmann_json ${SPDLOG_LIBRARIES})

add_executable(CGAL_EMST CGAL_EMST.cpp  ${SPD_LOG_HELPER_SRC})
target_link_libraries(CGAL_EMST ${PCL_LIBRARIES} nlohmann_json::nlohmann_json ${CGAL_LINK_LIBRARIES} ${SPDLOG_LIBRARIES})