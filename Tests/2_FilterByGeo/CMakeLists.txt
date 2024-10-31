
# Itertools test
add_executable(ItertoolsTest ItertoolsTest.cpp)
target_link_libraries(ItertoolsTest ${ITERTOOLS_LIBRARIES} )


message("====== FILTER_GEO_SRC: "${FILTER_GEO_SRC})
add_executable(SearchSpaceTest SearchSpaceTest.cpp ${SPD_LOG_HELPER_SRC} ${FILTER_GEO_SRC})
target_link_libraries(SearchSpaceTest ${PCL_LIBRARIES} nlohmann_json::nlohmann_json ${SPDLOG_LIBRARIES} ${ITERTOOLS_LIBRARIES})

add_executable(ScoreCalculatorConfigTest ScoreCalculatorConfigTest.cpp)
target_link_libraries(ScoreCalculatorConfigTest gtest nlohmann_json::nlohmann_json)