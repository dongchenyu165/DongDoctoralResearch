
add_executable(SearchSpaceTest SearchSpaceTest.cpp ${SPD_LOG_HELPER_SRC} ${SEARCH_SPACE_GEN_SRC} )
target_link_libraries(SearchSpaceTest ${PCL_LIBRARIES} gtest nlohmann_json::nlohmann_json ${SPDLOG_LIBRARIES} ${ITERTOOLS_LIBRARIES})
add_test(NAME SearchSpaceTest COMMAND SearchSpaceTest)