
# Itertools test
add_executable(ItertoolsTest ItertoolsTest.cpp)
target_link_libraries(ItertoolsTest ${ITERTOOLS_LIBRARIES} )


add_executable(ScoreCalculatorConfigTest ScoreCalculatorConfigTest.cpp)
target_link_libraries(ScoreCalculatorConfigTest gtest nlohmann_json::nlohmann_json)
add_test(NAME ScoreCalculatorTest COMMAND ScoreCalculatorConfigTest)