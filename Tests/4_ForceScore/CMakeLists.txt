add_executable(BalancerTests ForceBalancerTest.cpp)
target_link_libraries(BalancerTests gtest)
add_test(NAME BalancerTest COMMAND BalancerTests)

add_executable(FingerForceGeneratorTest FingerForceGeneratorTest.cpp)
target_link_libraries(FingerForceGeneratorTest gtest nlohmann_json::nlohmann_json)
add_test(NAME "Finger Force Generator Test" COMMAND FingerForceGeneratorTest)