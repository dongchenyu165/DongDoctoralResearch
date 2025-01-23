add_executable(BalancerTests ForceBalancerTest.cpp)
target_link_libraries(BalancerTests gtest)
add_test(NAME BalancerTest COMMAND BalancerTests)

# Enable CUDA for FingerForceGeneratorTest
add_executable(FingerForceGeneratorTest FingerForceGeneratorTest.cpp ../../4_ForceScore/CUDA/cuda_force_calc_kernal.cu)
set_target_properties(FingerForceGeneratorTest PROPERTIES 
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES 75  # Adjust based on your GPU
)
target_link_libraries(FingerForceGeneratorTest gtest nlohmann_json::nlohmann_json)
add_test(NAME "Finger Force Generator Test" COMMAND FingerForceGeneratorTest)