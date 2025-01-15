
# set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.2/bin/nvcc")

# # 启用 CUDA 支持
# set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.2)
# # find_package(CUDA 12.2 REQUIRED PATHS /usr/local/cuda NO_DEFAULT_PATH)
# find_package(CUDA 12.2 REQUIRED)
# enable_language(CUDA)

# set(CMAKE_CUDA_STANDARD 17)
# set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# # 设置 CUDA 架构（根据你的 GPU 架构调整）
# set(CMAKE_CUDA_ARCHITECTURES 75)  # 例如，75 对应 Turing 架构（如 RTX 20 系列）

include_directories(${CMAKE_SOURCE_DIR}/Tests/cuda_eigen)

# 添加可执行文件
add_executable(EigenCUDARotation main.cpp kernal_funcs.cu)

# 链接 Eigen 库
# target_link_libraries(EigenCUDARotation Eigen3::Eigen)

# # 配置 CUDA 编译选项
# target_compile_options(EigenCUDARotation PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
#     -O0  # 优化级别
#     --default-stream per-thread  # 每个线程使用独立的 CUDA 流
# >)

# # 配置 CUDA 链接选项
# target_link_options(EigenCUDARotation PRIVATE $<$<LINK_LANGUAGE:CUDA>:
#     -O0  # 优化级别
# >)