cmake_minimum_required(VERSION 3.10)

# 项目信息
project(mobile-robot LANGUAGES C CXX)

# 编译时显示详细信息（调试时可打开）
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_BUILD_TYPE Debug)

# 设置可执行文件输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 设置库文件输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib)

# 设置C++标准和编译选项
set(CMAKE_CXX_STANDARD 17)         # 使用C++20标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O0")         # 不进行优化，便于调试

# 设置C标准和编译选项
set(CMAKE_C_STANDARD 17)         # 使用C20标准
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "-O0")         # 不进行优化，便于调试

# 添加OpenCV支持
find_package(OpenCV REQUIRED)
# 添加glog支持
find_package (glog REQUIRED)

# 添加链接库路径
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 包含头文件目录
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/command/Vision
)

file(GLOB_RECURSE SRC_DIR_LIST "src/*.ixx" "src/*.cpp" "src/*.c")
add_library(robot SHARED ${SRC_DIR_LIST})
target_link_libraries(robot ${OpenCV_LIBS} glog::glog yaml-cpp ydlidar_sdk vmxpi_hal_cpp rt pthread libMNN.so libMNN_Express.so libMNNOpenCV.so)

# 添加测试目录
add_subdirectory(./test)
