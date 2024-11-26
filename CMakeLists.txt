cmake_minimum_required(VERSION 3.10)

# 项目信息
project(mobile-robot LANGUAGES C CXX)

# 编译时显示详细信息（调试时可打开）
set(CMAKE_VERBOSE_MAKEFILE ON)

# 设置可执行文件输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 设置库文件输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib)

# 设置C++标准和编译选项
set(CMAKE_CXX_STANDARD 17)         # 使用C++17标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O0")         # 不进行优化，便于调试

# 设置C标准和编译选项
set(CMAKE_C_STANDARD 17)         # 使用C17标准
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "-O0")         # 不进行优化，便于调试

# 添加OpenMP支持
find_package(OpenMP REQUIRED) # 确保系统安装了OpenMP
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# 添加OpenCV支持
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS}) # 添加OpenCV头文件路径
endif()

# 添加链接库路径
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 包含头文件目录
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/command/Vision
)

file(GLOB_RECURSE SRC_DIR_LIST "src/*.cpp" "src/*.c")
add_library(robot SHARED ${SRC_DIR_LIST})
target_link_libraries(robot ${OpenCV_LIBS} yaml-cpp ydlidar_sdk vmxpi_hal_cpp rt pthread libMNN.so libMNN_Express.so libMNNOpenCV.so)

# 添加测试目录
add_subdirectory(./test)
