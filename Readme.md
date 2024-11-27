# 移动机器人项目

本项目由 C++ 编写，基于VMXPI为基础开发

## 环境部署

本项目依赖于 `yaml-cpp、 `vmxpi_hal_cpp`、` ydlidar_sdk`、`opencv`、`boost`，在构建项目前，首先要部署依赖项目

- [jbeder/yaml-cpp: A YAML parser and emitter in C++](https://github.com/jbeder/yaml-cpp)
- [YDLIDAR/YDLidar-SDK: Driver for receiving YD LiDAR data and more...](https://github.com/YDLIDAR/YDLidar-SDK)
- [opencv/opencv: Open Source Computer Vision Library](https://github.com/opencv/opencv)
- [boostorg/boost: Super-project for modularized Boost](https://github.com/boostorg/boost)

### 安装必要软件包

在开始构建依赖之前，需要先安装必要的软件包

``````bash
# 更新软件包
sudo apt update
# 安装必要软件包
sudo apt install clang cmake ninja pkg-config
``````

### 从源代码编译

`yaml-cxx` 与 `ydlidar_sdk` 只能通过源代码编译的方式进行安装

#### yaml-cpp

``````bash
# 克隆项目
git clone https://github.com/YDLIDAR/yaml-cpp.git
# 进入项目根目录
cd yaml-cpp
# 创建构建文件夹
mkdir build
# 进入构建文件夹
cd build
# 创建编译配置
cmake ..
# 编译
make
# 安装
sudo make install
``````

#### ydlidar_sdk

``````bash
# 克隆项目
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
# 进入项目根目录
cd YDLidar-SDK
# 创建构建文件夹
mkdir build
# 进入构建文件夹
cd build
# 创建编译配置
cmake ..
# 编译
make
# 安装
sudo make install
``````

### 通过 `apt install` 进行安装

在 Raspberry pi 上，`opencv` 与 `boost` 也可通过 `apt install` 命令进行安装

``````bash
# 更新软件包
sudo apt update
# 安装 opencv 与 boost
sudo apt install libopencv-dev libboost-all-dev
``````

执行命令时需确保可以正常链接网络，国内推荐更换 [Debian 清华源]([debian | 镜像站使用帮助 | 清华大学开源软件镜像站 | Tsinghua Open Source Mirror](https://mirrors.tuna.tsinghua.edu.cn/help/debian/))

### 手动部署

`vmxpi_hal_cpp` 较为特殊，目前没找到如何方便的进行编译，在此提供 `.so` 文件与 `.h` 文件，移动到指定目录即可完成部署

1. 将 `lib\libvmxpi_hal_cpp.so` 文件复制至 `/usr/local/lib/` 目录下

2. 将 `vmxpi.zip` 解压到 `/usr/local/include/` 目录下

## 项目编译

按顺序运行以下指令

``````bash
# 进入用户目录
cd ~/
# 克隆项目
git clone https://github.com/StarChenPy/mobile-robot.git
# 进入项目根目录
cd mobile-robot
# 创建构建文件夹
mkdir build
# 进入构建文件夹
cd build
# 创建编译配置
cmake -DCMAKE_MAKE_PROGRAM=ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -G Ninja -S .. -B .
# 编译
cmake --build . --target robot -j 3
``````

编译结束后，在项目根目录下的 `bin` 文件夹中会出现编译结果