cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(PICameraCarTemp)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)
# 包含 OpenCV 头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(PICameraCarTemp main.cpp)

# 链接 OpenCV 库
target_link_libraries(PICameraCarTemp ${OpenCV_LIBS})