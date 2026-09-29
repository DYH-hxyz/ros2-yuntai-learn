# ============================================================================
# third_party/serial/CMakeLists.txt —— 内置的 wjwwood/serial 串口库
# ----------------------------------------------------------------------------
# 这份是"源码内置版"的构建脚本：官方原版 CMakeLists 依赖 catkin(ROS1 的构建
# 工具)，那和我们无关。这里用最简方式直接把它编成一个静态库 libserial.a。
#
# 这个库是【纯 C++ 串口库】，与 ROS 没有任何关系；只是官方把它发布在 ROS 的
# apt 源里所以叫 ros-jazzy-serial。内置后你不需要安装任何东西，g++ 直接编译。
# ============================================================================
cmake_minimum_required(VERSION 3.10)
project(serial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

# Linux 下需要这两个源文件(serial.cc = 对外 API，unix.cc = POSIX 串口实现)
add_library(serial STATIC
    src/serial.cc
    src/impl/unix.cc
)

# 头文件目录公开给使用方(scservo_sdk 通过它 #include <serial/serial.h>)
target_include_directories(serial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# POSIX 串口实现需要链接这两个系统库
if(UNIX AND NOT APPLE)
    target_link_libraries(serial PUBLIC pthread rt)
endif()
