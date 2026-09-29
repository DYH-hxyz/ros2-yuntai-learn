# ============================================================================
# ftservo_sdk/CMakeLists.txt —— 飞特 SMS/STS 舵机 SDK 的“内置子项目”构建脚本
# ----------------------------------------------------------------------------
# 这个子目录是从 src/sts_text/FTServo_C++-main 复制过来的 SDK 源码的精简版：
#   - 只保留 scservo_sdk/（协议层 + sms_sts 高层 API）和 third_party/serial（串口库）
#   - 去掉了 8 个示例程序、去掉了 README/详解文档/build 产物（那些不是库的一部分）
#
# 它是被外层 my_robot_hardware/CMakeLists.txt 用 add_subdirectory(ftservo_sdk)
# 拉进来的“子工程”。作用只有一个：把 SDK 编成一个静态库 target 名叫 scservo_sdk，
# 让外层直接 target_link_libraries(... scservo_sdk) 就能用。
#
# 为什么不直接链接 sts_text 里现成的 libscservo_sdk.a ？
#   因为那是“手工 cmake .. && make”编出来的，colcon 不知道它、也不负责重编它；
#   把它做成“本包子工程”后，colcon build 会自动帮你编，整个工作区一键构建，
#   最省心、最不容易出现“路径失效”的问题。
# ============================================================================
cmake_minimum_required(VERSION 3.10)
project(ftservo_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ----------------------------------------------------------------------------
# 1) 串口库：third_party/serial 自带一份 CMakeLists.txt，直接拉进来。
#    它会在本工程里生成一个静态库 target，名字也叫 serial（wjwwood/serial）。
# ----------------------------------------------------------------------------
add_subdirectory(third_party/serial)

# ----------------------------------------------------------------------------
# 2) SDK 静态库：把 5 个 .cpp 编成一个 libscservo_sdk.a
# ----------------------------------------------------------------------------
add_library(scservo_sdk STATIC
    scservo_sdk/port_handler.cpp
    scservo_sdk/protocol_packet_handler.cpp
    scservo_sdk/group_sync_write.cpp
    scservo_sdk/group_sync_read.cpp
    scservo_sdk/sms_sts.cpp
)

# 头文件目录用 PUBLIC 公开出去：SDK 头文件里写的是
#     #include "scservo_sdk/sms_sts.h"
# 而 scservo_sdk/ 文件夹的“上级目录”就是本目录(ftservo_sdk)，
# 所以把 ${CMAKE_CURRENT_SOURCE_DIR}（= ftservo_sdk 根）作为 include 根目录。
# PUBLIC = 谁链接 scservo_sdk，谁就自动拿到这个 include 目录（不用自己再加）。
target_include_directories(scservo_sdk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ----------------------------------------------------------------------------
# 3) 关掉 SDK 自身的编译警告：-w 只作用于 scservo_sdk 这一个 target，
#    不会影响我们自己写的文件（外层仍然有 -Wall -Wextra）。
#
#    为什么 SDK 会冒出这两类警告？（都不是 bug，是上游代码风格）
#      a) -Wsign-compare（protocol_packet_handler.cpp 的 readRx / readTxRx）
#           if (result == COMM_SUCCESS && rxpacket.size() >= PKT_PARAMETER0 + length)
#         rxpacket.size() 的类型是 size_t（无符号），而 PKT_PARAMETER0 + length
#         是 int（有符号，PKT_PARAMETER0 = 5，length 是包长，不可能为负）。
#         C++ 里“无符号和有符号比较”会把 int 转成无符号再比，GCC 提醒一句
#         “你确定没搞错？”。这里逻辑是对的，只是写得不严谨。
#      b) -Wunused-parameter（port_handler.cpp 的 setupPort）
#           bool PortHandler::setupPort(int cflag_baud)
#         这个形参根本没被用到：SDK 保留它是为了兼容老接口（老版本用它直接
#         拼串口标志位），现在波特率由 setBaudRate() 单独设置。C++ 对未使用的
#         形参报警告，用 (void)cflag_baud; 也能消，但那是改第三方源码，不如
#         整包关警告来得干净。
#
#    结论：这是第三方 SDK 的既有代码，我们不改它，只屏蔽它带来的刷屏警告。
# ----------------------------------------------------------------------------
target_compile_options(scservo_sdk PRIVATE -w)

# 链接串口库。PUBLIC = 链 scservo_sdk 的人会“顺带”也链上 serial（传递依赖）。
# 因为静态库 .a 不会自己带上依赖，你不链 serial，最后链接会报 undefined reference。
target_link_libraries(scservo_sdk PUBLIC serial)
