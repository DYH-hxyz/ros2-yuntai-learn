# ============================================================================
# CMakeLists.txt —— my_robot_hardware 功能包(ament_cmake, C++) 构建脚本
# ============================================================================
# 这个包编出来的是【一个 ros2_control 硬件接口插件】：
#   - 源文件：src/mobile_base_hardware_interface.cpp
#             src/arm_hardware_interface.cpp
#             (+ include 里 header-only 的 feetech_driver.hpp，头文件不用单独编)
#   - 产物  ：libmy_robot_hardware.so —— 一个“插件动态库”
#
# 结构说明（理解这张图，整个 CMake 就通了）：
#   ┌──────────────────────────────────────────────────────────────┐
#   │ 本包 my_robot_hardware                                        │
#   │   add_library(my_robot_hardware ...)   ← 把我们的插件编成 .so│
#   │       │ 链接：                                                 │
#   │       ├─ scservo_sdk   ← ftservo_sdk/ 子工程编出的静态库(SDK) │
#   │       └─ hardware_interface / pluginlib / rclcpp / rclcpp_lifecycle (ROS) │
#   │   pluginlib_export_plugin_description_file(...) ← 导出 my_robot_hardware.xml │
#   └──────────────────────────────────────────────────────────────┘
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 关键：本包最终要编成【共享库插件 .so】（pluginlib 用 dlopen 运行时加载，静态库加载不了）。
# 开 POSITION_INDEPENDENT_CODE(-fPIC)：让下面 ftservo_sdk 里的静态库也带 -fPIC，
# 这样它们才能被链接进我们最终的共享库 my_robot_hardware.so。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ============================================================================
# 1) find_package：找【外部(系统里已装)的包】。
#    这里找的都是 ROS 2 官方装在 /opt/ros/jazzy 里的包，它们各自导出过
#    Config.cmake（ament_package 自动生成的），所以能找到。
#    ament_cmake          —— ament 构建框架
#    hardware_interface   —— 提供 SystemInterface 基类（我们继承它）
#    pluginlib            —— 运行时动态加载插件的机制
#    rclcpp               —— read/write 签名里的 rclcpp::Time / rclcpp::Duration
#    rclcpp_lifecycle     —— 生命周期回调 on_configure 等
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ============================================================================
# 2) add_subdirectory：把【内置的飞特 SDK 子工程】拉进来。
#    它会生成一个静态库 target，名字叫 scservo_sdk（还附带 serial）。
#    这样我们就能像“自己编的库”一样直接 target_link_libraries(... scservo_sdk)。
#    （这就是“世界一：同工程内按 target 身份链接”，不需要 find_package。）
# ============================================================================
add_subdirectory(ftservo_sdk)

# ============================================================================
# 3) 编我们的插件【共享库】(.so)。SHARED 必须有：pluginlib 靠 dlopen 加载 .so，
#    静态库 .a 无法被动态加载成硬件接口插件。
# ============================================================================
add_library(my_robot_hardware SHARED
  src/yuntai_hardware_interface.cpp
)

# 把自己 include/ 目录加进头文件搜索路径，这样能 #include "my_robot_hardware/xxx.h"
# PUBLIC：以后别人若链接我们的库，也能拿到这些头文件
target_include_directories(my_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接：
#   - scservo_sdk：飞特 SDK（它的 PUBLIC include 会把 ftservo_sdk/ 加进搜索路径，
#                  所以我们能 #include "scservo_sdk/sms_sts.h"；它 PUBLIC 链的 serial
#                  也会被传递带过来）
#   - hardware_interface::hardware_interface：SystemInterface 基类所在的库
#   - pluginlib / rclcpp / rclcpp_lifecycle：ROS 依赖
target_link_libraries(my_robot_hardware
  scservo_sdk
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# ============================================================================
# 4) 导出插件描述文件：告诉 pluginlib “本库里有这个硬件接口类”
#    hardware_interface = 这个插件属于哪个“基类类型族”(ros2_control 认 SystemInterface)
#    my_robot_hardware.xml = 描述文件（定义 class/type 的对应关系）
#    这一句会做两件事：把 xml 拷到 share/ 下，并注册插件索引。
# ============================================================================
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)

# 把插件 .so 装进 install/<pkg>/lib。必须显式 install，否则 ament 不会自动装库文件，
# 运行时 pluginlib 就找不到 .so（只有索引没有实体）。
install(TARGETS my_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 5) ament 导出：让【别的包】也能 find_package(my_robot_hardware) 用到我们的库
#    （不是必须，但养成好习惯：硬件接口以后可能被别的包 include/链接）
# ============================================================================
ament_export_libraries(my_robot_hardware)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

# ============================================================================
# 6) ament_package：收尾。它会自动生成 my_robot_hardwareConfig.cmake、
#    version 文件、以及把 share/ 下该拷的东西拷好（含上面导出的插件 xml）。
# ============================================================================
ament_package()
