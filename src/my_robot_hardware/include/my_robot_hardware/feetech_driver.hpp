// ============================================================================
// feetech_driver.hpp —— 【硬件驱动层】舵机驱动的 header-only 包装类
// ============================================================================
// ★ 先搞清楚“驱动 driver”和“硬件接口 hardware interface”的区别（这是本包分层的关键）：
//
//   ┌─ 硬件接口层（mobile_base_hardware_interface.hpp/.cpp）─────────────┐
//   │   面向 ros2_control：它认识关节名、命令接口/状态接口、rad/s 等单位。│
//   │   它【不知道】舵机协议，只调用下面的 Driver 的“人话方法”。        │
//   └──────────────────────────────┬──────────────────────────────────┘
//                                  │ 调用（方法名都是“人话”）
//   ┌─ 硬件驱动层（本文件 feetech_driver.hpp）───────────────────────────┐
//   │   面向“这一种舵机”：它认识舵机 ID、轮子模式、寄存器速度值(raw)。  │
//   │   它【不知道】ros2_control / 关节名 / rad/s，只管“让某个 ID 的舵机 │
//   │   以多快速度转”。                                               │
//   └──────────────────────────────┬──────────────────────────────────┘
//                                  │ 调用
//   ┌─ 厂商 SDK 层（ftservo_sdk/scservo_sdk 里的 scs::sms_sts）────────┐
//   │   面向“总线协议”：打包一帧帧字节、校验和、收发、串口。           │
//   └──────────────────────────────┬──────────────────────────────────┘
//                                  │ 串口字节流
//                             真实舵机（硬件）
//
// 为什么叫 feetech_driver？
//   教程作者用他那款舵机写的包装类叫 XL330Driver；但【你手上是飞特 Feetech 舵机】，
//   不是 XL330，所以这里命名为 FeetechDriver，避免张冠李戴。方法接口
//   (activateWithVelocityMode / getVelocityRadianPerSec / ...) 则照教程保留，
//   方便你和教程逐行对照“接口层怎么写”。底层驱动的是飞特 Feetech SMS/STS 系列
//   总线舵机（SDK = ftservo_sdk 里的 scs::sms_sts）。
//
// 为什么是 header-only（只有 .hpp，没有 .cpp）？
//   这个类很小、只是把 scs::sms_sts 的调用和“单位换算”包一层，没有必须藏起来的
//   实现。C++ 里把函数体写在 class { ... } 大括号内的函数会自动视为 inline，
//   所以即使被多个 .cpp include 也不会产生“重复定义”的链接错误。它不需要单独编译，
//   它的代码会随着 include 它的 .cpp（也就是 mobile_base_hardware_interface.cpp）
//   一起被编进插件 .so 里。
// ============================================================================
#ifndef MY_ROBOT_HARDWARE__FEETECH_DRIVER_HPP_
#define MY_ROBOT_HARDWARE__FEETECH_DRIVER_HPP_

#include <cmath>    // M_PI
#include <cstdio>   // std::printf（出错时打印）
#include <memory>   // std::make_shared / shared_ptr
#include <string>   // std::string

// 引入飞特 SDK 的头文件（头文件搜索根目录 = ftservo_sdk/，由 CMake 的 PUBLIC include 提供）
#include "scservo_sdk/port_handler.h"   // 串口句柄：开/关/波特率
#include "scservo_sdk/sms_sts.h"        // 高层 API：WheelMode/ReadSpeed/WriteSpec...

namespace my_robot_hardware
{

class FeetechDriver
{
public:
  // --------------------------------------------------------------------------
  // 构造函数：只记住串口设备名。真正“打开串口”推迟到 init()，方便先构造后初始化。
  // --------------------------------------------------------------------------
  explicit FeetechDriver(const std::string & port_name)
  : port_(port_name)
  {
  }

  // --------------------------------------------------------------------------
  // init()：打开串口、设波特率、创建高层 API 对象。
  //   成功返回 true；失败返回 false（串口路径错 / 没权限 / 波特率不支持）。
  //   【为什么要分两步？】构造函数里抛异常不好处理；用返回 bool 的方式，让上层
  //   （硬件接口的 on_init / on_configure）能干净地决定“初始化失败就返回 ERROR”。
  // --------------------------------------------------------------------------
  bool init()
  {
    // 已经初始化过就不要再开一次
    if (port_handler_)
    {
      return true;
    }

    // 1) 创建串口句柄并打开（底层是 wjwwood/serial，打开失败多半是设备名错或没权限）
    port_handler_ = std::make_shared<scs::PortHandler>(port_);
    if (!port_handler_->openPort())
    {
      std::printf("[FeetechDriver] 打开串口失败: %s\n", port_.c_str());
      return false;
    }

    // 2) 设波特率。Feetech SCS 总线默认 1Mbps，SDK 示例都是 1000000。
    //    舵机实际波特率档位若不是 1M，要先去改舵机（地址 6 的档位值）或改这里。
    if (!port_handler_->setBaudRate(kBaudrate))
    {
      std::printf("[FeetechDriver] 设置波特率失败: %d\n", kBaudrate);
      return false;
    }

    // 3) 基于串口句柄创建高层 API 对象。sms_sts 里封装了“内存表 + 人话函数”。
    packet_handler_ = std::make_shared<scs::sms_sts>(port_handler_.get());
    return true;
  }

  // --------------------------------------------------------------------------
  // activateWithVelocityMode(id)：把某个舵机切到【轮子模式】（恒速转动）。
  //   对应教程 XL330Driver 的同名方法。
  //   底层动作：
  //     a) WheelMode(id) = 往内存表地址 33(MODE) 写 1 → 角度定位模式变成轮子恒速模式；
  //     b) 顺手把速度写成 0，避免一切换就冲出去。
  // --------------------------------------------------------------------------
  void activateWithVelocityMode(int motor_id)
  {
    if (!packet_handler_)
    {
      return;
    }
    int error = 0;
    int comm = packet_handler_->WheelMode(motor_id, error);
    if (comm != scs::COMM_SUCCESS)
    {
      std::printf(
        "[FeetechDriver] WheelMode 通信失败(id=%d): %s\n", motor_id,
        packet_handler_->getTxRxResult(comm).c_str());
    }
    if (error != 0)
    {
      std::printf(
        "[FeetechDriver] WheelMode 舵机报错(id=%d): %s\n", motor_id,
        packet_handler_->getRxPacketError(error).c_str());
    }
    // 切完模式先让速度归零（停住），等控制器真正下发指令再转。
    setTargetVelocityRadianPerSec(motor_id, 0.0);
  }

  // --------------------------------------------------------------------------
  // activateWithPositionMode(id)：切到角度定位模式并保持当前位置。
  // 先切 MODE=0，再把当前角度写成目标角度，避免切换模式时追逐旧目标。
  // --------------------------------------------------------------------------
  void activateWithPositionMode(int motor_id)
  {
    if (!packet_handler_)
    {
      return;
    }

    int error = 0;
    const int comm = packet_handler_->PositionMode(motor_id, error);
    if (comm != scs::COMM_SUCCESS || error != 0)
    {
      printCommunicationError("PositionMode", motor_id, comm, error);
      return;
    }

    setTargetPositionRadian(motor_id, getPositionRadian(motor_id));
  }

  // --------------------------------------------------------------------------
  // deactivate(id)：让舵机停下（速度 0）。
  //   对应教程 XL330Driver 的同名方法。停用硬件接口时调用。
  //   如需彻底“卸载/失能”，可再往 TORQUE_ENABLE(40) 写 0；这里先以“减速到 0”为准。
  // --------------------------------------------------------------------------
  void deactivate(int motor_id)
  {
    if (!packet_handler_)
    {
      return;
    }
    setTargetVelocityRadianPerSec(motor_id, 0.0);
  }

  // --------------------------------------------------------------------------
  // deactivatePosition(id)：位置模式下的停机方式。
  // 位置模式没有“速度命令为 0”这个语义，所以把目标位置改成当前位置。
  // --------------------------------------------------------------------------
  void deactivatePosition(int motor_id)
  {
    if (packet_handler_)
    {
      setTargetPositionRadian(motor_id, getPositionRadian(motor_id));
    }
  }

  // --------------------------------------------------------------------------
  // getVelocityRadianPerSec(id)：读回某舵机【当前实际速度】，单位 rad/s。
  //   对应教程 XL330Driver 的同名方法（注意：硬件接口层用 rad/s，这里负责换算）。
  //   底层：ReadSpeed(id) 读内存表地址 58/59(PRESENT_SPEED) 返回“原始值 raw”，
  //         然后把 raw 换算成 rad/s 返回。
  //   读失败时返回 0.0（调用方一般能容忍一两次 0）。
  // --------------------------------------------------------------------------
  double getVelocityRadianPerSec(int motor_id)
  {
    if (!packet_handler_)
    {
      return 0.0;
    }
    int present_speed_raw = 0;
    int error = 0;
    int comm = packet_handler_->ReadSpeed(motor_id, present_speed_raw, error);
    if (comm != scs::COMM_SUCCESS || error != 0)
    {
      // 通信失败/舵机报错：打印后返回 0（避免把垃圾值当作真实速度上报）
      if (comm != scs::COMM_SUCCESS)
      {
        std::printf(
          "[FeetechDriver] ReadSpeed 通信失败(id=%d): %s\n", motor_id,
          packet_handler_->getTxRxResult(comm).c_str());
      }
      return 0.0;
    }
    // raw → rad/s 换算：速度寄存器 1 个单位 ≈ 0.732 rpm；rpm→rad/s 乘 (2π/60)。
    return static_cast<double>(present_speed_raw) * kRawUnitToRadianPerSec;
  }

  // --------------------------------------------------------------------------
  // getPositionRadian(id)：读取当前位置并转换为 ros2_control 使用的弧度。
  // STS 位置原始值 0~4095 对应一整圈 0~2π。
  // --------------------------------------------------------------------------
  double getPositionRadian(int motor_id)
  {
    if (!packet_handler_)
    {
      return 0.0;
    }

    int present_position_raw = 0;
    int error = 0;
    const int comm = packet_handler_->ReadPos(motor_id, present_position_raw, error);
    if (comm != scs::COMM_SUCCESS || error != 0)
    {
      if (comm != scs::COMM_SUCCESS)
      {
        printCommunicationError("ReadPos", motor_id, comm, error);
      }
      return 0.0;
    }
    return static_cast<double>(present_position_raw) * kRawPositionToRadian;
  }

  // --------------------------------------------------------------------------
  // setTargetPositionRadian(id, position)：把弧度位置命令写给舵机。
  // forward_command_controller 下发的是弧度，WritePosEx 需要 0~4095 原始位置。
  // --------------------------------------------------------------------------
  void setTargetPositionRadian(int motor_id, double target_position_rad)
  {
    if (!packet_handler_)
    {
      return;
    }

    // 将角度归一化到 [0, 2π)，再换算成舵机位置寄存器值。
    const double normalized_position = std::fmod(target_position_rad, kTwoPi);
    const double positive_position = normalized_position < 0.0 ?
      normalized_position + kTwoPi : normalized_position;
    const int position_raw = static_cast<int>(std::lround(
      positive_position / kTwoPi * kPositionTicksPerRevolution));

    int error = 0;
    const int comm = packet_handler_->WritePosEx(
      motor_id, position_raw, kPositionSpeed, kAcceleration, error);
    if (comm != scs::COMM_SUCCESS || error != 0)
    {
      printCommunicationError("WritePosEx", motor_id, comm, error);
    }
  }

  // --------------------------------------------------------------------------
  // setTargetVelocityRadianPerSec(id, vel)：给某舵机下【目标速度】，单位 rad/s。
  //   对应教程 XL330Driver 的同名方法。
  //   底层：rad/s 先换算回“原始速度值 raw”，再 WriteSpec(id, raw, 加速度, err)
  //   写进内存表地址 41~47（ACC/位置区/时间区/速度区）——轮子模式下舵机就按该速度恒转。
  //   速度可正可负：正=正转，负=反转（SDK 的 toscs 会把符号位处理成 bit15）。
  // --------------------------------------------------------------------------
  void setTargetVelocityRadianPerSec(int motor_id, double target_vel_rad_per_sec)
  {
    if (!packet_handler_)
    {
      return;
    }
    // 1) rad/s → 原始速度值（四舍五入成整数，WriteSpec 的 speed 参数是 int）
    int speed_raw = static_cast<int>(
      std::lround(target_vel_rad_per_sec / kRawUnitToRadianPerSec));
    // 2) 写下去：加速度用一个固定档位（数字小=加得快，具体手感可调）
    int error = 0;
    int comm = packet_handler_->WriteSpec(motor_id, speed_raw, kAcceleration, error);
    if (comm != scs::COMM_SUCCESS)
    {
      std::printf(
        "[FeetechDriver] WriteSpec 通信失败(id=%d): %s\n", motor_id,
        packet_handler_->getTxRxResult(comm).c_str());
    }
    if (error != 0)
    {
      std::printf(
        "[FeetechDriver] WriteSpec 舵机报错(id=%d): %s\n", motor_id,
        packet_handler_->getRxPacketError(error).c_str());
    }
  }

private:
  // 统一打印位置 API 的通信错误，避免每个位置函数重复一套日志代码。
  void printCommunicationError(const char * operation, int motor_id, int comm, int error)
  {
    if (comm != scs::COMM_SUCCESS)
    {
      std::printf(
        "[FeetechDriver] %s 通信失败(id=%d): %s\n", operation, motor_id,
        packet_handler_->getTxRxResult(comm).c_str());
    }
    if (error != 0)
    {
      std::printf(
        "[FeetechDriver] %s 舵机报错(id=%d): %s\n", operation, motor_id,
        packet_handler_->getRxPacketError(error).c_str());
    }
  }

  // ---- 配置常量（可按你的舵机/手感调整）-------------------------------------
  static constexpr int kBaudrate = 1000000;                 // SCS 总线波特率 1M
  static constexpr int kAcceleration = 50;                  // WriteSpec 的加速度档位
  static constexpr int kPositionSpeed = 200;                // 位置模式目标速度档位
  static constexpr int kPositionTicksPerRevolution = 4096; // 0~4095 对应一整圈
  static constexpr double kTwoPi = 2.0 * M_PI;
  static constexpr double kRawPositionToRadian = kTwoPi / kPositionTicksPerRevolution;
  // 速度寄存器 1 个单位 ≈ 0.732 rpm；rad/s = rpm * 2π/60
  static constexpr double kRawUnitToRadianPerSec = 0.732 * (2.0 * M_PI) / 60.0;

  // ---- 成员 ----------------------------------------------------------------
  std::string port_;                                      // 串口设备路径
  std::shared_ptr<scs::PortHandler> port_handler_;        // 串口句柄（智能指针，自动释放）
  std::shared_ptr<scs::sms_sts> packet_handler_;          // 飞特高层 API 对象
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE__FEETECH_DRIVER_HPP_
