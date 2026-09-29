// ============================================================================
// yuntai_hardware_interface.hpp —— 【硬件接口层】的类声明
// ----------------------------------------------------------------------------
// 本类把 ros2_control 与“云台的两个飞特舵机”接起来：
//   ros2_control 只认识“关节名 + 接口名(position/velocity)”，
//   本类把接口翻译成对 FeetechDriver 的调用（舵机协议细节在 SDK 层）。
//   read()  : 读舵机位置 → 写进 position 状态接口
//   write() : 从 position 命令接口取目标值 → 写给舵机
// ★ 命名空间/类名必须与 my_robot_hardware.xml 里的 type 完全一致：
//     my_robot_hardware/YuntaiHardwareInterface
//       → yuntai_hardware::YuntaiHardwareInterface
// ============================================================================
#ifndef MY_ROBOT_HARDWARE__YUNTAI_HARDWARE_INTERFACE_HPP_
#define MY_ROBOT_HARDWARE__YUNTAI_HARDWARE_INTERFACE_HPP_

#include <memory>  
#include <string>   
#include <vector> 

#include "hardware_interface/system_interface.hpp"

#include "my_robot_hardware/feetech_driver.hpp"

namespace yuntai_hardware
{

class YuntaiHardwareInterface : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 串口设备名，来自 URDF 硬件参数 <param name="port">（默认 /dev/ttyACM0）
  std::string port_;

  /// 两个云台舵机在总线上的 ID，来自 <param name="jointN_motor_id">
  int joint1_motor_id_ = 1;
  int joint2_motor_id_ = 2;

  /// 飞特舵机驱动（真正“说舵机话”的那一层，在 on_init 里创建）
  std::shared_ptr<my_robot_hardware::FeetechDriver> driver_;
};

}  

#endif  