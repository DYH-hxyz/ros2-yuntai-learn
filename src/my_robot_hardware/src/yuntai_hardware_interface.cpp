#include "my_robot_hardware/yuntai_hardware_interface.hpp"

#include <cstdio>      
#include <string>      

#include "pluginlib/class_list_macros.hpp"

namespace yuntai_hardware
{
hardware_interface::CallbackReturn YuntaiHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
  
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const hardware_interface::HardwareInfo & info = params.hardware_info;

  auto get_param = [&info](const std::string & key, const std::string & default_value)
  {
    auto it = info.hardware_parameters.find(key);
    return (it != info.hardware_parameters.end()) ? it->second : default_value;
  };

  port_ = get_param("port", "/dev/ttyACM0");
  joint1_motor_id_ = std::stoi(get_param("joint1_motor_id", "1"));
  joint2_motor_id_ = std::stoi(get_param("joint2_motor_id", "2"));

  driver_ = std::make_shared<my_robot_hardware::FeetechDriver>(port_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YuntaiHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YuntaiHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  for (const auto & joint : info_.joints)
  {
    for (const auto & state_interface : joint.state_interfaces)
    {
      const std::string interface_name = joint.name + "/" + state_interface.name;
      set_state(interface_name, 0.0);
    }
  }


  if (!driver_->init())
  {
    std::fprintf(stderr, "[YuntaiHardwareInterface] 串口初始化失败，无法激活。\n");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 2) 左右舵机切到轮子(速度)模式并归零
  driver_->activateWithPositionMode(joint1_motor_id_);
  driver_->activateWithPositionMode(joint2_motor_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YuntaiHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->deactivatePosition(joint1_motor_id_);
  driver_->deactivatePosition(joint2_motor_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type YuntaiHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  const double joint1_position = driver_->getPositionRadian(joint1_motor_id_);
  const double joint2_position = driver_->getPositionRadian(joint2_motor_id_);


  set_state("yuntai_joint1/position", joint1_position);
  set_state("yuntai_joint2/position", joint2_position);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YuntaiHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  const double joint1_command = get_command("yuntai_joint1/position");
  const double joint2_command = get_command("yuntai_joint2/position");


  driver_->setTargetPositionRadian(joint1_motor_id_, joint1_command);
  driver_->setTargetPositionRadian(joint2_motor_id_, joint2_command);

  return hardware_interface::return_type::OK;
}
} 

PLUGINLIB_EXPORT_CLASS(
  yuntai_hardware::YuntaiHardwareInterface,
  hardware_interface::SystemInterface)