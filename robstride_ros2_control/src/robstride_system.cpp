#include "robstride_ros2_control/robstride_system.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "robstride_ros2_control/command_mode.hpp"
#include "robstride_ros2_control/driver_config.hpp"
#include "robstride_driver/driver.hpp"

namespace robstride_ros2_control
{
namespace
{
std::vector<CommandModeState> current_command_modes(robstride_driver::RobStrideDriver & driver)
{
  const auto modes = driver.command_modes();
  const auto & joints = driver.joints();
  std::vector<CommandModeState> states;
  states.reserve(joints.size());
  for (size_t index = 0; index < joints.size(); ++index) {
    states.push_back(CommandModeState{
      joints[index].name, modes[index].position, modes[index].velocity, modes[index].effort});
  }
  return states;
}
}  // namespace

struct RobStrideSystem::Impl
{
  Impl()
  : driver(rclcpp::get_logger("RobStrideSystem")),
    node(std::make_shared<rclcpp::Node>("robstride_health_publisher")),
    health_publisher(node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/robstride/health", rclcpp::QoS(10)))
  {
    recovery_subscription = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/robstride/recover", 10, [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) {
        for (const auto index : msg->data) {driver.request_recovery(index);}
      });
    gate_publisher = node->create_publisher<std_msgs::msg::UInt64MultiArray>(
      "/robstride/command_gate_state", 10);
    error_publisher = node->create_publisher<std_msgs::msg::Bool>(
      "/robstride/hardware_error", rclcpp::QoS(1).transient_local());
    gate_subscription = node->create_subscription<std_msgs::msg::UInt64>(
      "/robstride/release_command_gate", 10,
      [this](std_msgs::msg::UInt64::ConstSharedPtr msg) {driver.release_command_gate(msg->data);});
    executor.add_node(node);
    executor_thread = std::thread([this]() {executor.spin();});
  }

  ~Impl()
  {
    executor.cancel();
    if (executor_thread.joinable()) {executor_thread.join();}
  }

  void publish_health()
  {
    std_msgs::msg::Float64MultiArray message;
    for (const auto & joint : driver.joints()) {
      message.data.push_back(joint.state.online);
      message.data.push_back(joint.state.ready);
      message.data.push_back(joint.state.fault);
    }
    health_publisher->publish(message);
    std_msgs::msg::UInt64MultiArray gate;
    const auto status = driver.command_gate_status();
    gate.data.assign(status.begin(), status.end());
    gate_publisher->publish(gate);
  }

  void publish_error(bool error)
  {
    std_msgs::msg::Bool msg;
    msg.data = error;
    error_publisher->publish(msg);
  }

  robstride_driver::RobStrideDriver driver;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr health_publisher;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr recovery_subscription;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr gate_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_publisher;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr gate_subscription;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread executor_thread;
};

RobStrideSystem::RobStrideSystem()
: impl_(std::make_unique<Impl>())
{
}

RobStrideSystem::~RobStrideSystem() = default;

#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
hardware_interface::CallbackReturn RobStrideSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    return impl_->driver.initialize(
      parse_driver_configuration(params.hardware_info)) ?
           hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}
#else
hardware_interface::CallbackReturn RobStrideSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    return impl_->driver.initialize(
      parse_driver_configuration(info)) ?
           hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}
#endif

std::vector<hardware_interface::StateInterface> RobStrideSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : impl_->driver.joints()) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.state.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.state.effort);
    if (joint.exports_temperature) {
      interfaces.emplace_back(joint.name, "temperature", &joint.state.temperature);
    }
    interfaces.emplace_back(joint.name, "fault", &joint.state.fault);
    interfaces.emplace_back(joint.name, "online", &joint.state.online);
    interfaces.emplace_back(joint.name, "ready", &joint.state.ready);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobStrideSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : impl_->driver.joints()) {
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.command.position);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_EFFORT, &joint.command.effort);
  }
  return interfaces;
}

hardware_interface::CallbackReturn RobStrideSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return impl_->driver.open() ?
         hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn RobStrideSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  impl_->driver.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!impl_->driver.start()) {return hardware_interface::CallbackReturn::ERROR;}
  impl_->publish_error(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  impl_->driver.stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  impl_->driver.stop();
  impl_->driver.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_error(const rclcpp_lifecycle::State &)
{
  impl_->publish_error(true);
  impl_->driver.stop();
  impl_->driver.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobStrideSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  impl_->driver.update_state();
  impl_->publish_health();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return impl_->driver.send_commands() ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RobStrideSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::string validation_error;
  if (!validate_command_mode_switch(
      current_command_modes(impl_->driver), start, stop, &validation_error))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  const auto resulting_states = command_modes_after_switch(
    current_command_modes(impl_->driver), start, stop);
  std::vector<robstride_driver::ClaimedInterfaces> modes;
  modes.reserve(resulting_states.size());
  for (const auto & state : resulting_states) {
    modes.push_back(robstride_driver::ClaimedInterfaces{
      state.position_active, state.velocity_active, state.effort_active});
  }
  if (!impl_->driver.validate_command_modes(modes, &validation_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  const auto resulting_states = command_modes_after_switch(
    current_command_modes(impl_->driver), start, stop);
  std::vector<robstride_driver::ClaimedInterfaces> modes;
  modes.reserve(resulting_states.size());
  for (const auto & state : resulting_states) {
    modes.push_back(robstride_driver::ClaimedInterfaces{
      state.position_active, state.velocity_active, state.effort_active});
  }
  std::string validation_error;
  if (!impl_->driver.apply_command_modes(modes, &validation_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace robstride_ros2_control

PLUGINLIB_EXPORT_CLASS(robstride_ros2_control::RobStrideSystem, hardware_interface::SystemInterface)
