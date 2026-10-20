#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include "robstride_driver/driver.hpp"

namespace rs = robstride_driver;
using namespace std::chrono_literals;

void check_partial_operation(bool managed)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("partial_fake_bus",
    rclcpp::NodeOptions().context(context));
  auto publisher = node->create_publisher<can_msgs::msg::Frame>("partial_rx", 32);
  std::array<bool, 4> connected{{false, false, false, true}};
  std::array<int, 4> modes{};
  std::array<int, 4> motion_count{};
  std::array<int, 4> enable_count{};
  std::array<double, 4> positions{{0, 0.2, 0.4, 0.6}};
  std::array<double, 4> last_target{};
  std::map<std::pair<int, int>, uint32_t> parameters;
  const rs::Limits limits{-12, 12, -50, 50, -6, 6, -6, 6, 500, 5};
  auto subscriber = node->create_subscription<can_msgs::msg::Frame>("partial_tx", 32,
    [&](can_msgs::msg::Frame::ConstSharedPtr frame) {
      const auto id = frame->id & 0xff;
      const auto type = (frame->id >> 24) & 0x1f;
      if (id < 1 || id > 3 || !connected[id]) {return;}
      const int index = frame->data[0] | (frame->data[1] << 8);
      can_msgs::msg::Frame reply;
      reply.is_extended = true;
      reply.dlc = 8;
      if (type == rs::kTypeWriteParameter) {
        uint32_t value = 0;
        for (int i = 0; i < 4; ++i) {value |= uint32_t(frame->data[4 + i]) << (i * 8);}
        parameters[{id, index}] = value;
        return;
      }
      if (type == rs::kTypeReadParameter) {
        reply.id = (rs::kTypeReadParameter << 24) | (id << 8) | 0xfd;
        reply.data[0] = index & 0xff;
        reply.data[1] = index >> 8;
        const auto value = parameters[{id, index}];
        for (int i = 0; i < 4; ++i) {reply.data[4 + i] = (value >> (i * 8)) & 0xff;}
      } else {
        if (type == rs::kTypeEnable) {modes[id] = rs::kMotorModeRun; ++enable_count[id];}
        else if (type == rs::kTypeStop) {modes[id] = rs::kMotorModeReset;}
        else if (type == rs::kTypeMotionControl) {
          // Zero-gain keepalives are not position commands.
          if (frame->data[4] || frame->data[5]) {
            ++motion_count[id];
            last_target[id] = rs::decode_u16((frame->data[0] << 8) | frame->data[1], -12, 12);
          }
        } else {return;}
        reply.id = (rs::kTypeFeedback << 24) | (modes[id] << 22) | (id << 8) | 0xfd;
        const auto raw = rs::encode_u16(positions[id], -12, 12);
        reply.data[0] = raw >> 8;
        reply.data[1] = raw & 0xff;
        reply.data[2] = reply.data[4] = 0x80;
      }
      publisher->publish(reply);
    });
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  rs::DriverConfiguration config;
  config.settings.transport.node_name = "partial_driver";
  config.settings.transport.context = context;
  config.settings.transport.transmit_topic = "partial_tx";
  config.settings.transport.receive_topic = "partial_rx";
  config.settings.allow_partial_start = true;
  config.settings.managed_command_gate = managed;
  config.settings.feedback_timeout = 100ms;
  config.settings.recovery_retry_interval = 5ms;
  config.settings.startup_confirmation_timeout = 100ms;
  config.settings.stop_confirmation_timeout = 0ms;
  for (int id = 1; id <= 3; ++id) {
    rs::JointData joint;
    joint.name = "joint" + std::to_string(id);
    joint.can_id = id;
    joint.can_timeout_ticks = 4000;
    joint.limits = limits;
    joint.command_limits = {-2, 2, -10, 10, -3, 3};
    joint.kp = 20;
    config.joints.push_back(joint);
  }
  rs::RobStrideDriver driver(rclcpp::get_logger("partial_test"));
  ASSERT_TRUE(driver.initialize(config));
  ASSERT_TRUE(driver.open());
  ASSERT_TRUE(driver.start());
  ASSERT_TRUE(driver.apply_command_modes({{true, false, false}, {true, false, false},
    {true, false, false}}));
  auto cycle = [&]() {
      executor.spin_some();
      EXPECT_TRUE(driver.update_state());
      EXPECT_TRUE(driver.send_commands());
      std::this_thread::sleep_for(2ms);
    };
  auto wait = [&](auto predicate) {
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      while (!predicate() && std::chrono::steady_clock::now() < deadline) {cycle();}
      return predicate();
    };
  auto handshake = [&]() {
      ASSERT_TRUE(driver.apply_command_modes({{}, {}, {}}));
      ASSERT_TRUE(wait([&]() {return driver.command_gate_status()[2] == 1;}));
      ASSERT_TRUE(driver.apply_command_modes({{true, false, false}, {true, false, false},
        {true, false, false}}));
      const auto epoch = driver.command_gate_status()[0];
      driver.release_command_gate(epoch);
      ASSERT_TRUE(wait([&]() {return driver.command_gate_status()[1] == 0;}));
    };
  // An active component alone must not enable motors without a recovery request.
  for (int i = 0; i < 10; ++i) {cycle();}
  EXPECT_EQ(enable_count[3], 0);
  driver.request_recovery(0);
  driver.request_recovery(1);
  driver.request_recovery(2);
  ASSERT_TRUE(wait([&]() {return driver.joints()[2].state.ready > 0.5;}));
  EXPECT_EQ(driver.joints()[0].state.ready, 0);
  EXPECT_EQ(driver.joints()[1].state.ready, 0);
  driver.joints()[2].command.position = driver.joints()[2].state.position;
  if (managed) {
    // Even an exactly matching target and a release cannot bypass deactivate/activate.
    const auto epoch = driver.command_gate_status()[0];
    driver.release_command_gate(epoch);
    for (int i = 0; i < 10; ++i) {cycle();}
    EXPECT_EQ(motion_count[3], 0);
    EXPECT_EQ(driver.command_gate_status()[1], 1u);
    handshake();
  }
  const int initial_count = motion_count[3];
  ASSERT_TRUE(wait([&]() {return motion_count[3] > initial_count + 5;}));
  // Bring a peer online, then power it off while axis 3 keeps sending commands.
  connected[1] = true;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready > 0.5;}));
  driver.joints()[0].command.position = driver.joints()[0].state.position;
  if (managed) {handshake();}
  connected[1] = false;
  const int before_disconnect = motion_count[3];
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready == 0;}));
  if (!managed) {EXPECT_GT(motion_count[3], before_disconnect);}
  // A returning peer must not execute the position stored before its power loss.
  connected[1] = true;
  positions[1] = 1.0;
  modes[1] = rs::kMotorModeReset;
  parameters.clear();
  driver.request_recovery(0);
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready > 0.5;}));
  if (managed) {
    const auto epoch = driver.command_gate_status()[0];
    driver.release_command_gate(epoch - 1);
    driver.release_command_gate(epoch);
    const int held = motion_count[3];
    for (int i = 0; i < 10; ++i) {cycle();}
    EXPECT_EQ(motion_count[3], held);
    EXPECT_EQ(driver.command_gate_status()[1], 1u);
    // Complete the actual controller stop/start, but keep an old target in the interface.
    ASSERT_TRUE(driver.apply_command_modes({{}, {}, {}}));
    ASSERT_TRUE(wait([&]() {return driver.command_gate_status()[2] == 1;}));
    ASSERT_TRUE(driver.apply_command_modes({{true, false, false}, {true, false, false},
      {true, false, false}}));
    driver.release_command_gate(epoch);
  }
  driver.joints()[0].command.position = -1.0;
  const int before_stale = motion_count[1];
  for (int i = 0; i < 10; ++i) {cycle();}
  EXPECT_EQ(motion_count[1], before_stale);
  driver.joints()[0].command.position = driver.joints()[0].state.position;
  ASSERT_TRUE(wait([&]() {return motion_count[1] > before_stale;}));
  EXPECT_NEAR(last_target[1], positions[1], 0.001);
  if (managed) {
    // ros2_control can skip the mode-stop callback after a hardware write error.
    // Reactivation must discard the stale driver-side claims and command values.
    driver.stop();
    ASSERT_TRUE(driver.start());
    for (const auto & joint : driver.joints()) {
      EXPECT_FALSE(joint.claimed.position);
      EXPECT_TRUE(std::isnan(joint.command.position));
    }
    EXPECT_EQ(driver.command_gate_status()[1], 1u);
    EXPECT_EQ(driver.command_gate_status()[2], 0u);
  }
  driver.stop();
  driver.close();
  executor.remove_node(node);
  context->shutdown("test complete");
}

TEST(PartialOperation, MissingPeerAndPowerCycleDoNotStopHealthyAxis)
{
  check_partial_operation(false);
}

TEST(PartialOperation, ManagedGateWaitsForControllerStopSyncStartAndFreshCommand)
{
  check_partial_operation(true);
}
