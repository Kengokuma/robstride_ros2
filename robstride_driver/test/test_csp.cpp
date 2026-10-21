#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include <map>
#include <future>
#include <thread>
#include "robstride_driver/driver.hpp"

namespace rs = robstride_driver;
using namespace std::chrono_literals;
namespace
{
uint32_t bits(float value)
{
  uint32_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}
float number(uint32_t raw)
{
  float value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}
}

TEST(Csp, ContinuousPositionHoldingGatingAndPowerRecovery)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("csp_fake_bus", rclcpp::NodeOptions().context(context));
  auto publisher = node->create_publisher<can_msgs::msg::Frame>("csp_rx", 128);
  bool connected = true;
  bool answer_position = true;
  bool nan_position = false;
  bool reject_configuration = false;
  int mode = 0;
  int type1_count = 0;
  int stops = 0;
  int zeros = 0;
  int saves = 0;
  int enables = 0;
  int nonzero_current_writes = 0;
  float position = 12.5f;
  float target = -9.0f;  // Deliberately dangerous stored target.
  std::map<uint16_t, uint32_t> parameters;
  parameters[rs::kIndexPositionReference] = bits(target);
  const double pi = std::acos(-1.0);
  auto feedback = [&]() {
      if (!connected) {return;}
      can_msgs::msg::Frame reply;
      reply.is_extended = true;
      reply.dlc = 8;
      reply.id = (rs::kTypeFeedback << 24) | (mode << 22) | (3 << 8) | 0xfd;
      const double wrapped = position - 8 * pi * std::floor((position + 4 * pi) / (8 * pi));
      const auto raw = rs::encode_u16(wrapped, -4 * pi, 4 * pi);
      reply.data[0] = raw >> 8;
      reply.data[1] = raw & 0xff;
      reply.data[2] = reply.data[4] = 0x80;
      publisher->publish(reply);
    };
  auto timer = node->create_wall_timer(5ms, feedback);
  auto subscriber = node->create_subscription<can_msgs::msg::Frame>("csp_tx", 128,
    [&](can_msgs::msg::Frame::ConstSharedPtr frame) {
      if (!connected || (frame->id & 0xff) != 3) {return;}
      const auto type = (frame->id >> 24) & 0x1f;
      const uint16_t index = frame->data[0] | (frame->data[1] << 8);
      if (type == rs::kTypeMotionControl) {++type1_count;}
      if (type == rs::kTypeStop) {mode = 0; ++stops;}
      if (type == rs::kTypeSetZero) {
        EXPECT_FLOAT_EQ(number(parameters[rs::kIndexCurrentLimit]), 0);
        ++zeros;
        position = target = 0;
      }
      if (type == rs::kTypeSaveParameters) {++saves;}
      if (type == rs::kTypeEnable) {
        ++enables;
        EXPECT_EQ(parameters[rs::kIndexRunMode], 5u);
        EXPECT_FLOAT_EQ(number(parameters[rs::kIndexCurrentLimit]), 0.0f);
        EXPECT_FLOAT_EQ(target, position);  // Seeded before enabling, never old target.
        mode = rs::kMotorModeRun;
      }
      if (type == rs::kTypeWriteParameter) {
        uint32_t raw = 0;
        for (int i = 0; i < 4; ++i) {raw |= uint32_t(frame->data[4 + i]) << (8 * i);}
        if (reject_configuration && index == rs::kIndexRunMode) {return;}
        parameters[index] = raw;
        if (index == rs::kIndexPositionReference) {target = number(raw);}
        if (index == rs::kIndexCurrentLimit && number(raw) > 0) {
          ++nonzero_current_writes;
          EXPECT_TRUE(std::isfinite(target));
          EXPECT_GE(target, -20);
          EXPECT_LE(target, 20);
        }
      }
      if (type == rs::kTypeReadParameter) {
        if (index == rs::kIndexMechPosition && !answer_position) {return;}
        can_msgs::msg::Frame reply;
        reply.id = (rs::kTypeReadParameter << 24) | (3 << 8) | 0xfd;
        reply.is_extended = true;
        reply.dlc = 8;
        reply.data[0] = index & 0xff;
        reply.data[1] = index >> 8;
        const uint32_t raw = index == rs::kIndexMechPosition ?
          bits(nan_position ? std::numeric_limits<float>::quiet_NaN() : position) : parameters[index];
        for (int i = 0; i < 4; ++i) {reply.data[4 + i] = (raw >> (8 * i)) & 0xff;}
        publisher->publish(reply);
      }
      feedback();
    });
  rclcpp::ExecutorOptions options;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(node);
  rs::DriverConfiguration config;
  config.settings.transport.context = context;
  config.settings.transport.node_name = "csp_driver";
  config.settings.transport.transmit_topic = "csp_tx";
  config.settings.transport.receive_topic = "csp_rx";
  config.settings.allow_partial_start = true;
  config.settings.managed_command_gate = true;
  config.settings.recovery_retry_interval = 5ms;
  config.settings.feedback_timeout = 100ms;
  config.settings.startup_confirmation_timeout = 100ms;
  config.settings.stop_confirmation_timeout = 0ms;
  rs::JointData joint;
  joint.name = "arm_1_joint";
  joint.can_id = 3;
  joint.limits = {-4*pi, 4*pi, -20, 20, -60, 60, -60, 60, 5000, 100};
  joint.command_limits = {-20, 20, -20, 20, -60, 60};
  joint.csp = true;
  joint.csp_position_min = -20;
  joint.csp_position_max = 20;
  joint.csp_speed_limit = 0.5;
  joint.csp_current_limit = 1;
  joint.csp_torque_limit = 2;
  config.joints.push_back(joint);
  rs::RobStrideDriver driver(rclcpp::get_logger("csp_test"));
  ASSERT_TRUE(driver.initialize(config));
  ASSERT_TRUE(driver.open());
  ASSERT_TRUE(driver.start());
  auto cycle = [&]() {
      executor.spin_all(2ms);
      EXPECT_TRUE(driver.update_state());
      EXPECT_TRUE(driver.send_commands());
      std::this_thread::sleep_for(2ms);
    };
  auto wait = [&](auto predicate, std::chrono::milliseconds duration = 2s) {
      const auto until = std::chrono::steady_clock::now() + duration;
      while (!predicate() && std::chrono::steady_clock::now() < until) {cycle();}
      return predicate();
    };
  auto handshake = [&]() {
      ASSERT_TRUE(driver.apply_command_modes({{}}));
      ASSERT_TRUE(wait([&]() {return driver.command_gate_status()[2] == 1;}));
      ASSERT_TRUE(driver.apply_command_modes({{true, false, false}}));
      driver.release_command_gate(driver.command_gate_status()[0]);
      ASSERT_TRUE(wait([&]() {return driver.command_gate_status()[1] == 0;}));
    };
  for (int i = 0; i < 10; ++i) {cycle();}
  EXPECT_EQ(enables, 0);
  EXPECT_FALSE(driver.apply_command_modes({{false, true, false}}));
  EXPECT_FALSE(driver.apply_command_modes({{false, false, true}}));
  reject_configuration = true;
  driver.request_recovery(0);
  for (int i = 0; i < 100; ++i) {cycle();}
  EXPECT_EQ(enables, 0);  // A missing/mismatched readback cannot enable CSP.
  reject_configuration = false;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready > 0.5;}));
  EXPECT_FLOAT_EQ(number(parameters[rs::kIndexSpeedLimit]), 0.5);
  EXPECT_FLOAT_EQ(number(parameters[rs::kIndexTorqueLimit]), 2);
  EXPECT_EQ(nonzero_current_writes, 0);
  handshake();
  driver.joints()[0].command.position = 15;
  ASSERT_TRUE(wait([&]() {return target == 15 && nonzero_current_writes > 0;}));
  position = 15;
  ASSERT_TRUE(wait([&]() {return std::abs(driver.joints()[0].state.position - 15) < 1e-5;}));
  EXPECT_NEAR(driver.joints()[0].state.motor_position, 15, 1e-5);
  EXPECT_LT(driver.joints()[0].state.motor_position_wrapped, -10);
  const int stops_before = stops;
  for (double invalid : {21.0, -21.0, std::numeric_limits<double>::quiet_NaN()}) {
    driver.joints()[0].command.position = invalid;
    for (int i = 0; i < 10; ++i) {cycle();}
    EXPECT_FLOAT_EQ(target, 15);  // Invalid input retains last target and current limit.
    EXPECT_FLOAT_EQ(number(parameters[rs::kIndexCurrentLimit]), 1);
    EXPECT_EQ(stops, stops_before);
  }
  driver.joints()[0].command.position = -15;
  ASSERT_TRUE(wait([&]() {return target == -15;}));
  for (float value = 14.5f; value > -15; value -= 0.5f) {
    position = value;
    cycle();
  }
  position = -15;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.position == -15;}));
  EXPECT_GT(driver.joints()[0].state.motor_position_wrapped, 10);
  // Ordinary feedback continues, but NaN/absent mechPos cannot release or sustain the gate.
  nan_position = true;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready == 0;}));
  const auto paused_target = target;
  const auto writes_when_paused = nonzero_current_writes;
  for (int i = 0; i < 10; ++i) {cycle();}
  EXPECT_FLOAT_EQ(target, paused_target);
  EXPECT_EQ(nonzero_current_writes, writes_when_paused);
  EXPECT_EQ(driver.command_gate_status()[1], 1u);
  answer_position = false;
  nan_position = false;
  driver.request_recovery(0);
  for (int i = 0; i < 30; ++i) {cycle();}
  EXPECT_EQ(driver.joints()[0].state.ready, 0);
  answer_position = true;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready > 0.5;}));
  handshake();
  // Power cycle with a different count: never replay the pre-poweroff target.
  connected = false;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready == 0;}));
  mode = 0;
  position = 1;
  parameters.clear();
  target = -18;
  connected = true;
  driver.request_recovery(0);
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.ready > 0.5;}));
  EXPECT_FLOAT_EQ(target, 1);
  handshake();
  ASSERT_TRUE(wait([&]() {return number(parameters[rs::kIndexCurrentLimit]) == 1;}));
  EXPECT_FLOAT_EQ(target, 1);
  EXPECT_EQ(type1_count, 0);
  // Calibration waits for confirmed zero current and new mechPos, then requests save.
  ASSERT_TRUE(driver.apply_command_modes({{}}));
  ASSERT_TRUE(wait([&]() {return driver.command_gate_status()[2] == 1;}));
  std::string zero_error;
  auto zero_result = std::async(std::launch::async, [&]() {
    return driver.set_zero(0, &zero_error);
  });
  ASSERT_TRUE(wait([&]() {return zero_result.wait_for(0ms) == std::future_status::ready;}));
  ASSERT_TRUE(zero_result.get()) << zero_error;
  EXPECT_EQ(zeros, 1);
  EXPECT_EQ(saves, 1);
  EXPECT_EQ(driver.command_gate_status()[1], 1u);
  handshake();
  ASSERT_TRUE(wait([&]() {return number(parameters[rs::kIndexCurrentLimit]) == 1;}));
  EXPECT_FLOAT_EQ(target, 0);
  // A firmware that folds mechPos is not silently accepted as continuous.
  position = -15;
  ASSERT_TRUE(wait([&]() {return driver.joints()[0].state.fault > 0.5;}));
  EXPECT_NEAR(driver.joints()[0].state.position, 0.0, 1e-5);
  const int enables_before_fault = enables;
  driver.request_recovery(0);
  for (int i = 0; i < 30; ++i) {cycle();}
  EXPECT_EQ(enables, enables_before_fault);
  EXPECT_EQ(driver.command_gate_status()[1], 1u);
  driver.stop();
  executor.spin_all(10ms);
  EXPECT_EQ(type1_count, 0);
  EXPECT_FLOAT_EQ(number(parameters[rs::kIndexCurrentLimit]), 0);
  driver.close();
  executor.remove_node(node);
  context->shutdown("test complete");
}
