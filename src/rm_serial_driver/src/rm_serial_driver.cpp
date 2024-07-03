// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensebeetle_interfaces/msg/detail/twist_stamped_to_wheel__struct.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <memory>
#include <string>

#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{
    new drivers::serial_driver::SerialDriver(*owned_ctx_),
  },
  custom_qos(10)
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  custom_qos.best_effort();
  getParams();

  //  create publisher
  wheelVel_pub_ = this->create_publisher<sensebeetle_interfaces::msg::TwistStampedToWheel>(
    "wheel_vel", custom_qos);

  // create subscriber
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", custom_qos,
    std::bind(&RMSerialDriver::twistStampedEncodeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "WheelVelocityCalculator has started.");

  wheelVel_sub_ = this->create_subscription<sensebeetle_interfaces::msg::TwistStampedToWheel>(
    "wheel_vel", custom_qos, std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "SendData has started.");

  // open serial port
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // wheelVel pub thread
  publish_thread_ = std::thread([&]() {
    while (rclcpp::ok()) {
      // RCLCPP_INFO(get_logger(), "111111111!");
      if (!wheelVel_queue_.empty()) {
        auto msg = wheelVel_queue_.front();
        wheelVel_pub_->publish(*msg);
        wheelVel_queue_.pop();
      }
    }
  });
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::sendData(const sensebeetle_interfaces::msg::TwistStampedToWheel::SharedPtr msg)
{
  SendPacket packet;

  std::array<char *, 4> send_velocities{
    packet.wheel_1, packet.wheel_2, packet.wheel_3, packet.wheel_4};
  std::array<std::string, 4> wheel_velocities{
    msg->wheel_1, msg->wheel_2, msg->wheel_3, msg->wheel_4};
  try {
    for (uint8_t i = 0; i < 4; ++i) {
      //RCLCPP_INFO(this->get_logger(), "%s", wheel_velocities[i].c_str());
      strncpy(send_velocities[i], wheel_velocities[i].c_str(), sizeof(send_velocities[i]) - 1);
      // RCLCPP_INFO(this->get_logger(), "Wheel %d velocity: %s", i + 1, send_velocities[i]);
    }
    std::vector<uint8_t> data = toVector(packet);
    removeZeros(data);

    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::twistStampedEncodeCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  auto wheel_msg = std::make_shared<sensebeetle_interfaces::msg::TwistStampedToWheel>();

  double vx = static_cast<double>(3) * msg->twist.linear.x;
  double vy = static_cast<double>(3) * msg->twist.linear.y;

  double omega = 0;
  if (msg->twist.angular.z > 0.05) {
    omega = static_cast<double>(10 / 1.57) * msg->twist.angular.z;
  } else if (msg->twist.angular.z < -0.05) {
    omega = static_cast<double>(10 / 1.57) * msg->twist.angular.z;
  }

  // 四个轮子的角度 (45, 135, 225, 315度转为弧度)
  std::array<double, 4> angles = {M_PI / 4, 3 * M_PI / 4, 5 * M_PI / 4, 7 * M_PI / 4};
  std::array<std::string *, 4> wheel_velocities{
    &wheel_msg->wheel_1, &wheel_msg->wheel_2, &wheel_msg->wheel_3, &wheel_msg->wheel_4};

  double r = 0.080923;  // 轮子到中心的距离，根据你的机器人具体修改

  std::array<double, 4> velocities;
  for (int i = 0; i < 4; ++i) {
    if (i > 1) {
      velocities[i] = vx * cos(angles[i]) - vy * sin(angles[i]) + r * omega * sin(angles[i]);
      // RCLCPP_INFO(this->get_logger(), "Wheel %d velocity: %f", i + 1, velocities[i]);
    } else {
      velocities[i] = vx * cos(angles[i]) - vy * sin(angles[i]) - r * omega * sin(angles[i]);
    }

    int velocity_mega = static_cast<int>(std::round(velocities[i] * 1000));
    *wheel_velocities[i] = std::to_string(velocity_mega);
    RCLCPP_INFO(this->get_logger(), "Wheel %d velocity: %f", i + 1, velocities[i]);
  }

  // RCLCPP_INFO(this->get_logger(), "Wheelvelocity: %s", wheel_msg->wheel_1.c_str());
  wheelVel_queue_.push(wheel_msg);
}

void RMSerialDriver::removeZeros(std::vector<uint8_t> & data)
{
  auto newEnd = std::remove(data.begin(), data.end(), static_cast<uint8_t>(0));
  if (newEnd != data.end()) {  // 确保有元素需要被删除
    data.erase(newEnd, data.end());
  }
}

}  // namespace rm_serial_driver
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)