// Copyright (c) 2024 Quinton
// Licensed under the MIT License.

#ifndef DR16_DBUS__DR16_DBUS_HPP_
#define DR16_DBUS__DR16_DBUS_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensebeetle_interfaces/msg/detail/dr16_dbus_data__struct.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensebeetle_interfaces/msg/dr16_dbus_data.hpp"
#include "sensebeetle_interfaces/msg/twist_stamped_to_wheel.hpp"

// C++ system
#include <array>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "dr16_dbus/dbus.hpp"

namespace dr16_dbus
{
class Dr16Dbus : public rclcpp::Node
{
public:
  explicit Dr16Dbus(const rclcpp::NodeOptions & options);
  ~Dr16Dbus() override;

private:
  void getParams();
  void reopenPort();
  void receiveData();
  void unpack();
  void getData(sensebeetle_interfaces::msg::Dr16DbusData dbus_cmd_) const;

  void dbusToTwistStamped();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  std::thread dbus_thread_;

  // publisher
  rclcpp::Publisher<sensebeetle_interfaces::msg::TwistStampedToWheel>::SharedPtr dbus_pub_;

  // QoS
  rclcpp::QoS custom_qos;

  // dr16 cmd
  sensebeetle_interfaces::msg::Dr16DbusData dbus_cmd_;

  // dbus read
  ReceiveDbus receivedbus;
  int16_t buff_[18]{};
  bool is_update_ = false;
  bool is_success{};
};
}  // namespace dr16_dbus

#endif  // DR16_DBUS__DR16_DBUS_HPP_
