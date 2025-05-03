// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensebeetle_interfaces/msg/detail/twist_stamped_to_wheel__struct.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensebeetle_interfaces/msg/twist_stamped_to_wheel.hpp"

// C++ system
#include <array>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

  // QoS
  // rclcpp::QoS custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_t{
  //   RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  //   10,  // 保持的消息数量
  //   RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE}));

private:
  // send data
  void getParams();
  void reopenPort();
  void sendData(const sensebeetle_interfaces::msg::TwistStampedToWheel::SharedPtr msg);

  // encode
  void TwistEncodeCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // remove null
  void removeZeros(std::vector<uint8_t> & data);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<sensebeetle_interfaces::msg::TwistStampedToWheel>::SharedPtr wheelVel_sub_;

  // publisher
  rclcpp::Publisher<sensebeetle_interfaces::msg::TwistStampedToWheel>::SharedPtr wheelVel_pub_;
  std::queue<sensebeetle_interfaces::msg::TwistStampedToWheel::SharedPtr> wheelVel_queue_;

  rclcpp::QoS custom_qos;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
