// Copyright (c) 2024 Quinton
// Licensed under the MIT License.

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensebeetle_interfaces/msg/detail/twist_stamped_to_wheel__struct.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <unistd.h>

#include <cstdint>
#include <memory>
#include <string>

#include "dr16_dbus/dr16_dbus.hpp"

namespace dr16_dbus
{

Dr16Dbus::Dr16Dbus(const rclcpp::NodeOptions & options)
: Node("dr16_dbus", options),
  owned_ctx_{new IoContext(1)},
  serial_driver_{
    new drivers::serial_driver::SerialDriver(*owned_ctx_),
  },
  custom_qos(10)
{
  RCLCPP_INFO(get_logger(), "Start DbusSerialDriver!");

  custom_qos.best_effort();
  getParams();

  // create publisher
  dbus_pub_ =
    this->create_publisher<sensebeetle_interfaces::msg::TwistStampedToWheel>("cmd_vel", custom_qos);

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

  dbus_thread_ = std::thread([&]() {
    while (rclcpp::ok()) {
      receiveData();
      getData(this->dbus_cmd_);
      dbusToTwistStamped(this->dbus_cmd_);
    }
  });
}

Dr16Dbus::~Dr16Dbus()
{
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void Dr16Dbus::getParams()
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

void Dr16Dbus::reopenPort()
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

void Dr16Dbus::receiveData()
{
  std::vector<uint8_t> data(1);
  while (rclcpp::ok()) {
    uint8_t timeout = 0;
    uint8_t count = 0;
    try {
      while (timeout < 10) {
        size_t n = 0;
        if (serial_driver_->port()->receive(data)) {
          n = 1;
        };
        if (n == 0) {
          timeout++;
        } else if (n == 1) {
          for (uint8_t i = 0; i < 17; i++) {
            buff_[17] = buff_[i + 1];
          }
          buff_[17] = data[0];
        }
      }
      unpack();
      if (count < 17) {
        memset(&receivedbus, 0, sizeof(receivedbus));
        is_update_ = false;
      } else {
        is_update_ = true;
      }

    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void Dr16Dbus::unpack()
{
  receivedbus.ch0 = (buff_[0] | buff_[1] << 8) & 0x07FF;
  receivedbus.ch0 -= 1024;
  receivedbus.ch1 = (buff_[1] >> 3 | buff_[2] << 5) & 0x07FF;
  receivedbus.ch1 -= 1024;
  receivedbus.ch2 = (buff_[2] >> 6 | buff_[3] << 2 | buff_[4] << 10) & 0x07FF;
  receivedbus.ch2 -= 1024;
  receivedbus.ch3 = (buff_[4] >> 1 | buff_[5] << 7) & 0x07FF;
  receivedbus.ch3 -= 1024;
  /* prevent remote control zero deviation */
  if (receivedbus.ch0 <= 10 && receivedbus.ch0 >= -10) receivedbus.ch0 = 0;
  if (receivedbus.ch1 <= 10 && receivedbus.ch1 >= -10) receivedbus.ch1 = 0;
  if (receivedbus.ch2 <= 10 && receivedbus.ch2 >= -10) receivedbus.ch2 = 0;
  if (receivedbus.ch3 <= 10 && receivedbus.ch3 >= -10) receivedbus.ch3 = 0;

  receivedbus.s0 = ((buff_[5] >> 4) & 0x0003);
  receivedbus.s1 = ((buff_[5] >> 4) & 0x000C) >> 2;

  if (
    (abs(receivedbus.ch0) > 660) || (abs(receivedbus.ch1) > 660) || (abs(receivedbus.ch2) > 660) ||
    (abs(receivedbus.ch3) > 660)) {
    is_success = false;
    return;
  }
  receivedbus.x = buff_[6] | (buff_[7] << 8);
  receivedbus.y = buff_[8] | (buff_[9] << 8);
  receivedbus.z = buff_[10] | (buff_[11] << 8);
  receivedbus.l = buff_[12];
  receivedbus.r = buff_[13];
  receivedbus.key = buff_[14] | buff_[15] << 8;  // key board code
  receivedbus.wheel = (buff_[16] | buff_[17] << 8) - 1024;
  is_success = true;
}

void Dr16Dbus::getData(sensebeetle_interfaces::msg::Dr16DbusData dbus_cmd_) const
{
  if (is_success) {
    dbus_cmd_.ch_r_x = static_cast<double>(receivedbus.ch0 / 660.0);
    dbus_cmd_.ch_r_y = static_cast<double>(receivedbus.ch1 / 660.0);
    dbus_cmd_.ch_l_x = static_cast<double>(receivedbus.ch2 / 660.0);
    dbus_cmd_.ch_l_y = static_cast<double>(receivedbus.ch3 / 660.0);
    dbus_cmd_.m_x = static_cast<double>(receivedbus.x / 1600.0);
    dbus_cmd_.m_y = static_cast<double>(receivedbus.y / 1600.0);
    dbus_cmd_.m_z = static_cast<double>(receivedbus.z / 1600.0);
    dbus_cmd_.wheel = static_cast<double>(receivedbus.wheel / 660.0);

    if (receivedbus.s1 != 0) dbus_cmd_.s_l = receivedbus.s1;
    if (receivedbus.s0 != 0) dbus_cmd_.s_r = receivedbus.s0;
    dbus_cmd_.p_l = receivedbus.l;
    dbus_cmd_.p_r = receivedbus.r;

    dbus_cmd_.key_w = receivedbus.key & 0x01 ? true : false;
    dbus_cmd_.key_s = receivedbus.key & 0x02 ? true : false;
    dbus_cmd_.key_a = receivedbus.key & 0x04 ? true : false;
    dbus_cmd_.key_d = receivedbus.key & 0x08 ? true : false;
    dbus_cmd_.key_shift = receivedbus.key & 0x10 ? true : false;
    dbus_cmd_.key_ctrl = receivedbus.key & 0x20 ? true : false;
    dbus_cmd_.key_q = receivedbus.key & 0x40 ? true : false;
    dbus_cmd_.key_e = receivedbus.key & 0x80 ? true : false;
    dbus_cmd_.key_r = (receivedbus.key >> 8) & 0x01 ? true : false;
    dbus_cmd_.key_f = (receivedbus.key >> 8) & 0x02 ? true : false;
    dbus_cmd_.key_g = (receivedbus.key >> 8) & 0x04 ? true : false;
    dbus_cmd_.key_z = (receivedbus.key >> 8) & 0x08 ? true : false;
    dbus_cmd_.key_x = (receivedbus.key >> 8) & 0x10 ? true : false;
    dbus_cmd_.key_c = (receivedbus.key >> 8) & 0x20 ? true : false;
    dbus_cmd_.key_v = (receivedbus.key >> 8) & 0x40 ? true : false;
    dbus_cmd_.key_b = (receivedbus.key >> 8) & 0x80 ? true : false;
    if (is_update_) dbus_cmd_.header.stamp = rclcpp::Clock().now();
  }
  RCLCPP_INFO(
    this->get_logger(), "%f %f %f %f", dbus_cmd_.ch_r_x, dbus_cmd_.ch_r_y, dbus_cmd_.ch_l_x,
    dbus_cmd_.ch_l_y);
}

void Dr16Dbus::dbusToTwistStamped(sensebeetle_interfaces::msg::Dr16DbusData dbus_cmd_) const {}

}  // namespace dr16_dbus
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(dr16_dbus::Dr16Dbus)