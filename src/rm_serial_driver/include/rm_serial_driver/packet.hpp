// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 'p';
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  // pp0,30000,0,0,\r\n
  uint8_t header[3] = "pp";
  uint8_t motor = '1';
  uint8_t seg_0 = ',';
  char wheel_1[7] = "0";
  uint8_t seg_1 = ',';
  char wheel_2[7] = "0";
  uint8_t seg_2 = ',';
  char wheel_3[7] = "0";
  uint8_t seg_3 = ',';
  char wheel_4[7] = "0";
  uint8_t seg_4 = ',';
  uint8_t ender[4] = "\r\n";

} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
